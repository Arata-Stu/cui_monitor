import asyncio, time, random, os
import threading # スレッドロックのために追加

# --- Textual imports ---
from textual.app import App, ComposeResult
from textual.widget import Widget
from textual.widgets import Button, DataTable, Label
from textual.containers import Vertical, Horizontal 
from rich.table import Table 

# --- ROS import (常にインポートできる前提) ---
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rosidl_runtime_py.utilities import get_message


WIDGET_META = {
    "id": "hz",
    "title": "Topic HZ View",
    "class_name": "hz",
    "category": "ros",
    "description": "ROSトピックの発行周期 (Hz) をリアルタイムでモニタリングします。",
    "order": 30,
}


class TopicHZView(Widget):
    """ROS2 Topic HZ Monitor (Textual Widget)
       (rclpy がインストールされている環境専用)
       
       「Clear Stalled」ボタンを搭載。
       ROSの全処理を専用ワーカースレッドに分離し、
       スレッドセーフな通信を行います。
    """
    
    update_interval = 1.0  # UIの更新間隔
    
    has_error = False
    error_message = ""

    def __init__(self, topics: list[str] | None = None, **kwargs):
        super().__init__(**kwargs)
        self.topics_to_monitor = topics
        
        self._internal_stats = {} 
        self._stats_lock = threading.Lock()
        
        self._ros_thread_running = False
        self.node = None 
        self.subscriptions = {}
        
        self._clear_stalled_request = False

    # --- ★ 修正点: Vertical でラップし、縦並びを強制 ---
    def compose(self) -> ComposeResult:
        """ウィジェットのUIを構築"""
        with Vertical(): # ウィジェット全体を縦並びレイアウトにする
            # ツールバー
            with Horizontal(id="toolbar", classes="toolbar", align="center middle"):
                yield Label("📈 Topic HZ Monitor", classes="title")
                yield Button("Clear Stalled", id="clear_button", variant="error")

            
            # データテーブル
            yield DataTable(id="hz_table", zebra_stripes=True)


    async def on_mount(self):
        """起動時にROSワーカースレッドを開始し、UIを初期化"""
        
        # --- ★ 修正点: ツールバーとテーブルの高さを明示的に設定 ---
        try:
            # ツールバーの高さを3に固定
            self.query_one("#toolbar").styles.height = 3
            # DataTable が残りの高さをすべて占有 (1fr)
            self.query_one("#hz_table").styles.height = "1fr" 
        except Exception as e:
            self.log(f"WARN: Failed to set layout styles: {e}")
        # --- ここまで ---

        table = self.query_one(DataTable)
        
        # (v_017 で header=True を削除済み)
        table.clear() 
        
        table.add_columns("Topic", "Type", "Hz", "State")
        table.add_row("ROSノードを初期化中...", "-", "-", "-")
        
        await self.init_ros()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "clear_button":
            self.action_clear_stalled()

    def update_ui_from_stats(self):
        table = self.query_one(DataTable)

        if self.has_error:
            table.clear()
            table.add_columns("Error", "Message") 
            table.add_row(f"[red]ROS Error[/red]", f"[red]{self.error_message}[/red]")
            return
            
        with self._stats_lock: 
            stats_copy = {name: data.copy() for name, data in self._internal_stats.items()}

        if table.columns.keys() != ["Topic", "Type", "Hz", "State"]:
             table.clear()
             table.add_columns("Topic", "Type", "Hz", "State")
        else:
             table.clear() 

        if not stats_copy:
            table.add_row("ROSトピックを検索中...", "-", "-", "-")
            return

        sorted_topics = sorted(stats_copy.items())
        now = time.time() 

        for name, info in sorted_topics:
            state, state_style = self.evaluate_topic_state(info, now)
            
            if state == "🔴 Stalled":
                hz_str = "0.0"
            else:
                hz_str = f"{info['hz']:.1f}"
            
            table.add_row(
                f"[{state_style}]{name}[/]", 
                f"[{state_style}]{info['msg_type']}[/]", 
                f"[{state_style}]{hz_str}[/]",
                f"[{state_style}]{state}[/]"
            )

    async def init_ros(self):
        self._ros_thread_running = True
        self.run_worker(
            asyncio.to_thread(self.ros_worker_loop), 
            exclusive=True, 
            group="ros_spinner"
        )
        self.set_interval(self.update_interval, self.update_ui_from_stats)
        
    def action_clear_stalled(self):
        self._clear_stalled_request = True
        self.log("Clear Stalled request received.")

    def _create_callback(self, topic_name):
        def callback(msg):
            with self._stats_lock: 
                if topic_name not in self._internal_stats:
                    return
                stats = self._internal_stats[topic_name]
                stats["count"] += 1
                stats["last_time"] = time.time()
        return callback

    def _cleanup_subscriptions(self, topic_names_to_remove):
        if not hasattr(self, 'node') or not self.node:
             return
        for topic_name in topic_names_to_remove:
            if topic_name in self.subscriptions:
                try:
                    self.node.destroy_subscription(self.subscriptions[topic_name])
                except Exception as e:
                    self.app.call_from_thread(self.log, f"WARN: destroy_subscription failed: {e}")
                del self.subscriptions[topic_name]
            if topic_name in self._internal_stats:
                del self._internal_stats[topic_name]

    def ros_worker_loop(self):
        try:
            if not rclpy.ok(): 
                rclpy.init()
            self.node = Node("topic_hz_monitor") 
            executor = SingleThreadedExecutor()
            executor.add_node(self.node)
        except Exception as e:
            self.app.call_from_thread(setattr, self, "has_error", True)
            self.app.call_from_thread(setattr, self, "error_message", f"ROS 2 の初期化に失敗: {e}")
            return
        
        last_scan_time = 0.0

        while self._ros_thread_running and rclpy.ok():
            now = time.time()
            
            if self._clear_stalled_request:
                with self._stats_lock:
                    stalled_topics = []
                    for name, stats in self._internal_stats.items():
                        age = now - stats.get("last_time", 0)
                        if age > 5.0 or (stats.get("last_time", 0) == 0.0 and age > 1.0): 
                            stalled_topics.append(name)
                    
                    self._cleanup_subscriptions(stalled_topics)
                    self._clear_stalled_request = False
                
                self.app.call_from_thread(self.update_ui_from_stats)
                continue

            if now - last_scan_time > 1.0:
                last_scan_time = now
                try:
                    available_topics_list = self.node.get_topic_names_and_types()
                    with self._stats_lock: 
                        for topic_name, msg_types in available_topics_list:
                            if topic_name not in self._internal_stats:
                                if self.topics_to_monitor and topic_name not in self.topics_to_monitor: continue
                                if not msg_types: continue
                                msg_type = msg_types[0]
                                try:
                                    msg_class = get_message(msg_type)
                                    self._internal_stats[topic_name] = {
                                        "msg_type": msg_type, "count": 0, "last_time": 0.0,   
                                        "prev_count": 0, "prev_time": now, "hz": 0.0,          
                                    }
                                    sub = self.node.create_subscription(
                                        msg_class, topic_name, self._create_callback(topic_name), 10
                                    )
                                    self.subscriptions[topic_name] = sub 
                                except Exception:
                                    continue
                except Exception:
                    pass 

            with self._stats_lock: 
                for name, stats in self._internal_stats.items():
                    dt = now - stats["prev_time"]
                    if dt > 2.0: # 2秒ウィンドウ
                        d_count = stats["count"] - stats["prev_count"]
                        stats["hz"] = d_count / dt
                        stats["prev_count"] = stats["count"]
                        stats["prev_time"] = now

            executor.spin_once(timeout_sec=0.1)
        
        self._cleanup_subscriptions(list(self.subscriptions.keys()))
        if rclpy.ok() and self.node:
            self.node.destroy_node()

    def evaluate_topic_state(self, info, now):
        age = now - info.get("last_time", 0) 
        if info.get("last_time", 0) == 0.0 and age > 1.0:
             return "🔴 Stalled", "red"
        elif age < 2.5: 
            return "🟢 OK", "green"
        elif age < 5.0: 
            return "🟡 Slow", "yellow"
        else: 
            return "🔴 Stalled", "red"

    async def on_unmount(self):
        if hasattr(self, "node"):
            self._ros_thread_running = False
        if rclpy.ok():
             rclpy.shutdown()