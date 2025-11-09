import asyncio, time, random, os
from textual.widget import Widget
from textual.reactive import reactive
from rich.table import Table

# --- ROS import (optional) ---
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.utilities import get_message
    from ros_utils.ros_node_base import AsyncROSNode
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


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
       macOSではダミーデータ生成モード
    """

    topic_stats = reactive({})
    use_dummy = False
    update_interval = 1.0  # 秒

    def __init__(self, topics=None, **kwargs):
        super().__init__(**kwargs)
        self.topics = topics or []
        self.use_dummy = not ROS_AVAILABLE or not os.getenv("ROS_DOMAIN_ID")

    async def on_mount(self):
        """起動時にROS購読またはダミー生成を開始"""
        if self.use_dummy:
            self.set_interval(self.update_interval, self.update_dummy)
        else:
            await self.init_ros()

    # --- ダミーモード ---
    async def update_dummy(self):
        """疑似トピックHzデータを生成"""
        dummy_topics = ["/scan", "/odom", "/imu", "/cmd_vel"]

        for t in dummy_topics:
            if t not in self.topic_stats:
                self.topic_stats[t] = {
                    "msg_type": "std_msgs/msg/Float32",
                    "hz": 0.0,
                    "last_time": time.time(),
                }

            # Hzをランダムに変化
            hz = round(random.uniform(0.0, 20.0), 1)
            self.topic_stats[t]["hz"] = hz
            # 低Hzでも更新があったとみなす
            self.topic_stats[t]["last_time"] = time.time()

        self.refresh()

    # --- ROS初期化 ---
    async def init_ros(self):
        """ROS2ノード作成 + 複数トピック購読"""
        rclpy.init()
        self.node = AsyncROSNode("topic_hz_monitor")
        self.topic_stats = {}

        available = self.node.get_topic_names_and_types()
        for topic_name, msg_types in available:
            if not msg_types:
                continue
            msg_type = msg_types[0]

            try:
                msg_class = get_message(msg_type)
                self.topic_stats[topic_name] = {
                    "msg_type": msg_type,
                    "count": 0,
                    "hz": 0.0,
                    "last_time": time.time(),
                }

                def callback(msg, name=topic_name):
                    st = self.topic_stats[name]
                    now = time.time()
                    dt = now - st["last_time"]
                    st["hz"] = 1.0 / dt if dt > 0 else st["hz"]
                    st["last_time"] = now
                    st["count"] += 1

                self.node.create_subscription(msg_class, topic_name, callback, 10)
            except Exception:
                continue

        self.run_worker(self.node.spin_async(), exclusive=True)
        self.set_interval(self.update_interval, self.refresh)

    # --- 状態判定 ---
    def evaluate_topic_state(self, info):
        """Hz・経過時間に基づきトピック状態を判定"""
        now = time.time()
        age = now - info.get("last_time", 0)
        hz = info.get("hz", 0.0)

        if hz > 1.0 and age < 2.0:
            return "[green]🟢 OK[/green]"
        elif age < 5.0:
            return "[yellow]🟡 Slow[/yellow]"
        else:
            return "[red]🔴 Stalled[/red]"

    # --- 描画 ---
    def render(self):
        table = Table(
            title="📈 Topic HZ Monitor (Dummy)" if self.use_dummy else "📈 Topic HZ Monitor",
            expand=True,
        )
        table.add_column("Topic", style="cyan", no_wrap=True)
        table.add_column("Type", style="magenta")
        table.add_column("Hz", justify="right", style="green")
        table.add_column("State", justify="center")

        if not self.topic_stats:
            table.add_row("-", "-", "-", "-")
        else:
            for name, info in self.topic_stats.items():
                hz_str = f"{info['hz']:.1f}" if "hz" in info else "-"
                state = self.evaluate_topic_state(info)
                table.add_row(name, info["msg_type"], hz_str, state)

        return table

    async def on_unmount(self):
        if not self.use_dummy and hasattr(self, "node"):
            self.node.shutdown()
            rclpy.shutdown()
