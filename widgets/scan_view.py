#!/usr/bin/env python3
import math
import os
from datetime import datetime
from typing import List, Tuple

from textual.app import ComposeResult
from textual.widget import Widget
from textual.widgets import Label
from textual.reactive import reactive

from rich_pixels import Pixels
from PIL import Image, ImageDraw

# ROS2依存
try:
    import rclpy
    from sensor_msgs.msg import LaserScan
    from ros_utils.ros_node_base import AsyncROSNode
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


WIDGET_META = {
    "id": "scan",
    "title": "Scan View",
    "class_name": "scan",
    "category": "sensor",
    "description": "LiDARスキャンを等倍ピクセルで正方形フルビュー表示します。",
    "order": 40,
}


# --- 1. ピクセル描画専用の内部ウィジェット ---
# (ScanViewの外、同じファイル内に定義)

class _ScanImage(Widget):
    """ScanView専用の内部ウィジェット。ピクセル描画のみを担当。"""
    
    points: List[Tuple[float, float]] = reactive([])
    range_max: float = reactive(30.0)
    _image = None

    DEFAULT_CSS = """
    _ScanImage {
        width: 100%;
        height: 100%;
        align-horizontal: center;
        align-vertical: middle;
    }
    """

    def _make_image(self):
        """正方形ビューで描画（テキスト非表示）"""
        w, h = self.size
        if w < 8 or h < 8:
            return None

        size = min(w, h)
        cx, cy = w / 2, h / 2
        margin = 4
        scale = (size / 2 - margin) / max(self.range_max, 1e-6)
        shift_y = -size * 0.1
        shift_x = 0

        img = Image.new("RGB", (w, h), (0, 0, 0))
        draw = ImageDraw.Draw(img)

        for (x, y) in self.points:
            sx = int(cx + x * scale + shift_x)
            sy = int(cy - y * scale + shift_y)
            if 0 <= sx < w and 0 <= sy < h:
                dist = math.hypot(x, y)
                t = dist / self.range_max
                r = int(255 * t)
                g = int(255 * (1 - abs(0.5 - t) * 2))
                b = int(255 * (1 - t))
                img.putpixel((sx, sy), (r, g, b))
        return img

    def _update_view(self):
        img = self._make_image()
        if img:
            self._image = Pixels.from_image(img)
            self.refresh()

    def render(self):
        return self._image or "No data"

    # points または range_max が親から変更されたら再描画
    def watch_points(self):
        self._update_view()
    
    def watch_range_max(self):
        self._update_view()


# --- 2. ScanView (コンテナウィジェット) ---

class ScanView(Widget):
    """
    LaserScan BEV可視化（コンポジットウィジェット）
    _ScanImage(描画) と Label(Hz) を内包する
    """
    
    # ROS/ダミーから受け取ったデータ (子のウィジェットには渡さない)
    points: List[Tuple[float, float]] = reactive([])
    range_max: float = 30.0
    
    # Hz計算用のリアクティブ変数
    last_updated: datetime | None = reactive(None)
    hz: float = reactive(0.0)

    use_dummy: bool = False

    # <<< 親コンテナとしてのCSS
    DEFAULT_CSS = """
    .scan {
        width: 100%;
        height: 100%;
        /* 中身（_ScanImage）を中央揃え */
        align: center middle; 
    }

    /* Hzラベルを右下にオーバーレイ */
    #scan-hz-label {
        layer: overlay;
        align: right bottom;
        padding: 0 1;
        /* 背景を少し透過（オプション）*/
        background: $panel 50%;
    }
    """

    def __init__(self, topic="/scan", **kwargs):
        super().__init__(**kwargs)
        self.topic = topic
        self.use_dummy = not ROS_AVAILABLE or not os.getenv("ROS_DOMAIN_ID")

    # <<< compose で子ウィジェットを生成
    def compose(self) -> ComposeResult:
        """ピクセル描画ウィジェットとHzラベルを生成"""
        yield _ScanImage(id="scan-image")
        yield Label("-.-- Hz", id="scan-hz-label")

    async def on_mount(self):
        # STALE判定とLabel更新のためにタイマーをセット
        self.set_interval(0.5, self.update_hz_label) 
        
        if self.use_dummy:
            self.set_interval(0.1, self.update_dummy)
        else:
            await self.init_ros()

    # --- ROS/Dummyロジック (変更なし) ---
    async def init_ros(self):
        rclpy.init(args=None)
        self.node = AsyncROSNode("scan_textual_viewer")
        self.node.create_subscription(LaserScan, self.topic, self.on_scan, 10)
        self.run_worker(self.node.spin_async(), exclusive=True)

    def on_scan(self, msg):
        self.range_max = msg.range_max
        self.points = self.polar_to_cartesian(msg.ranges, msg.angle_min, msg.angle_increment)
        self.last_updated = datetime.now()
        # <<< データを子の_ScanImageに渡す
        self.update_child_widgets()

    def polar_to_cartesian(self, ranges, angle_min, angle_increment):
        pts: List[Tuple[float, float]] = []
        for i, r in enumerate(ranges):
            if not (r and r > 0.0 and math.isfinite(r)):
                continue
            a = angle_min + i * angle_increment
            pts.append((r * math.cos(a), r * math.sin(a)))
        return pts

    async def update_dummy(self):
        num = 1080
        self.range_max = 30.0
        angle_min = -math.pi
        angle_inc = (2 * math.pi) / num
        ranges = [max(0.1, (i / (num - 1)) * self.range_max) for i in range(num)]
        self.points = self.polar_to_cartesian(ranges, angle_min, angle_inc)
        self.last_updated = datetime.now()
        # <<< データを子の_ScanImageに渡す
        self.update_child_widgets()

    # --- Hz計算ロジック (変更なし) ---
    def watch_last_updated(self, old_time: datetime | None, new_time: datetime | None):
        """last_updated が更新されたらHzを計算する (EMAで平滑化)"""
        if old_time and new_time:
            dt = (new_time - old_time).total_seconds()
            if dt > 0.001:
                current_hz = 1.0 / dt
                if self.hz == 0.0:
                    self.hz = current_hz
                else:
                    alpha = 0.1
                    self.hz = (alpha * current_hz) + (1.0 - alpha) * self.hz

    # --- 子ウィジェット更新ロジック (新規) ---

    def update_child_widgets(self):
        """スキャンデータを _ScanImage に渡す"""
        try:
            image_view = self.query_one(_ScanImage)
            # リアクティブ変数を更新すると、_ScanImage側でwatchが発動して再描画される
            image_view.points = self.points
            image_view.range_max = self.range_max
        except Exception:
            pass # マウント前

    def update_hz_label(self):
        """HzとSTALE状態を Label に渡す"""
        try:
            hz_label = self.query_one("#scan-hz-label", Label)
        except Exception:
            return # マウント前

        # STALE判定
        is_stale = False
        if self.last_updated:
            elapsed = (datetime.now() - self.last_updated).total_seconds()
            if elapsed > 2.0: # 2秒以上更新がなければSTALE
                is_stale = True
        elif not self.last_updated and not self.use_dummy:
             is_stale = True # データ未受信

        effective_hz = 0.0 if is_stale else self.hz
        hz_text = f"{effective_hz:.1f} Hz"
        
        hz_label.styles.color = "orange" if is_stale else "green"
        hz_label.update(hz_text)


    async def on_unmount(self):
        if not self.use_dummy and hasattr(self, "node"):
            self.node.shutdown()
            rclpy.shutdown()

