#!/usr/bin/env python3
import math
import asyncio
import os
import random
from datetime import datetime
from typing import List, Tuple

from textual.widget import Widget
from textual.reactive import reactive

# ✅ 旧Textual対応: rich-pixels使用
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
    "description": "LiDARや距離センサのスキャンデータを画像レンダリングして表示します。",
    "order": 40,
}


class ScanView(Widget):
    """LaserScan BEV可視化 (ROS2対応 + Dummy fallback)"""

    points: List[Tuple[float, float]] = reactive([])
    range_max: float = 10.0
    use_dummy: bool = False
    last_updated: datetime | None = None
    _image = None

    def __init__(self, topic="/scan", **kwargs):
        super().__init__(**kwargs)
        self.topic = topic
        self.use_dummy = not ROS_AVAILABLE or not os.getenv("ROS_DOMAIN_ID")

    async def on_mount(self):
        """起動時にスキャン購読またはダミー生成"""
        if self.use_dummy:
            self.set_interval(0.1, self.update_dummy)
        else:
            await self.init_ros()

    async def init_ros(self):
        """ROS2ノード初期化 & 購読設定"""
        rclpy.init(args=None)
        self.node = AsyncROSNode("scan_textual_viewer")

        def callback(msg):
            self.on_scan(msg)

        self.node.create_subscription(LaserScan, self.topic, callback, 10)
        self.run_worker(self.node.spin_async(), exclusive=True)

    def on_scan(self, msg):
        """LaserScanコールバック"""
        ranges = msg.ranges
        self.range_max = msg.range_max
        self.points = self.polar_to_cartesian(ranges, msg.angle_min, msg.angle_increment)
        self.last_updated = datetime.now()
        self._update_view()

    def polar_to_cartesian(self, ranges, angle_min, angle_increment):
        """極座標をXYへ変換"""
        points = []
        for i, r in enumerate(ranges):
            if math.isinf(r) or math.isnan(r) or r <= 0.0:
                continue
            angle = angle_min + i * angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append((x, y))
        return points

    async def update_dummy(self):
        """ダミーデータ生成 (mac用)"""
        ranges = []
        for i in range(180):
            r = 5.0 + random.uniform(-1.0, 1.0) + 1.5 * math.sin(i / 10.0)
            r = max(0.5, min(r, 10.0))
            ranges.append(r)
        self.points = self.polar_to_cartesian(ranges, -math.pi / 2, (math.pi) / 180)
        self.last_updated = datetime.now()
        self._update_view()

    def _make_image(self):
        """点群をPillowで描画"""
        width, height = self.size
        if width < 8 or height < 8:
            return None

        img = Image.new("RGB", (width, height), (0, 0, 0))
        draw = ImageDraw.Draw(img)
        cx, cy = width / 2, height / 2
        scale = (min(width, height) / 2) / max(self.range_max, 1e-6)

        # 原点描画
        draw.ellipse((cx - 2, cy - 2, cx + 2, cy + 2), fill=(0, 255, 0))

        # 軸線
        draw.line((cx, 0, cx, height), fill=(40, 40, 40))
        draw.line((0, cy, width, cy), fill=(40, 40, 40))

        # 点群
        for (x, y) in self.points:
            sx = cx + x * scale
            sy = cy - y * scale
            if 0 <= sx < width and 0 <= sy < height:
                dist = math.sqrt(x ** 2 + y ** 2)
                c = max(0, min(255, int(255 * dist / self.range_max)))
                draw.point((sx, sy), fill=(255 - c, c, 128))

        # タイムスタンプ
        if self.last_updated:
            draw.text((5, 5), f"{'Dummy' if self.use_dummy else self.topic}", fill=(180, 180, 180))
            draw.text((5, 20), self.last_updated.strftime("%H:%M:%S"), fill=(100, 200, 100))

        return img

    def _update_view(self):
        """Pillow→Pixels→Textual更新"""
        img = self._make_image()
        if img:
            self._image = Pixels.from_image(img)
            self.refresh()

    def render(self):
        """rich-pixels互換 render()"""
        return self._image or "No data"

    async def on_unmount(self):
        if not self.use_dummy and hasattr(self, "node"):
            self.node.shutdown()
            rclpy.shutdown()
