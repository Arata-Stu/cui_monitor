import math, asyncio, os, random
from textual.widget import Widget
from textual.reactive import reactive

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
    "description": "LiDARや距離センサのスキャンデータをテキストベースで可視化します。",
    "order": 40,
}

class ScanView(Widget):
    """LaserScan BEV可視化 (ROS2対応 + Dummy fallback)"""

    points = reactive([])
    range_max = 10.0
    use_dummy = False

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
        self.refresh()

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
        self.points = self.polar_to_cartesian(ranges, -math.pi/2, (math.pi)/180)
        self.refresh()

    def render(self):
        """ASCII BEV描画"""
        width, height = self.size
        cx, cy = width // 2, height // 2
        scale = (self.range_max / (min(width, height) / 2)) or 1.0

        grid = [[" " for _ in range(width)] for _ in range(height)]
        if 0 <= cy < height and 0 <= cx < width:
            grid[cy][cx] = "+"

        for (x, y) in self.points:
            sx = int(cx + x / scale)
            sy = int(cy - y / scale)
            if 0 <= sy < height and 0 <= sx < width:
                grid[sy][sx] = "*"

        lines = ["".join(row) for row in grid]
        title = f"📡 [b]LaserScan Viewer[/b] ({'Dummy' if self.use_dummy else self.topic})"
        return title + "\n" + "\n".join(lines)

    async def on_unmount(self):
        if not self.use_dummy and hasattr(self, "node"):
            self.node.shutdown()
            rclpy.shutdown()
