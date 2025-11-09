import os, glob, asyncio, subprocess
from textual.widget import Widget
from textual.reactive import reactive
from textual.widgets import Static, Select, Button
from textual.containers import Vertical, Horizontal

# --- TextLog が存在しない環境向けフォールバック（Textual 0.6.x対応） ---
try:
    from textual.widgets import TextLog
except ImportError:
    class TextLog(Static):
        """簡易ログウィジェット (Textual 6.x 互換)"""
        def __init__(self, *args, **kwargs):
            super().__init__("", *args, **kwargs)
            self._lines = []

        def write(self, message: str):
            """新しいメッセージを末尾に追加して更新"""
            self._lines.append(str(message))
            # 最新20行のみ保持（無限に増えないように）
            if len(self._lines) > 20:
                self._lines = self._lines[-20:]
            self.update("\n".join(self._lines))
# --- ROS2環境の有無を確認 ---
try:
    import rclpy
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

WIDGET_META = {
    "id": "param",
    "title": "Param Loader",
    "class_name": "param-col",
    "category": "config",
    "description": "設定ファイルやROSパラメータを読み込み、動的に適用します。",
    "order": 50,
}


class ParamLoaderView(Widget):
    """ROS2 Param Loader (ノード選択 + YAMLロード)"""

    selected_node = reactive(None)
    selected_dir = reactive(os.getcwd())
    yaml_files = reactive([])
    nodes = reactive([])
    log_lines = reactive([])

    async def on_mount(self):
        """UI初期化"""
        await self.refresh_node_list()
        await self.refresh_yaml_list()

        # レイアウトを構築
        self.node_select = Select(
            options=[(n, n) for n in self.nodes] or [("🚫 No Nodes", "none")],
            prompt="Select Node",
            id="node_select",
        )
        self.dir_label = Static(f"📂 Dir: {self.selected_dir}")
        self.yaml_select = Select(
            options=[(os.path.basename(y), y) for y in self.yaml_files] or [("No YAML", "none")],
            prompt="Select YAML",
            id="yaml_select",
        )
        self.load_button = Button("💾 Load Parameters", id="load_button", classes="load-btn")
        self.refresh_button = Button("🔄 Refresh", id="refresh_button", classes="refresh-btn")

        self.log_view = TextLog(id="param_log")
        self.log_view.write("[bold cyan]Param Loader initialized.[/bold cyan]")

        # 配置
        await self.mount(
            Vertical(
                Horizontal(self.node_select, self.refresh_button),
                self.dir_label,
                self.yaml_select,
                self.load_button,
                self.log_view,
            )
        )

    async def refresh_node_list(self):
        """ROSノードリストを取得"""
        if ROS_AVAILABLE:
            try:
                result = subprocess.run(
                    ["ros2", "node", "list"], capture_output=True, text=True, check=False
                )
                self.nodes = [n for n in result.stdout.strip().split("\n") if n]
            except Exception:
                self.nodes = []
        else:
            # macOS fallback
            self.nodes = ["/demo_node", "/control_node", "/vision_node"]

    async def refresh_yaml_list(self):
        """カレントディレクトリ内のYAMLファイルを取得"""
        ymls = glob.glob(os.path.join(self.selected_dir, "*.yaml"))
        ymls += glob.glob(os.path.join(self.selected_dir, "*.yml"))
        self.yaml_files = sorted(ymls)

    async def on_button_pressed(self, event: Button.Pressed):
        """ボタン押下処理"""
        if event.button.id == "refresh_button":
            await self.refresh_node_list()
            await self.refresh_yaml_list()
            self.log_view.write("[yellow]Refreshed node and YAML lists.[/yellow]")
            self.node_select.options = [(n, n) for n in self.nodes]
            self.yaml_select.options = [(os.path.basename(y), y) for y in self.yaml_files]
            return

        elif event.button.id == "load_button":
            node = self.node_select.value
            yaml_path = self.yaml_select.value

            # --- 入力チェック ---
            if (
                node in (None, "none", "", getattr(self.node_select, "BLANK", None))
                or yaml_path in (None, "none", "", getattr(self.yaml_select, "BLANK", None))
            ):
                self.log_view.write("[red]⚠ Node or YAML not selected. Please select both.[/red]")
                return

            # --- パスが実在するかチェック（ROSなし環境ではスキップ可） ---
            if isinstance(yaml_path, str) and not os.path.exists(yaml_path):
                self.log_view.write(f"[red]❌ File not found: {yaml_path}[/red]")
                return

            await self.load_param(str(node), str(yaml_path))


    async def load_param(self, node, yaml_path):
        """ros2 param load 実行"""
        try:
            self.log_view.write(f"[bold cyan]⏳ ros2 param load {node} {yaml_path}[/bold cyan]")

            if ROS_AVAILABLE:
                result = subprocess.run(
                    ["ros2", "param", "load", node, yaml_path],
                    capture_output=True, text=True
                )
                if result.returncode == 0:
                    self.log_view.write(f"[green]✅ Success:[/green] {result.stdout.strip()}")
                else:
                    self.log_view.write(f"[red]❌ Failed:[/red] {result.stderr.strip()}")
            else:
                # mac fallback (simulate)
                await asyncio.sleep(0.8)
                base_name = os.path.basename(yaml_path) if isinstance(yaml_path, str) else "(unknown)"
                self.log_view.write(f"[green]✅ (Dummy) Loaded {base_name} for {node}[/green]")

        except Exception as e:
            self.log_view.write(f"[red]❌ Exception: {e}[/red]")