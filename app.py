#!/usr/bin/env python3
import os
import yaml
import logging
from textual.app import App, ComposeResult
# 変更: Vertical をインポート
from textual.containers import Grid, Horizontal, Vertical
from textual.widgets import Button, Collapsible
from widgets import WIDGET_REGISTRY
from widgets.widget_select_view import WidgetSelectView
from widgets.widget_remove_view import WidgetRemoveView
from widgets.default_view import DefaultView


# ==========================================================
# 🔧 ログ設定（Textualデバッグも含む）
# ==========================================================
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)


class RCDashboard(App):
    """RC Dashboard (DefaultView + 常時ツールバー + 安全なmount/remove対応)"""

    CSS_PATH = "config/theme.css"
    TITLE = "RC Car Dashboard"
    CONFIG_PATH = "config/default_layout.yaml"

    # ショートカットキー設定
    BINDINGS = [
        ("r", "reload", "Reload Layout"),
        ("q", "quit", "Quit"),
    ]

    def compose(self) -> ComposeResult:
        """全体レイアウト定義"""
        
        # 変更: 全体をVerticalコンテナで囲む
        with Vertical():
            # メイングリッド領域
            with Grid(id="main-grid"):
                yield DefaultView()  # 初期画面

            # 常時表示ツールバー
            with Horizontal(id="toolbar"):
                yield Button("➕ Add Widget", id="show-add-screen", variant="success")
                yield Button("➖ Remove Widget", id="show-remove-screen", variant="warning")
                yield Button("🗂 Load YAML", id="load-yaml", variant="primary")
                yield Button("🛑 Quit", id="quit-app", variant="error")

    async def on_mount(self):
        """アプリ起動時の初期化"""
        self.mounted_widgets = []
        self.widget_counter = 0
        self.log("RCDashboard 起動完了")

    # ==========================================================
    # 🧭 ボタン操作系
    # ==========================================================
    async def on_button_pressed(self, event: Button.Pressed):
        """ツールバーおよびDefaultViewのボタン操作"""
        btn = event.button.id
        self.log(f"[UI] Button pressed: {btn}")

        # --- DefaultView 用ボタン対応 ---
        if btn == "add-widget":
            self.push_screen(WidgetSelectView(), self.handle_widget_select_result)
            return

        elif btn == "load-yaml":
            layout = self._load_default_layout()
            if layout:
                await self._load_layout_widgets(layout)
            else:
                self.notify("⚠️ YAMLファイルが空または存在しません。", severity="warning")
            return

        elif btn == "quit":
            self.exit()
            return

        # --- Toolbar 用ボタン ---
        if btn == "show-add-screen":
            self.push_screen(WidgetSelectView(), self.handle_widget_select_result)

        elif btn == "show-remove-screen":
            if not self.mounted_widgets:
                self.bell()
                return
            widget_list = [(getattr(w, "border_title", w.id), w.id) for w in self.mounted_widgets]
            self.push_screen(WidgetRemoveView(widget_list), self.handle_widget_remove_result)

        elif btn == "quit-app":
            self.exit()


    # ==========================================================
    # 🧩 レイアウト読み込み
    # ==========================================================
    def _load_default_layout(self):
        """YAMLファイルからデフォルトレイアウトを読み込む"""
        if not os.path.exists(self.CONFIG_PATH):
            self.log("⚠️ YAMLファイルが存在しません。")
            return []
        try:
            with open(self.CONFIG_PATH, "r") as f:
                return yaml.safe_load(f).get("widgets", [])
        except Exception as e:
            self.log(f"[YAML ERROR] {e}")
            return []

    async def _load_layout_widgets(self, layout):
        """YAMLの指定に基づいてウィジェットをロード"""
        grid = self.query_one("#main-grid")

        # DefaultViewを削除（安全に実行）
        for default_view in grid.query("DefaultView"):
            await default_view.remove()

        # 各ウィジェットを追加
        for item in layout:
            await self._add_widget_by_type(item["id"])
        self.log("✅ YAMLレイアウトをロードしました。")

    # ==========================================================
    # 🧱 Widget追加・削除系
    # ==========================================================
    async def handle_widget_select_result(self, widget_type: str | None):
        """Addモーダルからの結果"""
        if not widget_type:
            return
        grid = self.query_one("#main-grid")

        # DefaultView削除
        for default_view in grid.query("DefaultView"):
            await default_view.remove()

        await self._add_widget_by_type(widget_type)

    async def handle_widget_remove_result(self, widget_id: str | None):
        """Removeモーダルからの結果"""
        if not widget_id:
            return

        try:
            widget = self.query_one(f"#{widget_id}")

            def safe_remove():
                try:
                    widget.remove()
                    if widget in self.mounted_widgets:
                        self.mounted_widgets.remove(widget)
                    self.log(f"🗑️ ウィジェット削除: {widget_id}")

                    # すべて削除後にDefaultViewを復帰
                    if not self.mounted_widgets:
                        grid = self.query_one("#main-grid")
                        grid.mount(DefaultView())
                except Exception as e:
                    self.log(f"[safe_remove] 削除中エラー: {e}")

            # ✅ removeは描画後に安全実行
            self.call_after_refresh(safe_remove)

        except Exception as e:
            self.log(f"[Remove Error] {e}")

    async def _add_widget_by_type(self, widget_type: str):
        """指定されたタイプのウィジェットを追加"""
        if widget_type not in WIDGET_REGISTRY:
            self.log(f"[Error] 不明なウィジェットタイプ: {widget_type}")
            return

        info = WIDGET_REGISTRY[widget_type]
        WidgetClass = info["class"]
        title = info["title"]
        class_name = info["class_name"]

        self.widget_counter += 1
        wid = f"widget-{self.widget_counter}"
        grid = self.query_one("#main-grid")

        try:
            if widget_type == "param":
                widget = Collapsible(
                    WidgetClass(id=f"param-inner-{self.widget_counter}"),
                    title=f"⚙️ {title}",
                    collapsed=False,
                    id=wid,
                    classes=class_name,
                )
            else:
                widget = WidgetClass(id=wid, classes=class_name)
                widget.border_title = title

            # 安全なmount実行
            def safe_mount():
                try:
                    grid.mount(widget)
                    self.mounted_widgets.append(widget)
                    self.log(f"✅ ウィジェット追加: {title} (ID={wid})")
                except Exception as e:
                    self.log(f"[safe_mount] 追加エラー: {e}")

            self.call_after_refresh(safe_mount)

        except Exception as e:
            self.log(f"[Add Error] {e}")

    # ==========================================================
    # 🧭 Actions
    # ==========================================================
    def action_reload(self):
        """キーバインド 'r' → すべてリセットして DefaultView に戻す"""
        self.log("🔁 Reload triggered: Reset to DefaultView")

        grid = self.query_one("#main-grid")

        # --- DefaultView を削除（もし複数存在しても安全に全削除） ---
        for view in grid.query("DefaultView"):
            try:
                view.remove()
            except Exception as e:
                self.log(f"[Reload] DefaultView remove failed: {e}")

        # --- すべてのウィジェットを削除 ---
        for widget in list(self.mounted_widgets):
            try:
                widget.remove()
            except Exception as e:
                self.log(f"[Reload] Widget remove failed: {e}")
        self.mounted_widgets.clear()

        # --- DefaultView を再マウント ---
        def safe_reset():
            try:
                grid.mount(DefaultView())
                self.log("✅ DefaultView 再マウント完了")
                self.notify("🔁 Reset to DefaultView", timeout=2)
            except Exception as e:
                self.log(f"[Reload] DefaultView mount failed: {e}")

        self.call_after_refresh(safe_reset)


    def action_quit(self):
        """キーバインド 'q' → 終了"""
        self.exit()


# ==========================================================
# 🏁 エントリポイント
# ==========================================================
if __name__ == "__main__":
    import os

    # --- デバッグ用環境変数を有効化 ---
    os.environ.setdefault("TEXTUAL_DEBUG", "1")
    os.environ.setdefault("TEXTUAL_DEVTOOLS", "1")

    # --- 起動 ---
    RCDashboard().run()