#!/usr/bin/env python3
import logging
import os
from typing import Optional

from textual.app import App, ComposeResult
from textual.containers import Grid, Horizontal, Vertical
from textual.widgets import Button, TabbedContent, TabPane, Footer

from tab_manager import TabManager
from dashboard_actions import DashboardActions 
from widgets.default_view import DefaultView

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)


# ==========================================================
# 🏎️ メインアプリ
# ==========================================================
class RCDashboard(App, DashboardActions):
    CSS_PATH = [
        "config/css/base.css",
        "config/css/widgets_common.css",
        "config/css/script_launcher.css",
        "config/css/topic_hz.css",
        "config/css/default_view.css"
    ]
    TITLE = "RC Car Dashboard (Tabbed)"
    BINDINGS = [
        ("r", "reload", "Reload Active Tab"),
        ("q", "quit", "Quit"),
        ("t", "add_tab", "Add Tab"),           
        ("a", "add_view", "Add Widget"),       
        ("d", "remove_view", "Remove Widget"), 
    ]

    def __init__(self) -> None:
        super().__init__()
        self.tab_manager = TabManager()
        self.tab_count = 0
        self.tabs_container: Optional[TabbedContent] = None

    # ------------------------------------------------------------
    # レイアウト構築
    # ------------------------------------------------------------
    def compose(self) -> ComposeResult:
        with Vertical(id="root-layout"):
            with TabbedContent(id="tabs") as tabs:
                self.tabs_container = tabs
                with TabPane("Tab 1", id="tab1"):
                    # Grid のクラス指定をこちらに移動
                    with Grid(id="grid-tab1", classes="main-grid-area"):
                        yield DefaultView()
            with Horizontal(id="toolbar"):
                yield Button("➕ Add Tab", id="add-tab", variant="success")
                yield Button("🗂 Load Layout (YAML)", id="load-yaml", variant="primary")
                yield Button("➕ Add Widget ", id="add-view", variant="primary")
                yield Button("➖ Remove Widget ", id="remove-view", variant="warning")
                yield Button("🛑 Quit", id="quit-app", variant="error")

        yield Footer()

    def on_mount(self) -> None:
        # 初期タブ登録
        self.tab_manager.add_tab("tab1", "Tab 1")
        self.tab_count = 1
        tabs = self.query_one("#tabs", TabbedContent)
        if not getattr(tabs, "active", None):
            tabs.active = "tab1"

    # ------------------------------------------------------------
    # 🔹 ボタンイベント (ロジックは Mixin 側を呼び出す)
    # ------------------------------------------------------------
    async def on_button_pressed(self, event: Button.Pressed) -> None:
        btn = event.button.id
        self.log(f"[UI] Button pressed: {btn}")

        if btn in ("quit-app", "quit"):
            self.exit()
            return

        if btn == "load-yaml":
            await self.action_load_yaml()
            return

        if btn == "add-tab":
            await self.action_add_tab()
            return

        if btn in ("add-view", "add-widget"):
            await self.action_add_view()
        elif btn == "remove-view":
            await self.action_remove_view()


# ==========================================================
# 🏁 エントリポイント
# ==========================================================
if __name__ == "__main__":
    os.environ.setdefault("TEXTUAL_DEBUG", "1")
    os.environ.setdefault("TEXTUAL_DEVTOOLS", "1")
    RCDashboard().run()