#!/usr/bin/env python3
import logging
from typing import Optional
from textual.app import App, ComposeResult
from textual.containers import Grid, Horizontal, Vertical
from textual.widgets import Button, Collapsible, TabbedContent, TabPane

from widgets import WIDGET_REGISTRY
from widgets.widget_select_view import WidgetSelectView
from widgets.widget_remove_view import WidgetRemoveView
from widgets.default_view import DefaultView


# ==========================================================
# 🔧 ログ設定
# ==========================================================
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)


# ==========================================================
# 🗂️ タブ管理
# ==========================================================
class TabManager:
    def __init__(self) -> None:
        self.tabs: dict[str, dict] = {}  # {tab_id: {"title": str, "widgets": [], "counter": int}}

    def add_tab(self, tab_id: str, title: str) -> None:
        self.tabs[tab_id] = {"title": title, "widgets": [], "counter": 0}

    def add_widget(self, tab_id: str, widget) -> None:
        self.tabs[tab_id]["widgets"].append(widget)

    def remove_widget(self, tab_id: str, widget) -> None:
        self.tabs[tab_id]["widgets"] = [w for w in self.tabs[tab_id]["widgets"] if w is not widget]

    def list_widgets(self, tab_id: str):
        return list(self.tabs.get(tab_id, {}).get("widgets", []))

    def next_counter(self, tab_id: str) -> int:
        self.tabs[tab_id]["counter"] += 1
        return self.tabs[tab_id]["counter"]


# ==========================================================
# 🏎️ メインアプリ
# ==========================================================
class RCDashboard(App):
    CSS_PATH = "config/theme.css"
    TITLE = "RC Car Dashboard (Tabbed)"
    BINDINGS = [
        ("r", "reload", "Reload Active Tab"),
        ("q", "quit", "Quit"),
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

                # 初期タブを同期生成
                with TabPane("Tab 1", id="tab1"):
                    with Grid(id="grid-tab1"):
                        yield DefaultView()

            # 共通ツールバー
            with Horizontal(id="toolbar"):
                yield Button("➕ Add Tab", id="add-tab", variant="success")
                yield Button("➕ Add View (Active Tab)", id="add-view", variant="primary")
                yield Button("➖ Remove View (Active Tab)", id="remove-view", variant="warning")
                yield Button("🛑 Quit", id="quit-app", variant="error")

    def on_mount(self) -> None:
        # 初期タブ登録
        self.tab_manager.add_tab("tab1", "Tab 1")
        self.tab_count = 1

        tabs = self.query_one("#tabs", TabbedContent)
        if not getattr(tabs, "active", None):
            tabs.active = "tab1"

    # ------------------------------------------------------------
    # 🔹 タブ追加
    # ------------------------------------------------------------
    async def _add_tab(self, title: str) -> None:
        tabs = self.query_one("#tabs", TabbedContent)
        tab_id = f"tab{self.tab_count + 1}"
        self.tab_count += 1
        self.tab_manager.add_tab(tab_id, title)

        pane = TabPane(title, id=tab_id)
        tabs.add_pane(pane)

        def mount_contents():
            try:
                pane_attached = self.query_one(f"#{tab_id}", TabPane)
                grid = Grid(id=f"grid-{tab_id}")
                pane_attached.mount(grid)
                grid.mount(DefaultView())
                self.log(f"🆕 Tab追加: {title} (id={tab_id})")
            except Exception as e:
                self.log(f"[add_tab/mount_contents] {e}")

        self.call_after_refresh(mount_contents)
        tabs.active = tab_id

    # ------------------------------------------------------------
    # 🔹 ボタンイベント
    # ------------------------------------------------------------
    async def on_button_pressed(self, event: Button.Pressed) -> None:
        btn = event.button.id
        self.log(f"[UI] Button pressed: {btn}")

        # ===== DefaultView内のボタン =====
        if btn == "add-widget":
            tab_id = self._get_active_tab_id()
            if not tab_id:
                self.notify("⚠️ アクティブなタブが見つかりません。", severity="warning")
                return
            self.push_screen(WidgetSelectView(), lambda wid_type: self._handle_add_view(tab_id, wid_type))
            return

        elif btn == "load-yaml":
            self.notify("🗂 YAMLレイアウト読込機能は未実装です。", severity="info")
            return

        elif btn == "quit":
            self.exit()
            return

        # ===== グローバルツールバー =====
        if btn == "quit-app":
            self.exit()
            return

        if btn == "add-tab":
            await self._add_tab(f"Tab {self.tab_count + 1}")
            return

        tab_id = self._get_active_tab_id()
        if not tab_id:
            self.notify("⚠️ アクティブなタブが見つかりません。", severity="warning")
            return

        if btn == "add-view":
            self.push_screen(WidgetSelectView(), lambda wid_type: self._handle_add_view(tab_id, wid_type))
        elif btn == "remove-view":
            widgets = self.tab_manager.list_widgets(tab_id)
            if not widgets:
                self.notify("⚠️ このタブには削除できるViewがありません。", severity="warning")
                return
            widget_list = [(getattr(w, "border_title", w.id), w.id) for w in widgets]
            self.push_screen(WidgetRemoveView(widget_list), lambda wid: self._handle_remove_view(tab_id, wid))

    # ------------------------------------------------------------
    # 🔹 Add / Remove View
    # ------------------------------------------------------------
    async def _handle_add_view(self, tab_id: str, widget_type: Optional[str]) -> None:
        if not widget_type:
            return
        await self._create_widget_in_tab(tab_id, widget_type)

    async def _handle_remove_view(self, tab_id: str, widget_id: Optional[str]) -> None:
        if not widget_id:
            return
        grid = self.query_one(f"#grid-{tab_id}")

        try:
            widget = self.query_one(f"#{widget_id}")
        except Exception:
            self.log(f"⚠️ Viewが見つからないため内部リストのみ削除: {widget_id}")
            self.tab_manager.tabs[tab_id]["widgets"] = [
                w for w in self.tab_manager.list_widgets(tab_id)
                if getattr(w, "id", None) != widget_id
            ]
            return

        def safe_remove():
            try:
                widget.remove()
                self.tab_manager.remove_widget(tab_id, widget)
                if not self.tab_manager.list_widgets(tab_id):
                    grid.mount(DefaultView())
                self.log(f"🗑️ View削除: {widget_id} (tab={tab_id})")
            except Exception as e:
                self.log(f"[safe_remove] 削除エラー: {e}")

        self.call_after_refresh(safe_remove)

    # ------------------------------------------------------------
    # 🔹 Widget生成
    # ------------------------------------------------------------
    async def _create_widget_in_tab(self, tab_id: str, widget_type: str) -> None:
        """指定タブにウィジェットを追加"""
        if widget_type.lower() in ["default", "defaultview"]:
            self.notify("⚠️ DefaultViewは直接追加できません。", severity="warning")
            return

        if widget_type not in WIDGET_REGISTRY:
            self.log(f"[Error] 不明なウィジェットタイプ: {widget_type}")
            return

        info = WIDGET_REGISTRY[widget_type]
        WidgetClass = info["class"]
        title = info["title"]
        class_name = info["class_name"]

        wid = f"{tab_id}-{widget_type}-{self.tab_manager.next_counter(tab_id)}"
        grid = self.query_one(f"#grid-{tab_id}")

        # DefaultView削除
        for default_view in grid.query("DefaultView"):
            await default_view.remove()

        # Widget生成
        if widget_type == "param":
            widget = Collapsible(
                WidgetClass(id=f"param-inner-{wid}"),
                title=f"⚙️ {title}",
                collapsed=False,
                id=wid,
                classes=class_name,
            )
        else:
            widget = WidgetClass(id=wid, classes=class_name)
            widget.border_title = title

        def safe_mount():
            try:
                grid.mount(widget)
                self.tab_manager.add_widget(tab_id, widget)
                self.log(f"✅ View追加: {title} (id={wid}, tab={tab_id})")
            except Exception as e:
                self.log(f"[safe_mount] 追加エラー: {e}")

        self.call_after_refresh(safe_mount)

    # ------------------------------------------------------------
    # 🔹 Actions（修正版）
    # ------------------------------------------------------------
    def action_reload(self) -> None:
        tab_id = self._get_active_tab_id()
        if not tab_id:
            self.bell()
            return

        grid = self.query_one(f"#grid-{tab_id}")

        # 🧩【追加】ScriptLauncherView があれば全停止
        for view in grid.query("ScriptLauncherView"):
            try:
                self.log("🧹 ScriptLauncherView リロード前に全スクリプト停止実行")
                self.call_later(asyncio.create_task, view.stop_all_scripts())
            except Exception as e:
                self.log(f"[WARN] stop_all_scripts呼び出し失敗: {e}")

        # 🩹 DefaultViewを削除（重複防止）
        for default_view in grid.query("DefaultView"):
            default_view.remove()

        # 🧹 grid配下の全Widgetを削除
        for child in list(grid.children):
            try:
                child.remove()
            except Exception as e:
                self.log(f"[Reload] 子要素削除エラー: {e}")

        # 🧭 tab_manager の該当タブの widget list を初期化
        self.tab_manager.tabs[tab_id]["widgets"].clear()

        # 🪄 再描画
        def safe_reset():
            try:
                grid.mount(DefaultView())
                self.notify(f"🔁 Reset {self.tab_manager.tabs[tab_id]['title']}", timeout=2)
            except Exception as e:
                self.log(f"[Reload] DefaultView mount failed: {e}")

        self.call_after_refresh(safe_reset)



    def action_quit(self) -> None:
        self.exit()

    # ------------------------------------------------------------
    # 🔹 Helper
    # ------------------------------------------------------------
    def _get_active_tab_id(self) -> Optional[str]:
        try:
            tabs = self.query_one("#tabs", TabbedContent)
            active = getattr(tabs, "active", None)
            if not active:
                return "tab1" if "tab1" in self.tab_manager.tabs else None
            return active if isinstance(active, str) else getattr(active, "id", None)
        except Exception:
            return None


# ==========================================================
# 🏁 エントリポイント
# ==========================================================
if __name__ == "__main__":
    import os
    os.environ.setdefault("TEXTUAL_DEBUG", "1")
    os.environ.setdefault("TEXTUAL_DEVTOOLS", "1")
    RCDashboard().run()
