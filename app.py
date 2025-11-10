#!/usr/bin/env python3
import asyncio
import logging
import os
import yaml
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
                with TabPane("Tab 1", id="tab1"):
                    with Grid(id="grid-tab1"):
                        yield DefaultView()
            with Horizontal(id="toolbar"):
                yield Button("➕ Add Tab", id="add-tab", variant="success")
                yield Button("🗂 Load Layout (YAML)", id="load-yaml", variant="primary")
                yield Button("➕ Add Widget ", id="add-view", variant="primary")
                yield Button("➖ Remove Widget ", id="remove-view", variant="warning")
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
        tabs = self.tabs_container
        tab_id = f"tab{self.tab_count + 1}"
        self.tab_count += 1
        self.tab_manager.add_tab(tab_id, title)

        pane = TabPane(title, id=tab_id)
        
        # add_pane は await が必要
        await tabs.add_pane(pane)

        def mount_contents():
            try:
                # pane オブジェクトを直接参照する
                # vvvvvvvvvvvvvv ここに classes="main-grid-area" を追加 vvvvvvvvvvvvvv
                grid = Grid(id=f"grid-{tab_id}", classes="main-grid-area")
                # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                pane.mount(grid)
                grid.mount(DefaultView())
                self.log(f"🆕 Tab追加: {title} (id={tab_id})")
            except Exception as e:
                self.log(f"[add_tab/mount_contents] {e}")

        self.call_after_refresh(mount_contents)
        tabs.active = tab_id
        await asyncio.sleep(0.05)  # Grid生成完了待ち

    # ------------------------------------------------------------
    # 🔹 YAMLレイアウト読込（遅延構築対策版）
    # ------------------------------------------------------------
    async def _load_preset_from_yaml(self, yaml_path: str):
        """YAMLから安全に全タブを再構築 (修正版)"""
        try:
            with open(yaml_path, "r") as f:
                preset = yaml.safe_load(f)
        except Exception as e:
            self.notify(f"❌ YAML読込失敗: {e}", severity="error")
            return

        tabs_def = preset.get("tabs", [])
        if not tabs_def:
            self.notify("⚠️ YAMLにタブ定義がありません。", severity="warning")
            return

        # 既存タブのクリーンアップ
        try:
            old_tabs = self.tabs_container or self.query_one("#tabs", TabbedContent)
            await old_tabs.remove()
        except Exception:
            pass
        self.tab_manager = TabManager()
        self.tab_count = 0
        root = self.query_one("#root-layout", Vertical)

        # rebuild_tabs はメインの処理を担う非同期関数
        async def rebuild_tabs():
            new_tabs = TabbedContent(id="tabs")
            await root.mount(new_tabs)
            self.tabs_container = new_tabs

            registry_lc = {k.strip().lower(): v for k, v in WIDGET_REGISTRY.items()}
            
            panes_to_add = [] # TabbedContent に一括追加するためのリスト

            # 1. YAML定義をループし、PaneとGridをメモリ上で構築する
            for tab_def in tabs_def:
                title = tab_def.get("title", f"Tab {self.tab_count + 1}")
                self.tab_count += 1
                tab_id = f"tab{self.tab_count}"
                self.tab_manager.add_tab(tab_id, title)
                
                widgets_def = tab_def.get("widgets", [])
                widgets_for_grid = [] # このタブのGridに追加するウィジェットのリスト
                added_any = False

                # 2. Gridに追加するウィジェットのリストを作成
                for w in widgets_def:
                    raw_type = w.get("type")
                    normalized = (raw_type or "").strip().lower()
                    if not normalized or normalized not in registry_lc:
                        self.log(f"[WARN] 未登録 widget: {raw_type}")
                        continue
                    
                    info = registry_lc[normalized]
                    WidgetClass = info["class"]
                    wtitle = info["title"]
                    class_name = info["class_name"]
                    wid = f"{tab_id}-{normalized}-{self.tab_manager.next_counter(tab_id)}"
                    
                    widget = None
                    if normalized == "param":
                        widget = Collapsible(
                            WidgetClass(id=f"param-inner-{wid}"),
                            title=f"⚙️ {wtitle}",
                            collapsed=False,
                            id=wid,
                            classes=class_name,
                        )
                    else:
                        widget = WidgetClass(id=wid, classes=class_name)
                        widget.border_title = wtitle
                    
                    widgets_for_grid.append(widget)
                    self.tab_manager.add_widget(tab_id, widget) # TabManagerには登録
                    added_any = True

                # 3. ウィジェットが一つもなければDefaultViewを追加
                if not added_any:
                    widgets_for_grid.append(DefaultView())

                # 4. ウィジェットリストをアンパック(*args)してGridを初期化
                grid = Grid(*widgets_for_grid, id=f"grid-{tab_id}", classes="main-grid-area")

                # 5. 完成したGridを子としてTabPaneを初期化
                pane = TabPane(title, grid, id=tab_id)
                
                # 6. TabbedContentに追加するPaneのリストに保存
                panes_to_add.append(pane)

            # 7. 構築したすべてのPaneをTabbedContentに一括追加
            if panes_to_add:
                # ★★★ エラー修正 ★★★
                # add_panes は存在しないため、add_pane をループで呼び出す
                for pane in panes_to_add:
                    await new_tabs.add_pane(pane)
            
            # 8. 最後にアクティブタブを設定
            if self.tab_count > 0:
                new_tabs.active = "tab1"
            
            self.notify(f"✅ YAML '{yaml_path}' ロード完了", timeout=3)

        # rebuild_tabs を非同期タスクとして実行
        self.call_after_refresh(lambda: asyncio.create_task(rebuild_tabs()))

    # ------------------------------------------------------------
    # 🔹 ボタンイベント
    # ------------------------------------------------------------
    async def on_button_pressed(self, event: Button.Pressed) -> None:
        btn = event.button.id
        self.log(f"[UI] Button pressed: {btn}")

        if btn in ("quit-app", "quit"):
            self.exit()
            return

        if btn == "load-yaml":
            yaml_path = "config/default_layout.yaml"
            if not os.path.exists(yaml_path):
                self.notify(f"⚠️ YAMLが見つかりません: {yaml_path}", severity="warning")
                return
            self.log(f"[UI] YAMLプリセットロード: {yaml_path}")
            await self._load_preset_from_yaml(yaml_path)
            return

        if btn == "add-tab":
            await self._add_tab(f"Tab {self.tab_count + 1}")
            return

        tab_id = self._get_active_tab_id()
        if not tab_id:
            self.notify("⚠️ アクティブなタブが見つかりません。", severity="warning")
            return

        if btn in ("add-view", "add-widget"):
            self.push_screen(WidgetSelectView(), lambda wid_type: self._handle_add_view(tab_id, wid_type))
        elif btn == "remove-view":
            widgets = self.tab_manager.list_widgets(tab_id)
            if not widgets:
                self.notify("⚠️ このタブには削除できるViewがありません。", severity="warning")
                return
            # ウィジェットのタイトルかIDを取得する
            widget_list = []
            for w in widgets:
                title = getattr(w, "border_title", None) or getattr(w, "title", None) or w.id
                widget_list.append((title, w.id))
            
            self.push_screen(WidgetRemoveView(widget_list), lambda wid: self._handle_remove_view(tab_id, wid))

    # ------------------------------------------------------------
    # 🔹 Add / Remove View
    # ------------------------------------------------------------
    async def _handle_add_view(self, tab_id: str, widget_type: Optional[str]) -> None:
        if widget_type:
            await self._create_widget_in_tab(tab_id, widget_type)

    async def _handle_remove_view(self, tab_id: str, widget_id: Optional[str]) -> None:
        if not widget_id:
            return
        
        # Grid を先に取得
        grid = None
        try:
            grid = self.query_one(f"#grid-{tab_id}")
        except Exception as e:
            self.log(f"⚠️ Remove View: Grid #{tab_id} が見つかりません。 {e}")
            return
            
        # 削除対象ウィジェットを取得
        try:
            widget = self.query_one(f"#{widget_id}")
        except Exception:
            self.log(f"⚠️ Viewが見つからないため内部リストのみ削除: {widget_id}")
            # 内部リストからのみ削除（もし Textual 側で見つからない場合）
            self.tab_manager.tabs[tab_id]["widgets"] = [
                w for w in self.tab_manager.list_widgets(tab_id)
                if getattr(w, "id", None) != widget_id
            ]
            return

        # Textual の DOM から削除
        def safe_remove():
            try:
                widget.remove()
                self.tab_manager.remove_widget(tab_id, widget)
                # ウィジェットが0個になったら DefaultView をマウント
                if not self.tab_manager.list_widgets(tab_id):
                    grid.mount(DefaultView())
                self.log(f"🗑️ View削除: {widget_id} (tab={tab_id})")
            except Exception as e:
                self.log(f"[safe_remove] 削除エラー: {e}")

        self.call_after_refresh(safe_remove)

    # ------------------------------------------------------------
    # 🔹 Widget生成（Grid待機つき）
    # ------------------------------------------------------------
    async def _create_widget_in_tab(self, tab_id: str, widget_type: str) -> None:
        
        normalized_type = widget_type.strip().lower()
        
        if normalized_type in ["default", "defaultview"]:
            self.notify("⚠️ DefaultViewは直接追加できません。", severity="warning")
            return
        
        registry_lc = {k.strip().lower(): v for k, v in WIDGET_REGISTRY.items()}
        if normalized_type not in registry_lc:
            self.log(f"[Error] 不明なウィジェットタイプ: {widget_type}")
            return
        
        info = registry_lc[normalized_type]
        WidgetClass = info["class"]
        title = info["title"]
        class_name = info["class_name"]
        wid = f"{tab_id}-{normalized_type}-{self.tab_manager.next_counter(tab_id)}"

        # Grid を取得（最大1秒待機）
        grid = None
        for _ in range(20):
            try:
                grid = self.query_one(f"#grid-{tab_id}", Grid)
                break
            except Exception:
                await asyncio.sleep(0.05)
        if grid is None:
            self.log(f"[ERROR] grid-{tab_id} が見つからず、ウィジェット {widget_type} を追加できません。")
            return

        # DefaultView を除去
        for default_view in grid.query("DefaultView"):
            try:
                await default_view.remove()
            except Exception:
                pass

        # Widget 生成
        if normalized_type == "param":
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

        try:
            await grid.mount(widget)
            self.tab_manager.add_widget(tab_id, widget)
            self.log(f"✅ View追加: {title} (id={wid}, tab={tab_D})")
        except Exception as e:
            self.log(f"[safe_mount] 追加エラー: {e}")

    # ------------------------------------------------------------
    # 🔹 Helper
    # ------------------------------------------------------------
    def _get_active_tab_id(self) -> Optional[str]:
        try:
            tabs = self.tabs_container or self.query_one("#tabs", TabbedContent)
            active = getattr(tabs, "active", None)
            if not active:
                # アクティブなタブがNoneの場合、管理リストの最初のタブを返す
                if self.tab_manager.tabs:
                    return list(self.tab_manager.tabs.keys())[0]
                return None
            return active if isinstance(active, str) else getattr(active, "id", None)
        except Exception:
            # #tabs が見つからない場合（ロード中など）
            return None
        
    async def action_reload(self) -> None:
        """現在アクティブなタブをリセットして DefaultView に戻す"""
        tab_id = self._get_active_tab_id()
        if not tab_id:
            self.notify("⚠️ アクティブなタブが見つかりません。", severity="warning")
            return

        self.log(f"[Reload] タブ {tab_id} をリセット中...")

        try:
            grid = self.query_one(f"#grid-{tab_id}", Grid)
        except Exception as e:
            self.log(f"[Reload] Grid取得失敗: {e}")
            self.notify(f"❌ Gridが見つかりません: {tab_id}", severity="error")
            return

        async def reset_tab():
            try:
                # 既存ウィジェットを全削除
                for w in list(grid.children):
                    await w.remove()
                self.tab_manager.tabs[tab_id]["widgets"].clear()
                self.tab_manager.tabs[tab_id]["counter"] = 0

                # DefaultViewを追加
                await grid.mount(DefaultView())
                self.log(f"✅ タブ {tab_id} をリセットしました。")
                self.notify(f"🔁 {tab_id} をリセットしました。", timeout=2)
            except Exception as e:
                self.log(f"[Reload/reset_tab] エラー: {e}")
                self.notify(f"❌ リロード中にエラー: {e}", severity="error")

        self.call_after_refresh(lambda: asyncio.create_task(reset_tab()))


# ==========================================================
# 🏁 エントリポイント
# ==========================================================
if __name__ == "__main__":
    os.environ.setdefault("TEXTUAL_DEBUG", "1")
    os.environ.setdefault("TEXTUAL_DEVTOOLS", "1")
    RCDashboard().run()