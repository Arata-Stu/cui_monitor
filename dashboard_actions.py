import asyncio
import logging
import os
import yaml
from typing import Optional, TYPE_CHECKING

from textual_fspicker import FileOpen
from textual.widgets import Collapsible, TabbedContent, TabPane
from textual.containers import Grid
from textual.dom import NoMatches  

from widgets import WIDGET_REGISTRY
from widgets.widget_select_view import WidgetSelectView
from widgets.widget_remove_view import WidgetRemoveView
from widgets.default_view import DefaultView

if TYPE_CHECKING:
    from .main import RCDashboard

logger = logging.getLogger(__name__)


class DashboardActions:
    async def _add_tab(self: "RCDashboard", title: str) -> None:
        tabs = self.tabs_container
        if not tabs:
            self.log.error("[_add_tab] TabbedContent が見つかりません。")
            return

        tab_id = f"tab{self.tab_count + 1}"
        self.tab_count += 1
        self.tab_manager.add_tab(tab_id, title)
        pane = TabPane(title, id=tab_id)
        await tabs.add_pane(pane)

        def mount_contents():
            try:
                grid = Grid(id=f"grid-{tab_id}", classes="main-grid-area")
                pane.mount(grid)
                grid.mount(DefaultView())
                self.log(f"🆕 Tab追加: {title} (id={tab_id})")
            except Exception as e:
                self.log(f"[_add_tab/mount_contents] {e}")

        self.call_after_refresh(mount_contents)
        tabs.active = tab_id
        await asyncio.sleep(0.05)

    async def _load_preset_from_yaml(self: "RCDashboard", yaml_path: str):
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

        try:
            old_tabs = self.tabs_container or self.query_one("#tabs", TabbedContent)
            await old_tabs.remove()
        except Exception:
            pass

        self.tab_manager = type(self.tab_manager)()
        self.tab_count = 0
        root = self.query_one("#root-layout")

        async def rebuild_tabs():
            new_tabs = TabbedContent(id="tabs")
            await root.mount(new_tabs)
            self.tabs_container = new_tabs
            registry_lc = {k.strip().lower(): v for k, v in WIDGET_REGISTRY.items()}
            panes_to_add = []

            for tab_def in tabs_def:
                title = tab_def.get("title", f"Tab {self.tab_count + 1}")
                self.tab_count += 1
                tab_id = f"tab{self.tab_count}"
                self.tab_manager.add_tab(tab_id, title)

                widgets_def = tab_def.get("widgets", [])
                widgets_for_grid = []
                added_any = False

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
                    self.tab_manager.add_widget(tab_id, widget)
                    added_any = True

                if not added_any:
                    widgets_for_grid.append(DefaultView())

                grid = Grid(*widgets_for_grid, id=f"grid-{tab_id}", classes="main-grid-area")
                pane = TabPane(title, grid, id=tab_id)
                panes_to_add.append(pane)

            if panes_to_add:
                for pane in panes_to_add:
                    await new_tabs.add_pane(pane)

            if self.tab_count > 0:
                new_tabs.active = "tab1"

            self.notify(f"✅ YAML '{os.path.basename(yaml_path)}' ロード完了", timeout=3)

        self.call_after_refresh(rebuild_tabs)

    async def _handle_add_view(self: "RCDashboard", tab_id: str, widget_type: Optional[str]) -> None:
        if widget_type:
            await self._create_widget_in_tab(tab_id, widget_type)

    async def _handle_remove_view(self: "RCDashboard", tab_id: str, widget_id: Optional[str]) -> None:
        if not widget_id:
            return

        grid = None
        try:
            grid = self.query_one(f"#grid-{tab_id}")
        except NoMatches as e:
            self.log(f"⚠️ Remove View: Grid #{tab_id} が見つかりません。 {e}")
            return

        try:
            widget = self.query_one(f"#{widget_id}")
        except NoMatches:
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

    async def _create_widget_in_tab(self: "RCDashboard", tab_id: str, widget_type: str) -> None:
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

        grid = None
        for _ in range(20):
            try:
                grid = self.query_one(f"#grid-{tab_id}", Grid)
                break
            except NoMatches:
                await asyncio.sleep(0.05)
        if grid is None:
            self.log(f"[ERROR] grid-{tab_id} が見つからず、ウィジェット {widget_type} を追加できません。")
            return

        for default_view in grid.query("DefaultView"):
            try:
                await default_view.remove()
            except Exception:
                pass

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
            self.log(f"✅ View追加: {title} (id={wid}, tab={tab_id})")
        except Exception as e:
            self.log(f"[safe_mount] 追加エラー: {e}")

    def _get_active_tab_id(self: "RCDashboard") -> Optional[str]:
        try:
            tabs = self.tabs_container or self.query_one("#tabs", TabbedContent)
            active = getattr(tabs, "active", None)
            if not active:
                if self.tab_manager.tabs:
                    return list(self.tab_manager.tabs.keys())[0]
                return None
            return active if isinstance(active, str) else getattr(active, "id", None)
        except NoMatches:
            return None

    async def action_reload(self: "RCDashboard") -> None:
        tab_id = self._get_active_tab_id()
        if not tab_id:
            self.notify("⚠️ アクティブなタブが見つかりません。", severity="warning")
            return

        self.log(f"[Reload] タブ {tab_id} をリセット中...")
        try:
            grid = self.query_one(f"#grid-{tab_id}", Grid)
        except NoMatches as e:
            self.log(f"[Reload] Grid取得失敗: {e}")
            self.notify(f"❌ Gridが見つかりません: {tab_id}", severity="error")
            return

        async def reset_tab():
            try:
                for w in list(grid.children):
                    await w.remove()
                self.tab_manager.clear_widgets(tab_id)
                await grid.mount(DefaultView())
                self.log(f"✅ タブ {tab_id} をリセットしました。")
                self.notify(f"🔁 {tab_id} をリセットしました。", timeout=2)
            except Exception as e:
                self.log(f"[Reload/reset_tab] エラー: {e}")
                self.notify(f"❌ リロード中にエラー: {e}", severity="error")

        self.call_after_refresh(reset_tab)

    async def action_add_tab(self: "RCDashboard") -> None:
        await self._add_tab(f"Tab {self.tab_count + 1}")

    async def action_load_yaml(self: "RCDashboard") -> None:
        if FileOpen is None:
            self.notify("❌ 'textual-fspicker' が未インストールです。", severity="error")
            self.log.error("action_load_yaml: textual-fspicker がインストールされていません。")
            await self._try_fallback_load()
            return

        def handle_file_select(path) -> None:
            try:
                path_str = str(path)
            except Exception:
                path_str = ""

            if not path_str:
                self.notify("ファイル選択がキャンセルされました。", severity="info")
                return

            if not (path_str.lower().endswith(".yaml") or path_str.lower().endswith(".yml")):
                self.notify("⚠️ .yaml または .yml ファイルを選択してください。", severity="warning")
                return

            self.log(f"[Action] YAMLプリセットロード開始: {path_str}")
            self.call_after_refresh(self._load_preset_from_yaml, path_str)

        self.push_screen(
            FileOpen(location="config/preset", title="YAMLレイアウトを選択"),
            handle_file_select
        )

    async def _try_fallback_load(self: "RCDashboard") -> None:
        yaml_path = "config/preset/default_layout.yaml"
        self.notify(f"フォールバック: {yaml_path} をロードします。", severity="warning")
        if not os.path.exists(yaml_path):
            self.notify(f"⚠️ YAMLが見つかりません: {yaml_path}", severity="warning")
            return
        self.log(f"[Fallback] YAMLプリセットロード: {yaml_path}")
        await self._load_preset_from_yaml(yaml_path)

    async def action_add_view(self: "RCDashboard") -> None:
        tab_id = self._get_active_tab_id()
        if not tab_id:
            self.notify("⚠️ アクティブなタブが見つかりません。", severity="warning")
            return
        self.push_screen(
            WidgetSelectView(),
            lambda wid_type: self._handle_add_view(tab_id, wid_type)
        )

    async def action_remove_view(self: "RCDashboard") -> None:
        tab_id = self._get_active_tab_id()
        if not tab_id:
            self.notify("⚠️ アクティブなタブが見つかりません。", severity="warning")
            return

        widgets = self.tab_manager.list_widgets(tab_id)
        if not widgets:
            self.notify("⚠️ このタブには削除できるViewがありません。", severity="warning")
            return

        widget_list = []
        for w in widgets:
            title = getattr(w, "border_title", None)
            if title is None and isinstance(w, Collapsible):
                title = getattr(w, "title", w.id)
            if title is None:
                title = w.id
            widget_list.append((str(title), w.id))

        self.push_screen(
            WidgetRemoveView(widget_list),
            lambda wid: self._handle_remove_view(tab_id, wid)
        )
