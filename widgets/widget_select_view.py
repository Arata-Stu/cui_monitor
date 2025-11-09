from textual.app import ComposeResult, Screen
from textual.containers import VerticalScroll, Horizontal
from textual.widgets import Button, Static
from widgets import WIDGET_REGISTRY


class WidgetSelectView(Screen):
    """Widget選択画面（split/tab の選択付き）"""

    def compose(self) -> ComposeResult:
        with VerticalScroll(id="select-dialog"):
            yield Static("追加するウィジェットを選択してください", classes="title")

            # 各Widgetごとに1行（横並び構成）
            for wid, meta in WIDGET_REGISTRY.items():
                title = meta["title"]
                with Horizontal(classes="widget-select-row"):
                    # 📦 Widget名ラベル（左寄せ）
                    yield Static(f"📦 {title}", classes="widget-name")
                    # Split / Tab ボタン（右側）
                    yield Button("🪟 Split", id=f"add-split-{wid}", variant="primary", classes="split-btn")
                    yield Button("🗂 Tab", id=f"add-tab-{wid}", variant="success", classes="tab-btn")

            # 最後にCancelボタン
            yield Button("--- Cancel ---", id="cancel", variant="error")

    def on_button_pressed(self, event: Button.Pressed):
        bid = event.button.id

        if bid == "cancel":
            self.dismiss(None)
            return

        if bid.startswith("add-split-"):
            widget_type = bid.replace("add-split-", "")
            self.dismiss((widget_type, "split"))

        elif bid.startswith("add-tab-"):
            widget_type = bid.replace("add-tab-", "")
            self.dismiss((widget_type, "tab"))
