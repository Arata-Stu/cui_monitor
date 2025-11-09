from textual.app import ComposeResult, Screen
from textual.containers import VerticalScroll
from textual.widgets import Button, Static
from widgets import WIDGET_REGISTRY

class WidgetSelectView(Screen):
    """widgets/__init__.py のレジストリから自動生成"""

    def compose(self) -> ComposeResult:
        print("WIDGET_REGISTRY in select view:", list(WIDGET_REGISTRY.keys()))

        print("=== WIDGET_REGISTRY (from WidgetSelectView) ===")
        for wid, meta in WIDGET_REGISTRY.items():
            print(f"- {wid}: {meta['title']}")

        with VerticalScroll(id="select-dialog"):
            yield Static("追加するウィジェットを選択してください", classes="title")
            for wid, meta in WIDGET_REGISTRY.items():
                yield Button(meta["title"], id=f"add-{wid}", variant="primary")
            yield Button("--- Cancel ---", id="cancel", variant="error")


    def on_button_pressed(self, event: Button.Pressed):
        if event.button.id == "cancel":
            self.dismiss(None)
        else:
            widget_type = event.button.id.replace("add-", "")
            self.dismiss(widget_type)
