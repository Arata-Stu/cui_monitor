from textual.app import ComposeResult
from textual.containers import Vertical, Center
from textual.widgets import Button, Static, Label
from textual import on


WIDGET_META = {
    "id": "default",
    "title": "Default View",
    "class_name": "default",
    "category": "system",
    "description": "初期状態で表示されるホーム画面。",
    "order": 0,
}


class DefaultView(Static):
    """アプリ起動時に表示されるデフォルトビュー"""

    @on(Button.Pressed)
    def forward_button_to_app(self, event: Button.Pressed) -> None:
        """DefaultView内ボタンをAppへ転送"""
        event.stop()
        self.app.post_message(event)

    def compose(self) -> ComposeResult:
        with Center():
            with Vertical(classes="default-card"):
                yield Label("🏎️ RC Dashboard", id="app-title")
                yield Label("ようこそ！初期状態ではウィジェットはまだありません。", id="subtitle")
                yield Button("🗂 Load Layout (YAMLから読み込み)", id="load-yaml", variant="primary")
                yield Button("➕ Add Widget", id="add-widget", variant="success")
                yield Button("🛑 Quit", id="quit", variant="error")
