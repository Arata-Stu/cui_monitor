from textual.app import ComposeResult, Screen
from textual.containers import Vertical
from textual.widgets import Button, Static

class WidgetRemoveView(Screen):
    """削除するウィジェットを選択するためのモーダルスクリーン"""

    def __init__(self, mounted_widgets: list, **kwargs) -> None:
        """
        現在マウントされているウィジェットのリストを受け取る
        
        Args:
            mounted_widgets (list): (border_title, widget_id) のタプルのリスト
        """
        self.mounted_widgets = mounted_widgets
        super().__init__(**kwargs)

    def compose(self) -> ComposeResult:
        with Vertical(id="remove-dialog"):
            yield Static("削除するウィジェットを選択してください", classes="title")
            
            # 受け取ったリストからボタンを動的に生成
            for title, widget_id in self.mounted_widgets:
                # Buttonの表示名(label) と 内部ID(id) を設定
                yield Button(title, id=widget_id, variant="primary")
                
            yield Button("--- Cancel ---", id="cancel", variant="error")

    def on_button_pressed(self, event: Button.Pressed):
        """ボタンが押されたら、対応するID (削除対象のwidget_id) を返してスクリーンを閉じる"""
        if event.button.id == "cancel":
            self.dismiss(None) # Noneを返して閉じる
        else:
            # 押されたボタンのID (e.g., "widget-1") をそのまま返す
            self.dismiss(event.button.id)