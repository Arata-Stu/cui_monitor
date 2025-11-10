
from typing import Any

class TabManager:
    """タブとそのウィジェットの状態を管理するクラス"""
    
    def __init__(self) -> None:
        self.tabs: dict[str, dict] = {}  # {tab_id: {"title": str, "widgets": [], "counter": int}}

    def add_tab(self, tab_id: str, title: str) -> None:
        self.tabs[tab_id] = {"title": title, "widgets": [], "counter": 0}

    def add_widget(self, tab_id: str, widget: Any) -> None:
        if tab_id not in self.tabs:
            self.add_tab(tab_id, f"Tab {tab_id}")
        self.tabs[tab_id]["widgets"].append(widget)

    def remove_widget(self, tab_id: str, widget: Any) -> None:
        if tab_id in self.tabs:
            self.tabs[tab_id]["widgets"] = [
                w for w in self.tabs[tab_id]["widgets"] if w is not widget
            ]

    def list_widgets(self, tab_id: str) -> list[Any]:
        return list(self.tabs.get(tab_id, {}).get("widgets", []))
    
    def clear_widgets(self, tab_id: str) -> None:
        if tab_id in self.tabs:
            self.tabs[tab_id]["widgets"].clear()
            self.tabs[tab_id]["counter"] = 0

    def next_counter(self, tab_id: str) -> int:
        if tab_id not in self.tabs:
            self.add_tab(tab_id, f"Tab {tab_id}")
            
        self.tabs[tab_id]["counter"] += 1
        return self.tabs[tab_id]["counter"]