#!/usr/bin/env python3
import asyncio
import re
from textual.widget import Widget
from textual.reactive import reactive
from textual import on
from textual.app import ComposeResult
from textual.containers import Vertical
from textual.widgets import Static

WIDGET_META = {
    "id": "net",
    "title": "Network Interface Monitor",
    "class_name": "net",
    "category": "system",
    "description": "NetworkManagerが管理する全インタフェースの状態とIPv4アドレスを表示します。",
    "order": 20,
}

# ==========================================================
# 🌐 NetworkManager情報取得
# ==========================================================
async def get_nmcli_device_info() -> list[dict]:
    """nmcli device show を解析し、全インタフェース情報を返す"""
    proc = await asyncio.create_subprocess_shell(
        "nmcli -t device show",
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.DEVNULL,
    )
    out, _ = await proc.communicate()
    text = out.decode()

    interfaces = []
    current = {}
    for line in text.splitlines():
        if not line.strip():
            if current:
                interfaces.append(current)
                current = {}
            continue
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        key, value = key.strip(), value.strip()
        if key == "GENERAL.DEVICE":
            current["DEVICE"] = value
        elif key == "GENERAL.TYPE":
            current["TYPE"] = value
        elif key == "GENERAL.STATE":
            current["STATE"] = value
        elif key == "IP4.ADDRESS[1]":
            current["IP"] = value.split("/")[0]
        elif key == "GENERAL.CONNECTION":
            current["CONNECTION"] = value

    if current:
        interfaces.append(current)

    # IPv4があるものを先頭へソート
    interfaces.sort(key=lambda i: "IP" in i, reverse=True)
    return interfaces

# ==========================================================
# 🌐 NetworkMonitorView
# ==========================================================
class NetworkMonitorView(Widget):
    """NetworkManagerの全インタフェース監視"""

    interfaces = reactive([])

    def compose(self) -> ComposeResult:
        yield Static("🌐 [b]Network Interface Monitor[/b]", id="net-title")
        self.container = Vertical(id="net-container")
        yield self.container

    async def on_mount(self):
        self.set_interval(10.0, self.update_interfaces)
        await self.update_interfaces()

    async def update_interfaces(self):
        try:
            self.interfaces = await get_nmcli_device_info()
            self.refresh_interface_container()
        except Exception as e:
            self.notify(f"⚠️ Failed to get interfaces: {e}", severity="error")

    def refresh_interface_container(self):
        c = self.query_one("#net-container", Vertical)
        c.remove_children()

        if not self.interfaces:
            c.mount(Static("🚫 No interfaces detected"))
            return

        active = [i for i in self.interfaces if i.get("IP")]
        inactive = [i for i in self.interfaces if not i.get("IP")]

        if active:
            c.mount(Static("🟢 [b]Active Interfaces[/b]", classes="section"))
            for iface in active:
                dev = iface.get("DEVICE", "?")
                typ = iface.get("TYPE", "?")
                ip = iface.get("IP", "N/A")
                conn = iface.get("CONNECTION", "N/A")
                state = iface.get("STATE", "")
                c.mount(Static(f"{dev:10s} [{typ}]  {ip:15s}  ({conn}, {state})"))

        if inactive:
            c.mount(Static("\n⚪ [b]Inactive Interfaces[/b]", classes="section"))
            for iface in inactive:
                dev = iface.get("DEVICE", "?")
                typ = iface.get("TYPE", "?")
                state = iface.get("STATE", "")
                conn = iface.get("CONNECTION", "N/A")
                c.mount(Static(f"{dev:10s} [{typ}]  ({conn}, {state})"))

        self.refresh(layout=True)
