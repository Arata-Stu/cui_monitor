#!/usr/bin/env python3
import asyncio
import psutil
from textual.widget import Widget
from textual.reactive import reactive
from textual import on
from textual.app import ComposeResult
from textual.containers import Vertical, Horizontal
from textual.widgets import Button, Input, Static


WIDGET_META = {
    "id": "net",
    "title": "Network Monitor View (Linux)",
    "class_name": "net",
    "category": "system",
    "description": "nmcliを用いたWi-Fiスキャン・接続・切断を行うネットワークモニタ。",
    "order": 20,
}


# ==========================================================
# 🛰️ RSSIユーティリティ
# ==========================================================
def _rssi_to_bar(rssi: int, width: int = 5) -> str:
    """RSSI値(0〜100)を棒グラフ表示に変換"""
    try:
        rssi = int(rssi)
    except Exception:
        return "▯" * width
    level = max(min(rssi // 20, width), 0)
    return "▮" * level + "▯" * (width - level)


# ==========================================================
# 🌐 Wi-Fiスキャン（nmcli）
# ==========================================================
async def scan_wifi_networks(limit: int = 10):
    """nmcliを使用してWi-Fi一覧を取得"""
    proc = await asyncio.create_subprocess_shell(
        "nmcli -t -f SSID,SIGNAL,SECURITY dev wifi list",
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.DEVNULL,
    )
    out, _ = await proc.communicate()
    lines = out.decode().splitlines()

    networks = []
    for line in lines:
        if not line or line.startswith(":"):
            continue
        parts = line.split(":")
        if len(parts) >= 3:
            ssid, signal, sec = parts[0], parts[1], parts[2]
            if ssid:
                networks.append((ssid, int(signal or 0), sec or "Unknown", False))
    networks.sort(key=lambda x: x[1], reverse=True)
    return networks[:limit]


# ==========================================================
# 🌐 Wi-Fi接続/切断（nmcli）
# ==========================================================
async def connect_wifi(ssid: str, password: str) -> bool:
    """Wi-Fi接続処理"""
    cmd = f"nmcli dev wifi connect '{ssid}' password '{password}'"
    proc = await asyncio.create_subprocess_shell(
        cmd,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )
    _, err = await proc.communicate()
    return proc.returncode == 0 and not err


async def disconnect_wifi() -> bool:
    """Wi-Fi切断処理"""
    cmd = "nmcli con down id $(nmcli -t -f NAME con show --active | head -n 1)"
    proc = await asyncio.create_subprocess_shell(
        cmd,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )
    await proc.communicate()
    return proc.returncode == 0


# ==========================================================
# 🌐 NetworkMonitorView (Textual)
# ==========================================================
class NetworkMonitorView(Widget):
    """Wi-Fiスキャン・接続およびネットワーク状態を監視"""

    wifi_list = reactive([])
    ip_address = reactive("N/A")
    active_ssid = reactive(None)
    scanning_active = reactive(False)

    def compose(self) -> ComposeResult:
        yield Static("🌐 [b]Network Monitor[/b]", id="net-title")
        self.ip_label = Static("IP: [cyan]N/A[/]", id="net-ip")
        yield self.ip_label

        yield Static("📶 [b]Available Wi-Fi Networks[/b]:", id="wifi-title")

        self.wifi_container = Vertical(id="wifi-container")
        yield self.wifi_container

        self.input_password = Input(password=True, placeholder="Enter Wi-Fi password...", id="wifi-pass")
        yield self.input_password

    async def on_mount(self):
        """UI構築完了後の初期化"""
        self.set_interval(5.0, self.update_network)
        asyncio.create_task(self._delayed_start())

    async def _delayed_start(self):
        await asyncio.sleep(0.5)
        await self.update_wifi_list()

    # -----------------------------
    # ネットワーク情報更新
    # -----------------------------
    async def update_network(self):
        """IPアドレス更新"""
        try:
            for iface, addrs in psutil.net_if_addrs().items():
                for a in addrs:
                    if a.family.name == "AF_INET" and not iface.startswith("lo"):
                        self.ip_address = a.address
                        self.ip_label.update(f"IP: [cyan]{self.ip_address}[/]")
                        return
        except Exception:
            self.ip_label.update("IP: [red]N/A[/]")

    async def update_wifi_list(self):
        """Wi-Fiスキャン"""
        self.scanning_active = True
        nets = await scan_wifi_networks()
        self.wifi_list = nets or [("No Network Found", 0, "N/A", False)]
        self.scanning_active = False
        self.refresh_wifi_container()

    def refresh_wifi_container(self):
        """Wi-Fiリスト更新"""
        container = self.query_one("#wifi-container", Vertical)
        container.remove_children()

        for ssid, rssi, sec, connected in self.wifi_list:
            bar = _rssi_to_bar(rssi)
            label = Static(f"{ssid:20s} RSSI: {rssi:>3}% {bar} Sec: {sec}")
            button = Button(
                "Disconnect" if connected else "Connect",
                id=f"{'disconnect' if connected else 'connect'}-{ssid}",
                variant="error" if connected else "success",
                classes="wifi-button"
            )
            row = Horizontal(label, button, classes="wifi-row")
            container.mount(row)

        self.refresh(layout=True)

    # -----------------------------
    # イベント処理
    # -----------------------------
    @on(Button.Pressed)
    async def handle_button(self, event: Button.Pressed):
        btn = event.button
        if btn.id.startswith("connect-"):
            ssid = btn.id.replace("connect-", "")
            self.active_ssid = ssid
            self.notify(f"🔑 Enter password for {ssid}")
            self.input_password.focus()
        elif btn.id.startswith("disconnect-"):
            self.notify("📴 Disconnecting Wi-Fi...")
            await disconnect_wifi()
            await self.update_wifi_list()

    @on(Input.Submitted)
    async def handle_password_submit(self, event: Input.Submitted):
        password = event.value.strip()
        ssid = self.active_ssid
        event.input.value = ""  # 即クリア

        if not password:
            self.notify("⚠️ Password is empty", severity="warning")
            return

        self.notify(f"🔗 Connecting to {ssid}...")
        ok = await connect_wifi(ssid, password)
        if ok:
            self.notify(f"✅ Connected to {ssid}", severity="info")
        else:
            self.notify(f"❌ Failed to connect {ssid}", severity="error")
        await self.update_wifi_list()
