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
    "title": "Advanced Network Monitor",
    "class_name": "net",
    "category": "system",
    "description": "複数Wi-Fi/有線インタフェース対応ネットワークモニタ（nmcliベース）",
    "order": 20,
}


# ==========================================================
# 🛰️ RSSIユーティリティ
# ==========================================================
def _rssi_to_bar(rssi: int, width: int = 5) -> str:
    try:
        rssi = int(rssi)
    except Exception:
        return "▯" * width
    level = max(min(rssi // 20, width), 0)
    return "▮" * level + "▯" * (width - level)


# ==========================================================
# 🌐 ネットワーク列挙
# ==========================================================
async def list_interfaces():
    """nmcliからWi-Fiインタフェース一覧を取得"""
    proc = await asyncio.create_subprocess_shell(
        "nmcli -t -f DEVICE,TYPE,STATE dev status",
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.DEVNULL,
    )
    out, _ = await proc.communicate()
    lines = out.decode().splitlines()

    interfaces = {"wifi": [], "ethernet": []}
    for line in lines:
        parts = line.strip().split(":")
        if len(parts) < 3:
            continue
        dev, typ, state = parts[:3]
        if typ == "wifi":
            interfaces["wifi"].append((dev, state))
        elif typ == "ethernet":
            interfaces["ethernet"].append((dev, state))
    return interfaces


# ==========================================================
# 🌐 Wi-Fiスキャン（nmcli）
# ==========================================================
async def scan_wifi_networks(interface: str, limit: int = 10):
    """指定インタフェースでWi-Fi一覧を取得"""
    cmd = f"nmcli -t -f SSID,SIGNAL,SECURITY dev wifi list ifname {interface}"
    proc = await asyncio.create_subprocess_shell(
        cmd,
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


async def connect_wifi(interface: str, ssid: str, password: str) -> bool:
    cmd = f"nmcli dev wifi connect '{ssid}' password '{password}' ifname {interface}"
    proc = await asyncio.create_subprocess_shell(
        cmd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE
    )
    _, err = await proc.communicate()
    return proc.returncode == 0 and not err


async def disconnect_wifi(interface: str) -> bool:
    cmd = f"nmcli dev disconnect {interface}"
    proc = await asyncio.create_subprocess_shell(
        cmd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE
    )
    await proc.communicate()
    return proc.returncode == 0


# ==========================================================
# 🌐 NetworkMonitorView (Textual)
# ==========================================================
class NetworkMonitorView(Widget):
    """複数Wi-Fi/有線インタフェース監視"""

    interfaces = reactive({})
    wifi_lists = reactive({})
    ip_info = reactive({})
    active_iface = reactive(None)
    active_ssid = reactive(None)

    def compose(self) -> ComposeResult:
        yield Static("🌐 [b]Network Monitor[/b]", id="net-title")
        self.ip_label = Static("IP: [cyan]N/A[/]", id="net-ip")
        yield self.ip_label

        self.wifi_container = Vertical(id="wifi-container")
        yield self.wifi_container

        self.input_password = Input(password=True, placeholder="Enter Wi-Fi password...", id="wifi-pass")
        yield self.input_password

    async def on_mount(self):
        self.set_interval(10.0, self.update_status)
        asyncio.create_task(self._delayed_start())

    async def _delayed_start(self):
        await asyncio.sleep(0.5)
        await self.update_status()

    # -----------------------------
    # ネットワーク状態更新
    # -----------------------------
    async def update_status(self):
        self.interfaces = await list_interfaces()
        self.ip_info = self._get_ip_addresses()
        await self.update_wifi_lists()
        self.refresh_wifi_container()

    def _get_ip_addresses(self):
        """インタフェースごとのIPを取得"""
        ip_map = {}
        for iface, addrs in psutil.net_if_addrs().items():
            for a in addrs:
                if a.family.name == "AF_INET":
                    ip_map[iface] = a.address
        return ip_map

    async def update_wifi_lists(self):
        """Wi-Fiインタフェースごとのスキャン"""
        wifi_lists = {}
        for iface, _ in self.interfaces.get("wifi", []):
            wifi_lists[iface] = await scan_wifi_networks(iface)
        self.wifi_lists = wifi_lists

    # -----------------------------
    # UI構築
    # -----------------------------
    def refresh_wifi_container(self):
        container = self.query_one("#wifi-container", Vertical)
        container.remove_children()

        # 有線表示
        eth_list = self.interfaces.get("ethernet", [])
        if eth_list:
            container.mount(Static("🧩 [b]Ethernet Interfaces[/b]", classes="section-title"))
            for iface, state in eth_list:
                ip = self.ip_info.get(iface, "N/A")
                label = Static(f"{iface:10s} [{state}] IP: {ip}")
                container.mount(label)

        # Wi-Fi表示
        wifi_list = self.interfaces.get("wifi", [])
        if wifi_list:
            container.mount(Static("📶 [b]Wi-Fi Interfaces[/b]", classes="section-title"))
            for iface, state in wifi_list:
                ip = self.ip_info.get(iface, "N/A")
                container.mount(Static(f"▶ {iface} [{state}] IP: {ip}"))
                nets = self.wifi_lists.get(iface, [])
                for ssid, rssi, sec, connected in nets:
                    bar = _rssi_to_bar(rssi)
                    label = Static(f"  {ssid:18s} RSSI:{rssi:>3}% {bar} Sec:{sec}")
                    button = Button(
                        "Disconnect" if connected else "Connect",
                        id=f"{'disconnect' if connected else 'connect'}-{iface}-{ssid}",
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
            _, iface, ssid = btn.id.split("-", 2)
            self.active_iface, self.active_ssid = iface, ssid
            self.notify(f"🔑 Enter password for {ssid} ({iface})")
            self.input_password.focus()
        elif btn.id.startswith("disconnect-"):
            _, iface, ssid = btn.id.split("-", 2)
            self.notify(f"📴 Disconnecting {iface} from {ssid}...")
            await disconnect_wifi(iface)
            await self.update_status()

    @on(Input.Submitted)
    async def handle_password_submit(self, event: Input.Submitted):
        password = event.value.strip()
        iface = self.active_iface
        ssid = self.active_ssid
        event.input.value = ""

        if not (iface and ssid):
            self.notify("⚠️ No target selected", severity="warning")
            return
        if not password:
            self.notify("⚠️ Password empty", severity="warning")
            return

        self.notify(f"🔗 Connecting {iface} → {ssid} ...")
        ok = await connect_wifi(iface, ssid, password)
        if ok:
            self.notify(f"✅ Connected to {ssid} on {iface}", severity="info")
        else:
            self.notify(f"❌ Failed to connect {ssid} on {iface}", severity="error")
        await self.update_status()
