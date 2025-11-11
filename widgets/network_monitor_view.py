#!/usr/bin/env python3
import asyncio
import platform
import psutil
import subprocess
from textual.widget import Widget
from textual.reactive import reactive
from textual import on
from textual.app import ComposeResult
from textual.containers import Vertical, Horizontal
from textual.widgets import Button, Input, Static, Switch


WIDGET_META = {
    "id": "net",
    "title": "Network Monitor View",
    "class_name": "net",
    "category": "system",
    "description": "Wi-Fiスキャン・接続およびネットワーク状態を監視します（接続/切断対応）。",
    "order": 20,
}


async def get_nmcli_device_info() -> list[dict]:
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

    interfaces.sort(key=lambda i: "IP" in i, reverse=True)
    return interfaces


def _rssi_to_bar(rssi: int, width: int = 5) -> str:
    try:
        rssi = int(rssi)
    except Exception:
        return "▯" * width
    level = max(min((rssi + 100) // 10, width), 0)
    return "▮" * level + "▯" * (width - level)


def _get_current_ssid() -> tuple[str, int, str, bool]:
    try:
        proc = subprocess.run(
            ["networksetup", "-getairportnetwork", "en0"],
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            text=True,
            timeout=5,
        )
        out = proc.stdout.strip()
        if "Current Wi-Fi Network:" in out:
            ssid = out.split(":", 1)[1].strip()
            try:
                from CoreWLAN import CWInterface
                iface = CWInterface.interfaceWithName_("en0")
                rssi = iface.rssiValue() or -50
                sec = str(iface.security())
            except Exception:
                rssi, sec = 0, "Unknown"
            return (ssid, rssi, sec, True)
        return ("(not connected)", 0, "N/A", False)
    except Exception as e:
        return ("diagnosing", 0, str(e), False)


def _get_known_networks(limit: int = 5) -> list[tuple[str, int, str, bool]]:
    try:
        proc = subprocess.run(
            ["networksetup", "-listpreferredwirelessnetworks", "en0"],
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.DEVNULL,
            text=True,
        )
        ssids = [
            line.strip()
            for line in proc.stdout.splitlines()
            if line.strip() and not line.startswith("Preferred")
        ]
        return [(s, 0, "history", False) for s in ssids[:limit]]
    except Exception:
        return [("diagnosing", 0, "N/A", False)]


def _corewlan_scan_sync():
    try:
        from CoreWLAN import CWInterface
        iface = CWInterface.interface()
        if not iface:
            return []

        current_ssid = iface.ssid()
        current_rssi = iface.rssiValue() or -100
        networks, _ = iface.scanForNetworksWithName_error_(None, None)

        SEC_MAP = {
            0: "Open", 1: "WEP", 2: "WPA", 4: "WPA2", 8: "DynamicWEP",
            32: "WPA2E", 128: "WPA2", 192: "WPA2E", 4096: "WPA3",
        }

        results = []
        for net in networks:
            ssid = net.ssid() or ""
            if not ssid:
                continue
            rssi = int(net.rssiValue())
            try:
                sec_code = int(net.securityType())
            except Exception:
                sec_code = 0
            sec_label = SEC_MAP.get(sec_code, f"0x{sec_code:X}")
            connected = (ssid == current_ssid)
            results.append((ssid, rssi, sec_label, connected))

        if current_ssid and not any(s == current_ssid for s, *_ in results):
            results.insert(0, (current_ssid, current_rssi, "current", True))

        results.sort(key=lambda x: x[1], reverse=True)
        return results[:10]
    except Exception:
        return []


async def scan_wifi_networks():
    system = platform.system()
    if system == "Linux":
        proc = await asyncio.create_subprocess_shell(
            "nmcli -t -f SSID,SIGNAL,SECURITY dev wifi list",
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.DEVNULL,
        )
        out, _ = await proc.communicate()
        lines = out.decode().splitlines()
        nets = [tuple(l.split(":")) for l in lines if l and not l.startswith(":")]
        nets.sort(key=lambda x: int(x[1]), reverse=True)
        return nets[:10]

    elif system == "Darwin":
        results = await asyncio.to_thread(_corewlan_scan_sync)
        cur = _get_current_ssid()
        if not results:
            if cur[0] != "(not connected)":
                known = _get_known_networks(limit=5)
                return [cur] + known
            else:
                return [("Demo-Network", -55, "WPA2", False)]
        return results


class NetworkMonitorView(Widget):
    wifi_list = reactive([])
    ip_addresses = reactive([])
    active_ssid = reactive(None)
    scanning_active = reactive(False)
    _scan_dots = 0

    is_hotspot_enabled = reactive(False)
    IS_LINUX = platform.system() == "Linux"
    HOTSPOT_SSID = "TextualHotspot"
    HOTSPOT_PASS = "textual123"


    def compose(self) -> ComposeResult:
        yield Static("🌐 [b]Network Monitor[/b]", id="net-title")
        self.ip_container = Vertical(id="net-ip-container")
        yield self.ip_container

        if self.IS_LINUX:
            yield Horizontal(
                Static("Hotspot (Linux):", classes="label"),
                Button("ON", id="hotspot-on", variant="default"),
                Button("OFF", id="hotspot-off", variant="error"),
                id="hotspot-row"
            )

        yield Static("📶 [b]Available Wi-Fi Networks[/b]:", id="wifi-title")
        self.wifi_container = Vertical(id="wifi-container")
        yield self.wifi_container
        self.input_password = Input(password=True, placeholder="Enter Wi-Fi password...", id="wifi-pass")
        yield self.input_password

    async def on_mount(self):
        self.set_interval(5.0, self.update_network)
        await self.update_network()
        
        self.scan_anim_timer = self.set_interval(
            0.5, self.animate_scanning, pause=True
        )
        
        if self.IS_LINUX:
            asyncio.create_task(self.check_hotspot_status())
            
        asyncio.create_task(self._delayed_start())

    async def _delayed_start(self):
        await asyncio.sleep(0.5)
        await self.update_wifi_list()

    async def update_network(self):
        system = platform.system()
        new_ips = []
        try:
            if system == "Linux":
                interfaces = await get_nmcli_device_info()
                for iface in interfaces:
                    if "IP" in iface:
                        new_ips.append((iface.get("DEVICE", "?"), iface.get("IP", "N/A")))
            else:
                for iface, addrs in psutil.net_if_addrs().items():
                    for a in addrs:
                        if a.family.name == "AF_INET" and not iface.startswith("lo"):
                            new_ips.append((iface, a.address))
            if new_ips != self.ip_addresses:
                self.ip_addresses = new_ips
        except Exception as e:
            self.ip_addresses = [("ERROR", str(e))]

    def watch_ip_addresses(self, old_ips: list, new_ips: list):
        container = self.query_one("#net-ip-container", Vertical)
        container.remove_children()
        if not new_ips:
            container.mount(Static("IP: [red]N/A (No active IPs)[/]", id="net-ip"))
            return
        for iface, ip in new_ips:
            container.mount(Static(f"{iface:10s} IP: [cyan]{ip}[/]", classes="net-ip-row"))
        self.refresh(layout=True)

    def watch_is_hotspot_enabled(self, is_enabled: bool):
        if not self.IS_LINUX:
            return
        
        try:
            on_button = self.query_one("#hotspot-on", Button)
            off_button = self.query_one("#hotspot-off", Button)
            
            on_button.variant = "success" if is_enabled else "default"
            off_button.variant = "error" if not is_enabled else "default"
            
        except Exception:
            pass

    async def check_hotspot_status(self):
        if not self.IS_LINUX:
            return

        try:
            proc = await asyncio.create_subprocess_shell(
                "nmcli -t -f TYPE,DEVICE,MODE connection show --active",
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.DEVNULL,
            )
            out, _ = await proc.communicate()
            
            is_active = False
            for line in out.decode().splitlines():
                parts = line.split(":")
                if len(parts) >= 3 and "wifi" in parts[0] and "ap" in parts[2]:
                    is_active = True
                    break
            
            self.is_hotspot_enabled = is_active
            
        except Exception as e:
            self.notify(f"Hotspot check failed: {e}", severity="error")
            self.is_hotspot_enabled = False

    async def run_hotspot_toggle(self, target_state: bool):
        if not self.IS_LINUX:
            return

        if target_state:
            self.notify("Turning hotspot ON...")
            cmd_on = f'nmcli device wifi hotspot ifname "*" ssid "{self.HOTSPOT_SSID}" password "{self.HOTSPOT_PASS}"'
            proc = await asyncio.create_subprocess_shell(
                cmd_on,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
        else:
            self.notify("Turning hotspot OFF...")
            cmd_off = f'nmcli connection down "{self.HOTSPOT_SSID}"'
            proc = await asyncio.create_subprocess_shell(
                cmd_off,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )

        stdout, stderr = await proc.communicate()
        
        if proc.returncode == 0:
            self.notify(f"Hotspot {'ON' if target_state else 'OFF'} success.")
            self.is_hotspot_enabled = target_state
            await asyncio.sleep(2)
            await self.update_network()
        else:
            self.notify(f"Failed to toggle hotspot: {stderr.decode()}", severity="error")

    def animate_scanning(self):
        self._scan_dots = (self._scan_dots + 1) % 5
        dots = "." * (self._scan_dots + 1)
        try:
            status_widget = self.query_one("#wifi-scan-status", Static)
            status_widget.update(f"Scanning{dots:5s}")
        except Exception:
            self.scan_anim_timer.pause()

    def watch_scanning_active(self, scanning: bool):
        container = self.query_one("#wifi-container", Vertical)
        if scanning:
            container.remove_children()
            container.mount(Static("Scanning...", id="wifi-scan-status"))
            self._scan_dots = 0
            self.scan_anim_timer.resume()
        else:
            self.scan_anim_timer.pause()
            self.refresh_wifi_container()

    async def update_wifi_list(self):
        if self.scanning_active:
            return
        self.scanning_active = True
        try:
            self.wifi_list = await scan_wifi_networks()
        except Exception as e:
            self.notify(f"⚠️ Wi-Fi scan failed: {e}", severity="error")
            self.wifi_list = [("Scan Error", 0, str(e), False)]
        self.scanning_active = False

    def refresh_wifi_container(self):
        if self.scanning_active:
            return
            
        container = self.query_one("#wifi-container", Vertical)
        container.remove_children()

        if not self.wifi_list:
            container.mount(Static("🚫 No Wi-Fi networks found"))
            self.refresh(layout=True)
            return

        for ssid, rssi, sec, connected in self.wifi_list:
            bar = _rssi_to_bar(rssi)
            label = Static(f"{ssid:20s} RSSI: {rssi:>4} {bar} Sec: {sec}")
            button = Button(
                "Disconnect" if connected else "Connect",
                id=f"{'disconnect' if connected else 'connect'}-{ssid}",
                variant="error" if connected else "success",
                classes="wifi-button"
            )
            row = Horizontal(label, button, classes="wifi-row")
            container.mount(row)

        self.refresh(layout=True)

    @on(Button.Pressed)
    async def handle_button(self, event: Button.Pressed):
        btn = event.button
        
        if btn.id.startswith("connect-"):
            ssid = btn.id.replace("connect-", "")
            self.active_ssid = ssid
            self.notify(f"🔑 Enter password for {ssid}")
            self.input_password.focus()
        elif btn.id.startswith("disconnect-"):
            ssid = btn.id.replace("disconnect-", "")
            await self.disconnect_wifi(ssid)

        elif btn.id == "hotspot-on":
            if not self.is_hotspot_enabled:
                await self.run_hotspot_toggle(True)
        elif btn.id == "hotspot-off":
            if self.is_hotspot_enabled:
                await self.run_hotspot_toggle(False)

    @on(Input.Submitted)
    async def handle_password_submit(self, event: Input.Submitted):
        password = event.value.strip()
        ssid = self.active_ssid
        event.input.value = ""
        if not password:
            self.notify("⚠️ Password is empty", severity="warning")
            return
        await self.connect_wifi(ssid, password)

    async def connect_wifi(self, ssid: str, password: str):
        self.notify(f"🧪 Simulated connection to {ssid} (password={password})")
        await asyncio.sleep(3)
        await self.update_wifi_list()
        await self.update_network()

    async def disconnect_wifi(self, ssid: str):
        self.notify(f"📴 Disconnected from {ssid}")
        await asyncio.sleep(2)
        await self.update_wifi_list()
        await self.update_network()