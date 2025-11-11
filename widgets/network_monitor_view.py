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
from textual.widgets import Button, Input, Static


WIDGET_META = {
    "id": "net",
    "title": "Network Monitor View",
    "class_name": "net",
    "category": "system",
    "description": "Wi-Fiスキャン・接続およびネットワーク状態を監視します（接続/切断対応）。",
    "order": 20,
}


# ==========================================================
# 🌐 NetworkManager情報取得 (Linux / nmcli)
# (ユーザーから提供された、最初のコードの関数)
# ==========================================================
async def get_nmcli_device_info() -> list[dict]:
    """nmcli device show を解析し、全インタフェース情報を返す (Linux専用)"""
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
# 🛰️ RSSIユーティリティ
# ==========================================================
def _rssi_to_bar(rssi: int, width: int = 5) -> str:
    """RSSI値(-100〜0)を棒グラフ表示に変換"""
    try:
        rssi = int(rssi)
    except Exception:
        return "▯" * width
    level = max(min((rssi + 100) // 10, width), 0)
    return "▮" * level + "▯" * (width - level)


# ==========================================================
# 🧩 現在接続中SSID / 既知ネットワーク取得 (macOS)
# ==========================================================
def _get_current_ssid() -> tuple[str, int, str, bool]:
    """現在接続中SSIDをnetworksetup経由で取得 (macOS)"""
    try:
        proc = subprocess.run(
            ["networksetup", "-getairportnetwork", "en0"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
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
    """過去に接続したSSID一覧を取得 (macOS)"""
    try:
        proc = subprocess.run(
            ["networksetup", "-listpreferredwirelessnetworks", "en0"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
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


# ==========================================================
# 🛰️ CoreWLANスキャン (macOS)
# ==========================================================
def _corewlan_scan_sync():
    """CoreWLANスキャン（macOS用）"""
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


# ==========================================================
# 🧵 macOS/Linux共通スキャンAPI
# ==========================================================
async def scan_wifi_networks():
    """OSごとのWi-Fiスキャン処理"""
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


# ==========================================================
# 🌐 NetworkMonitorView (Textual)
# ==========================================================
class NetworkMonitorView(Widget):
    """Wi-Fiスキャン・接続およびネットワーク状態を監視"""
    # (CSSは省略)

    wifi_list = reactive([])
    ip_addresses = reactive([])
    active_ssid = reactive(None)

    # 🔽 スキャン状態とアニメーション関連の変数を追加
    scanning_active = reactive(False)
    _scan_dots = 0 # アニメーションのドット数

    def compose(self) -> ComposeResult:
        yield Static("🌐 [b]Network Monitor[/b]", id="net-title")
        self.ip_container = Vertical(id="net-ip-container")
        yield self.ip_container
        yield Static("📶 [b]Available Wi-Fi Networks[/b]:", id="wifi-title")
        self.wifi_container = Vertical(id="wifi-container")
        yield self.wifi_container
        self.input_password = Input(password=True, placeholder="Enter Wi-Fi password...", id="wifi-pass")
        yield self.input_password

    async def on_mount(self):
        """UI構築完了後の初期化"""
        self.set_interval(5.0, self.update_network)
        await self.update_network()
        
        # 🔽 スキャンアニメーション用のタイマーを初期化 (最初は停止状態)
        self.scan_anim_timer = self.set_interval(
            0.5, self.animate_scanning, pause=True
        )
        
        asyncio.create_task(self._delayed_start())

    async def _delayed_start(self):
        await asyncio.sleep(0.5)
        await self.update_wifi_list()

    # -----------------------------
    # ネットワーク・スキャン (IPアドレス)
    # -----------------------------
    
    # ... (変更なし: update_network, watch_ip_addresses) ...
    async def update_network(self):
        """OSごとにIPアドレス一覧を更新する"""
        system = platform.system()
        new_ips = [] # (iface, ip) のタプルを格納
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
        """IPアドレスリストの変更を監視し、UIを更新"""
        container = self.query_one("#net-ip-container", Vertical)
        container.remove_children()
        if not new_ips:
            container.mount(Static("IP: [red]N/A (No active IPs)[/]", id="net-ip"))
            return
        for iface, ip in new_ips:
            container.mount(Static(f"{iface:10s} IP: [cyan]{ip}[/]", classes="net-ip-row"))
        self.refresh(layout=True)

    # -----------------------------
    # ネットワーク・スキャン (Wi-Fi)
    # -----------------------------

    # 🔽 新規追加: スキャンアニメーションのロジック
    def animate_scanning(self):
        """スキャン中のアニメーションを更新"""
        self._scan_dots = (self._scan_dots + 1) % 5
        dots = "." * (self._scan_dots + 1)
        try:
            # "wifi-scan-status" IDを持つウィジェットを探して更新
            status_widget = self.query_one("#wifi-scan-status", Static)
            status_widget.update(f"Scanning{dots:5s}") # 5文字幅を確保
        except Exception:
            # スキャン完了と同時にウィジェットが消えるとエラーになるためキャッチ
            self.scan_anim_timer.pause()

    # 🔽 新規追加: scanning_active の変更を監視
    def watch_scanning_active(self, scanning: bool):
        """スキャン状態に応じてUIを変更し、タイマーを制御"""
        container = self.query_one("#wifi-container", Vertical)
        
        if scanning:
            # スキャン開始時
            container.remove_children()
            container.mount(Static("Scanning...", id="wifi-scan-status"))
            self._scan_dots = 0
            self.scan_anim_timer.resume() # アニメーションタイマー再開
        else:
            # スキャン終了時
            self.scan_anim_timer.pause() # アニメーションタイマー停止
            # 結果を表示するためにコンテナをリフレッシュ
            self.refresh_wifi_container()

    # 🔽 修正: スキャン実行ロジック
    async def update_wifi_list(self):
        """Wi-Fiスキャン（実際のOSスキャンを実行）"""
        if self.scanning_active: # 既にスキャン中の場合は実行しない
            return
            
        self.scanning_active = True # 👈 これで watch_scanning_active が起動
        
        try:
            # 実際のOSスキャンを呼び出す
            self.wifi_list = await scan_wifi_networks()
        
        except Exception as e:
            # スキャンに失敗した場合
            self.notify(f"⚠️ Wi-Fi scan failed: {e}", severity="error")
            self.wifi_list = [("Scan Error", 0, str(e), False)]
        
        self.scanning_active = False # 👈 これで watch_scanning_active が起動

    # 🔽 修正: 0件の場合の表示を追加
    def refresh_wifi_container(self):
        """Wi-Fiリストを再構築 (スキャン終了時に呼ばれる)"""
        # スキャン中はこの関数はUIを更新しない
        if self.scanning_active:
            return
            
        container = self.query_one("#wifi-container", Vertical)
        container.remove_children()

        # 🔽 0件だった場合の表示を追加
        if not self.wifi_list:
            container.mount(Static("🚫 No Wi-Fi networks found"))
            self.refresh(layout=True)
            return

        # (↓) 既存のリスト表示処理
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

    # -----------------------------
    # イベント処理
    # -----------------------------
    
    # ... (変更なし: handle_button, handle_password_submit, connect_wifi, disconnect_wifi) ...
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
            self.notify(f"📴 Disconnected from {ssid}")

    @on(Input.Submitted)
    async def handle_password_submit(self, event: Input.Submitted):
        password = event.value.strip()
        ssid = self.active_ssid
        event.input.value = ""
        if not password:
            self.notify("⚠️ Password is empty", severity="warning")
            return
        self.notify(f"🧪 Simulated connection to {ssid} (password={password})", severity="info")
        await self.connect_wifi(ssid, password)

    async def connect_wifi(self, ssid: str, password: str):
        """Wi-Fi接続"""
        self.notify(f"🧪 Simulated connection to {ssid} (password={password})")
        await self.update_wifi_list()

    async def disconnect_wifi(self, ssid: str):
        """Wi-Fi切断"""
        self.notify(f"📴 Disconnected from {ssid}")
        await self.update_wifi_list()

        