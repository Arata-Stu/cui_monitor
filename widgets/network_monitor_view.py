import psutil, asyncio
from textual.widget import Widget
from textual.reactive import reactive

WIDGET_META = {
    "id": "net",
    "title": "Network Monitor View",
    "class_name": "net",
    "category": "system",
    "description": "ネットワークインタフェースごとの送受信速度、スループットを監視します。",
    "order": 20,
}

def get_active_interfaces():
    """通信可能なインタフェース名を返す（loや仮想IFを除外）"""
    candidates = []
    for iface, addrs in psutil.net_if_addrs().items():
        if iface.startswith(("lo", "gif", "stf", "utun", "anpi", "ap")):
            continue
        if not addrs:
            continue
        stats = psutil.net_io_counters(pernic=True).get(iface)
        if not stats:
            continue
        if stats.bytes_sent > 0 or stats.bytes_recv > 0:
            candidates.append(iface)
    return candidates or ["en0", "eth0"]  # fallback


class NetworkMonitorView(Widget):
    """ネットワーク監視ビュー（安全なキャンセル対応版）"""

    stats = reactive({})
    prev_counters = {}
    active_ifaces = []
    _task = None  # ← 背景タスクハンドル

    async def on_mount(self):
        """初期化および監視開始"""
        self.active_ifaces = get_active_interfaces()
        self.prev_counters = psutil.net_io_counters(pernic=True)
        # 非同期ループをタスクとして起動
        self._task = asyncio.create_task(self._updater_loop())

    async def _updater_loop(self):
        """バックグラウンドで1秒周期更新"""
        try:
            while True:
                await self.update_network()
                await asyncio.sleep(1.0)
        except asyncio.CancelledError:
            # 安全な終了
            self.log("🛑 NetworkMonitorView updater cancelled.")
        except Exception as e:
            self.log(f"[Error] {e}")

    async def on_unmount(self):
        """削除時にバックグラウンドタスクを安全に停止"""
        if self._task and not self._task.done():
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self.log("✅ NetworkMonitorView task stopped.")

    async def update_network(self):
        """ネットワーク統計更新"""
        try:
            new_counters = psutil.net_io_counters(pernic=True)
            diff_stats = {}
            current_ifaces = get_active_interfaces()

            if set(current_ifaces) != set(self.active_ifaces):
                self.active_ifaces = current_ifaces

            for iface in self.active_ifaces:
                if iface not in new_counters or iface not in self.prev_counters:
                    continue
                prev = self.prev_counters[iface]
                data = new_counters[iface]
                diff_stats[iface] = {
                    "sent": (data.bytes_sent - prev.bytes_sent) / 1024.0,
                    "recv": (data.bytes_recv - prev.bytes_recv) / 1024.0,
                    "total_sent": data.bytes_sent / 1024 / 1024,
                    "total_recv": data.bytes_recv / 1024 / 1024,
                    "packets_sent": data.packets_sent,
                    "packets_recv": data.packets_recv,
                }

            self.prev_counters = new_counters
            self.stats = diff_stats or {"info": "No active traffic detected"}
        except Exception as e:
            self.stats = {"error": str(e)}

        self.refresh()

    def render(self) -> str:
        """描画"""
        lines = ["🌐 [b]Network Monitor[/b]\n"]
        if "error" in self.stats:
            lines.append(f"[red]Error:[/] {self.stats['error']}")
            return "\n".join(lines)
        if "info" in self.stats:
            lines.append(self.stats["info"])
            return "\n".join(lines)
        if not self.stats:
            lines.append("(no active interface)")
            return "\n".join(lines)

        for iface, s in self.stats.items():
            lines.append(f"[b]{iface}[/b]")
            lines.append(f"  ↑ Sent: {s['sent']:6.1f} KiB/s ({s['total_sent']:6.1f} MiB total)")
            lines.append(f"  ↓ Recv: {s['recv']:6.1f} KiB/s ({s['total_recv']:6.1f} MiB total)")
            lines.append(f"  Packets: ↑{s['packets_sent']} / ↓{s['packets_recv']}\n")
        return "\n".join(lines)
