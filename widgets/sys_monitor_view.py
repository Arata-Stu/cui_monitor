import psutil, asyncio, subprocess, os, re
from textual.widget import Widget
from textual.reactive import reactive

WIDGET_META = {
    "id": "sys",
    "title": "System Monitor",
    "class_name": "sys",  
    "category": "system",
    "description": "CPU・メモリ・GPU・温度をリアルタイム監視します（警告色対応）。",
    "order": 10,
}

class SysMonitorView(Widget):
    """CPU・メモリ・スワップ・GPU・温度をリアルタイム監視"""

    cpu = reactive(0.0)
    per_cpu = reactive([])
    mem = reactive(0.0)
    swap = reactive(0.0)
    gpu = reactive("N/A")
    temps = reactive({})
    is_jetson = os.path.exists("/usr/bin/tegrastats")

    async def on_mount(self):
        self.set_interval(1.0, self.update_metrics)

    async def update_metrics(self):
        try:
            self.cpu = psutil.cpu_percent()
            self.per_cpu = psutil.cpu_percent(percpu=True)
            self.mem = psutil.virtual_memory().percent
            self.swap = psutil.swap_memory().percent

            if self.is_jetson:
                self.gpu = self.get_jetson_gpu_usage()
                self.temps = self.get_jetson_temps()
            else:
                self.gpu = self.get_gpu_usage_generic()
                self.temps = self.get_generic_temps()

        except Exception as e:
            self.gpu = f"Error: {e}"

        self.refresh()

    def get_jetson_gpu_usage(self):
        try:
            out = subprocess.check_output(
                ["tegrastats", "--interval", "1000", "--count", "1"],
                text=True, timeout=1
            )
            match = re.search(r"GR3D_FREQ\s+(\d+)%", out)
            if match:
                return match.group(1)
        except Exception:
            pass
        return "N/A"

    def get_gpu_usage_generic(self):
        if os.path.exists("/usr/bin/nvidia-smi"):
            try:
                out = subprocess.check_output(
                    ["nvidia-smi", "--query-gpu=utilization.gpu", "--format=csv,noheader,nounits"],
                    text=True
                )
                return out.strip()
            except Exception:
                return "N/A"
        return "N/A"

    def get_jetson_temps(self):
        temps = {}
        for name, path in [("CPU", "/sys/devices/virtual/thermal/thermal_zone0/temp"),
                           ("GPU", "/sys/devices/virtual/thermal/thermal_zone1/temp")]:
            try:
                with open(path) as f:
                    temps[name] = float(f.read().strip()) / 1000.0
            except Exception:
                pass
        return temps

    def get_generic_temps(self):
        temps = {}
        if hasattr(psutil, "sensors_temperatures"):
            t = psutil.sensors_temperatures(fahrenheit=False)
            for k, sensors in t.items():
                if sensors:
                    temps[k] = sensors[0].current
        return temps

    # ===== 色付けヘルパ =====
    def colorize(self, value: float, warn: float, critical: float) -> str:
        """数値を閾値で色付けして文字列に変換"""
        if value == "N/A" or value is None:
            return "[dim]N/A[/dim]"
        try:
            v = float(value)
        except ValueError:
            return str(value)
        if v >= critical:
            color = "bold red"
        elif v >= warn:
            color = "yellow"
        else:
            color = "green"
        return f"[{color}]{v:5.1f}%[/]"

    def render(self) -> str:
        lines = []
        lines.append("🧠 [b]System Monitor[/b]\n")

        # CPU
        lines.append(f"CPU Total: {self.colorize(self.cpu, 70, 90)}")
        per_core = " | ".join([self.colorize(v, 70, 90) for v in self.per_cpu])
        lines.append(f"Cores: {per_core}\n")

        # Memory / Swap
        lines.append(f"Memory: {self.colorize(self.mem, 75, 90)}")
        lines.append(f"Swap:   {self.colorize(self.swap, 60, 80)}")

        # GPU
        if self.gpu != "N/A":
            lines.append(f"\nGPU: {self.colorize(self.gpu, 70, 90)}")
        else:
            lines.append(f"\nGPU: [dim]{self.gpu}[/dim]")

        # 温度
        if self.temps:
            temp_str = " | ".join([
                f"{k}:{self.colorize(v, 70, 85).replace('%', '°C')}" for k, v in self.temps.items()
            ])
            lines.append(f"Temp: {temp_str}")

        return "\n".join(lines)
