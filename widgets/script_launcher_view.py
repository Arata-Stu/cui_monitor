#!/usr/bin/env python3
import asyncio
import os
import signal
from pathlib import Path
from textual.widget import Widget
from textual.containers import VerticalScroll, Horizontal, Grid
from textual.widgets import Button, Static, Label, Log, Input
from textual.reactive import reactive
from textual import on

WIDGET_META = {
    "id": "launcher",
    "title": "Script Launcher View",
    "class_name": "launcher",
    "category": "system",
    "description": "複数スクリプトを並列実行・中断・監視します（対話入力＋安全停止対応）。",
    "order": 50,
}

# =========================================================
# スクリプト管理クラス
# =========================================================
class ScriptTask:
    """各スクリプトの状態を保持"""
    def __init__(self, path: Path):
        self.path = path
        self.process: asyncio.subprocess.Process | None = None
        self.status: str = "IDLE"


# =========================================================
# ScriptLauncherView 本体
# =========================================================
class ScriptLauncherView(Widget):
    """複数スクリプトを非同期で管理できるランチャー"""
    tasks: dict[str, ScriptTask] = reactive({})
    current_task_name: str | None = None
    _update_lock: asyncio.Lock

    # =========================================================
    # 初期構築
    # =========================================================
    def compose(self):
        yield Horizontal(Label("🚀 Script Launcher View", classes="title"), id="toolbar")
        yield VerticalScroll(id="script-list")
        yield Log("🕹️ Script Launcher でスクリプトを実行してください。", id="log-box", auto_scroll=True)
        with Horizontal(id="input-bar"):
            yield Input(placeholder="💬 対話入力（Enterで送信）", id="input-box")
            yield Button("🧹 Clear Log", id="clear-log", variant="primary")

    async def on_mount(self):
        self._update_lock = asyncio.Lock()
        for sel, h in [("#script-list", 10), ("#input-bar", 3), ("#toolbar", 3)]:
            self.query_one(sel).styles.height = h
        self.query_one("#log-box", Log).styles.height = "1fr"

        self.scripts_dir = Path(__file__).resolve().parent.parent / "scripts"
        if not self.scripts_dir.exists():
            self.safe_notify(f"⚠️ スクリプトフォルダが存在しません: {self.scripts_dir}")
            return

        scripts = sorted(self.scripts_dir.glob("*.sh"))
        if not scripts:
            self.safe_notify("⚠️ scripts フォルダに .sh ファイルがありません")
            return

        self.tasks = {s.name: ScriptTask(s) for s in scripts}
        await self.safe_update_buttons()

    # =========================================================
    # UI更新
    # =========================================================
    async def safe_update_buttons(self, retries=3):
        """UI更新を安全に再試行つきで実行"""
        for attempt in range(retries):
            try:
                await self.update_script_buttons()
                return
            except Exception as e:
                self.safe_log(f"[WARN] update_script_buttons失敗({attempt+1}/{retries}): {type(e).__name__}: {e}")
                await asyncio.sleep(0.1)

    async def update_script_buttons(self):
        """スクリプトリストをグリッド形式でUIに反映"""
        async with self._update_lock:
            scroll = self.query_one("#script-list", VerticalScroll)
            await scroll.remove_children()

            grid = Grid(classes="script-grid")
            await scroll.mount(grid)
            grid.styles.grid_size_columns = 2
            grid.styles.grid_gap = 1

            for name, task in self.tasks.items():
                safe_name = name.replace(".", "_")
                run_button = Button(f"▶ {name}", id=f"run-{safe_name}", variant="success", classes="script-run-button")
                stop_button = (
                    Button("🛑 Stop", id=f"stop-{safe_name}", variant="error", classes="script-stop-button")
                    if task.status == "RUNNING" else None
                )
                status_label = Static(f"{task.status}", classes=f"script-status {task.status.lower()}")

                children = [run_button]
                if stop_button:
                    children.append(stop_button)
                children.append(status_label)

                try:
                    await grid.mount(Widget(*children, classes="script-card", id=f"card-{safe_name}"))
                except Exception as e:
                    self.safe_log(f"[WARN] UI構築中例外: {type(e).__name__}: {e}")

    # =========================================================
    # ボタン押下処理
    # =========================================================
    async def on_button_pressed(self, event: Button.Pressed):
        bid = event.button.id
        if bid == "clear-log":
            self.safe_clear_log(); return

        name = None
        if bid.startswith("run-"):
            name = bid[4:].replace("_", ".")
            asyncio.create_task(self.start_script(name))
        elif bid.startswith("stop-"):
            name = bid[5:].replace("_", ".")
            asyncio.create_task(self.stop_script(name))

    # =========================================================
    # スクリプト実行処理
    # =========================================================
    async def start_script(self, name: str):
        task = self.tasks.get(name)
        if not task or task.status == "RUNNING":
            return

        self.current_task_name = name
        task.status = "RUNNING"
        await self.safe_update_buttons()
        self.safe_log(f"[INFO] ▶ Start: {name}")

        try:
            task.process = await asyncio.create_subprocess_exec(
                "bash", str(task.path),
                stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT, preexec_fn=os.setpgrp,
            )
            asyncio.create_task(self._read_output(task, name))
            asyncio.create_task(self._wait_for_exit(task, name))
        except Exception as e:
            task.status = "ERROR"
            self.safe_log(f"[ERROR] {name} 起動失敗: {e}")
            await self.safe_update_buttons()

    async def _read_output(self, task: ScriptTask, name: str):
        try:
            while task.process:
                try:
                    line = await asyncio.wait_for(task.process.stdout.readline(), timeout=1.0)
                except asyncio.TimeoutError:
                    continue
                if not line:
                    break
                decoded = line.decode(errors="ignore").rstrip()
                self.safe_log(f"{name}: {decoded}")
                if "password" in decoded.lower() or "yes/no" in decoded.lower():
                    self.safe_log("💬 入力を求めています。下のテキストボックスに入力してください。")
        except Exception as e:
            self.safe_log(f"[WARN] 出力読み取り中例外: {e}")

    async def _wait_for_exit(self, task: ScriptTask, name: str):
        try:
            rc = await task.process.wait()
        except Exception:
            rc = -1
        task.status = "DONE" if rc == 0 else "ERROR"
        self.safe_log(f"[INFO] {name} 終了: code={rc}")
        task.process = None
        await self.safe_update_buttons()

    # =========================================================
    # 標準入力転送
    # =========================================================
    @on(Input.Submitted, "#input-box")
    async def on_user_input(self, event: Input.Submitted):
        user_input = event.value.strip()
        input_box = self.query_one("#input-box", Input)
        input_box.value = ""

        if not self.current_task_name:
            self.safe_log("[WARN] 実行中スクリプトがありません。"); return

        task = self.tasks.get(self.current_task_name)
        if not task or not task.process or not task.process.stdin:
            self.safe_log(f"[WARN] {self.current_task_name} は入力を受け付けられません。"); return

        try:
            task.process.stdin.write(user_input.encode() + b"\n")
            await task.process.stdin.drain()
            self.safe_log(f"[INFO] ➡ 入力送信: {user_input}")
        except Exception as e:
            self.safe_log(f"[ERROR] 入力送信失敗: {e}")

    # =========================================================
    # 停止処理（安全kill）
    # =========================================================
    async def stop_script(self, name: str):
        task = self.tasks.get(name)
        if not task or not task.process:
            return
        self.safe_log(f"[INFO] 🛑 Stop requested: {name}")

        try:
            proc = task.process
            pid = getattr(proc, "pid", None)
            if not pid:
                task.status = "STOPPED"
                await self.safe_update_buttons()
                return

            # stdinを閉じる
            try:
                if proc.stdin:
                    proc.stdin.close()
            except Exception as e:
                self.safe_log(f"[WARN] stdin close失敗: {e}")

            # プロセスグループ取得
            try:
                pgid = os.getpgid(pid)
            except ProcessLookupError:
                pgid = pid

            # SIGINT→TERM→KILL
            for sig in [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]:
                try:
                    os.killpg(pgid, sig)
                except ProcessLookupError:
                    break
                await asyncio.sleep(0.3)
                if proc.returncode is not None:
                    break

            # wait回収
            try:
                await asyncio.wait_for(proc.wait(), timeout=1.5)
            except Exception:
                pass

            task.status = "STOPPED"
            self.safe_log(f"[INFO] {name} 停止完了")

        except Exception as e:
            self.safe_log(f"[ERROR] {name} 停止中に例外: {type(e).__name__}: {e}")

        finally:
            task.process = None
            await self.safe_update_buttons()

    # =========================================================
    # 🔹 全停止（reloadや終了時に使用）
    # =========================================================
    async def stop_all_scripts(self):
        running = [n for n, t in self.tasks.items() if t.process and t.status == "RUNNING"]
        if not running:
            return
        self.safe_log(f"[INFO] 🔻 全スクリプト停止開始: {running}")
        for name in running:
            await self.stop_script(name)
        self.safe_log("[INFO] ✅ 全スクリプト停止完了")

    # =========================================================
    # 終了時クリーンアップ
    # =========================================================
    async def on_unmount(self):
        await self.stop_all_scripts()
        print("💤 ScriptLauncherView: 全プロセス停止完了")

    # =========================================================
    # 安全ロガー群
    # =========================================================
    def safe_log(self, text: str):
        try:
            log_box = self.query_one("#log-box", Log)
            log_box.write_line(text)
        except Exception:
            print(f"[LOG_FALLBACK] {text}")

    def safe_clear_log(self):
        try:
            log_box = self.query_one("#log-box", Log)
            log_box.clear(); log_box.write_line("🧹 Log cleared.")
        except Exception:
            print("[WARN] ログクリア失敗")

    def safe_notify(self, text: str):
        try:
            self.notify(text, severity="warning")
        except Exception:
            print(f"[NOTIFY_FALLBACK] {text}")
