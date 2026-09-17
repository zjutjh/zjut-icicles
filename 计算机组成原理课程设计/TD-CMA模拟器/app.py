from __future__ import annotations

import os
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from tkinter.scrolledtext import ScrolledText

from tdcma_core import ParseError, TdcmaCpu, load_tdcma_file


class TdcmaApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("TD-CMA 3.3 带中断功能的复杂模型机模拟器")
        self.geometry("1120x760")
        self.minsize(920, 640)
        self.cpu = TdcmaCpu()
        self.running = False
        self.loaded_path: str | None = None
        self.status_vars: dict[str, tk.StringVar] = {}
        self._build_ui()
        self._refresh()
        self._log("请先打开 TD-CMA txt 文件，或加载 examples/interrupt_demo.txt。")

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        toolbar = ttk.Frame(root)
        toolbar.pack(fill=tk.X)
        ttk.Button(toolbar, text="打开文件", command=self.open_file).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(toolbar, text="连续运行", command=self.start_run).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(toolbar, text="终止/复位", command=self.reset_cpu).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(toolbar, text="单指令运行", command=self.step_instruction).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(toolbar, text="触发中断", command=self.trigger_interrupt).pack(side=tk.LEFT, padx=(0, 8))

        self.file_var = tk.StringVar(value="未加载文件")
        ttk.Label(toolbar, textvariable=self.file_var).pack(side=tk.LEFT, padx=12)

        body = ttk.PanedWindow(root, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        left = ttk.Frame(body, padding=(0, 0, 10, 0))
        right = ttk.Frame(body)
        body.add(left, weight=1)
        body.add(right, weight=3)

        reg_frame = ttk.LabelFrame(left, text="寄存器")
        reg_frame.pack(fill=tk.X)
        for idx, name in enumerate(["R0", "R1", "R2", "R3"]):
            self._add_status(reg_frame, name, idx // 2, (idx % 2) * 2)

        state_frame = ttk.LabelFrame(left, text="状态")
        state_frame.pack(fill=tk.X, pady=(10, 0))
        names = ["PC", "IR", "AR", "SP", "FZ", "FC", "EI", "INTR", "uAR", "OUT"]
        for idx, name in enumerate(names):
            self._add_status(state_frame, name, idx, 0)

        io_frame = ttk.LabelFrame(left, text="IN 单元")
        io_frame.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(io_frame, text="8 位二进制").grid(row=0, column=0, sticky=tk.W, padx=6, pady=6)
        self.in_var = tk.StringVar(value="00000000")
        ttk.Entry(io_frame, textvariable=self.in_var, width=12).grid(row=0, column=1, sticky=tk.EW, padx=6, pady=6)
        ttk.Button(io_frame, text="写入 IN", command=self.update_input).grid(row=1, column=0, columnspan=2, sticky=tk.EW, padx=6, pady=(0, 6))
        io_frame.columnconfigure(1, weight=1)

        notes = ttk.LabelFrame(left, text="当前假设")
        notes.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        text = (
            "M23 暂按 ALU 的 CN 输入处理。\n"
            "8259 实现 ICW1/2/3/4、OCW1、IR0~IR7、两次 INTA# 和自动 EOI。\n"
            "I/O 空间：00~3F 为 IN，40~7F 为 OUT，C0~FF 为 8259。"
        )
        ttk.Label(notes, text=text, wraplength=300, justify=tk.LEFT).pack(anchor=tk.NW, padx=8, pady=8)

        log_frame = ttk.LabelFrame(right, text="运行日志")
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.log = ScrolledText(log_frame, height=25, font=("Menlo", 12), wrap=tk.WORD)
        self.log.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

    def _add_status(self, parent: ttk.Frame, name: str, row: int, col: int) -> None:
        var = tk.StringVar(value="00")
        self.status_vars[name] = var
        ttk.Label(parent, text=name, width=6).grid(row=row, column=col, sticky=tk.W, padx=(6, 2), pady=4)
        ttk.Entry(parent, textvariable=var, width=8, justify=tk.CENTER, state="readonly").grid(
            row=row, column=col + 1, sticky=tk.EW, padx=(2, 6), pady=4
        )
        parent.columnconfigure(col + 1, weight=1)

    def open_file(self) -> None:
        initial = os.path.join(os.path.dirname(__file__), "examples")
        path = filedialog.askopenfilename(
            title="打开 TD-CMA txt 文件",
            initialdir=initial if os.path.isdir(initial) else os.getcwd(),
            filetypes=[("TD-CMA text", "*.txt"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            image = load_tdcma_file(path)
        except (OSError, ParseError, ValueError) as exc:
            messagebox.showerror("加载失败", str(exc))
            return
        self.stop_run()
        self.cpu.load_image(image)
        self.loaded_path = path
        self.file_var.set(os.path.basename(path))
        self._refresh()
        self._log(f"已加载：{path}")
        for warning in image.warnings:
            self._log(f"WARN: {warning}")

    def update_input(self) -> None:
        try:
            self.cpu.set_input_binary(self.in_var.get())
        except ValueError as exc:
            messagebox.showwarning("IN 输入错误", str(exc))
            return
        self._refresh()
        self._log(f"IN <- {self.cpu.in_unit:02X} ({self.in_var.get().strip()})")

    def trigger_interrupt(self) -> None:
        self.update_input()
        self.cpu.trigger_irq0()
        self._refresh()
        self._log(f"IR0 中断请求置位，IN={self.cpu.in_unit:02X}")

    def step_instruction(self) -> None:
        self.stop_run()
        self._run_one_instruction()

    def start_run(self) -> None:
        if self.running:
            return
        self.running = True
        self._log("连续运行开始")
        self.after(1, self._run_tick)

    def stop_run(self) -> None:
        self.running = False

    def reset_cpu(self) -> None:
        self.stop_run()
        self.cpu.reset()
        self._refresh()
        self._log("已复位：主存/微控存恢复到已加载文件，寄存器清零。")

    def _run_tick(self) -> None:
        if not self.running:
            return
        for _ in range(5):
            if self.cpu.halted:
                self.running = False
                self._log("CPU 已停机。")
                break
            self._run_one_instruction(log_limit=8)
        if self.running:
            self.after(60, self._run_tick)

    def _run_one_instruction(self, log_limit: int | None = None) -> None:
        if self.cpu.halted:
            self._log("CPU 已停机。")
            return
        events = self.cpu.step_instruction()
        if not events:
            self._log("CPU 已停机。")
            return
        shown = events if log_limit is None else events[:log_limit]
        for event in shown:
            self._log(event.format())
        if log_limit is not None and len(events) > log_limit:
            self._log(f"... 本条机器指令还有 {len(events) - log_limit} 条微指令日志已折叠")
        self._refresh()

    def _refresh(self) -> None:
        state = self.cpu.state()
        for name, var in self.status_vars.items():
            value = state.get(name, 0)
            if name in {"FZ", "FC", "EI", "INTR"}:
                var.set(str(value))
            else:
                var.set(f"{value:02X}")
        self.in_var.set(f"{self.cpu.in_unit:08b}")

    def _log(self, line: str) -> None:
        self.log.insert(tk.END, line + "\n")
        self.log.see(tk.END)


if __name__ == "__main__":
    app = TdcmaApp()
    app.mainloop()

