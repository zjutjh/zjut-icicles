from __future__ import annotations

import argparse
import errno
import json
import mimetypes
import os
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
from typing import Any, Dict, List, Optional
import urllib.error
import urllib.request
import webbrowser

from tdcma_core import MemoryImage, ParseError, TdcmaCpu, load_tdcma_file, parse_tdcma_text


ROOT = Path(__file__).resolve().parent
STATIC_DIR = ROOT / "web_static"
EXAMPLE_PATH = ROOT / "examples" / "interrupt_demo.txt"
SIMULATOR_VERSION = "2026-06-01-ri-indexed-addressing"


class SimulatorService:
    def __init__(self) -> None:
        self.cpu = TdcmaCpu()
        self.loaded_name = "未加载文件"
        self.lock = threading.RLock()
        if EXAMPLE_PATH.exists():
            image = load_tdcma_file(str(EXAMPLE_PATH))
            self.cpu.load_image(image)
            self.loaded_name = EXAMPLE_PATH.name

    def load_text(self, text: str, filename: str) -> Dict[str, Any]:
        with self.lock:
            image = parse_tdcma_text(text)
            self.cpu.load_image(image)
            self.loaded_name = filename or "浏览器导入文件"
            logs = [f"已加载：{self.loaded_name}"]
            logs.extend(f"WARN: {warning}" for warning in image.warnings)
            return self.snapshot(logs)

    def load_example(self) -> Dict[str, Any]:
        with self.lock:
            image = load_tdcma_file(str(EXAMPLE_PATH))
            self.cpu.load_image(image)
            self.loaded_name = EXAMPLE_PATH.name
            return self.snapshot([f"已加载示例：{EXAMPLE_PATH.name}"])

    def reset(self) -> Dict[str, Any]:
        with self.lock:
            self.cpu.reset()
            return self.snapshot(["已复位"])

    def set_input(self, bits: str) -> Dict[str, Any]:
        with self.lock:
            self.cpu.set_input_binary(bits)
            return self.snapshot([f"IN <- {self.cpu.in_unit:02X} ({self.cpu.in_unit:08b})"])

    def trigger_irq0(self) -> Dict[str, Any]:
        with self.lock:
            self.cpu.trigger_irq0()
            return self.snapshot([f"IR0 中断请求置位，IN={self.cpu.in_unit:02X}"])

    def step(self, count: int = 1, log_limit: Optional[int] = None) -> Dict[str, Any]:
        with self.lock:
            logs: List[str] = []
            count = max(1, min(count, 100))
            for _ in range(count):
                if self.cpu.halted:
                    logs.append("CPU 已停机")
                    break
                events = self.cpu.step_instruction()
                if log_limit is None:
                    logs.extend(event.format() for event in events)
                else:
                    logs.extend(event.format() for event in events[:log_limit])
                    if len(events) > log_limit:
                        logs.append(f"... 本条机器指令还有 {len(events) - log_limit} 条微指令日志已折叠")
                if self.cpu.halted:
                    logs.append("CPU 已停机")
                    break
            return self.snapshot(logs)

    def snapshot(self, logs: Optional[List[str]] = None) -> Dict[str, Any]:
        state = self.cpu.state()
        return {
            "state": state,
            "halted": self.cpu.halted,
            "loadedName": self.loaded_name,
            "simulatorVersion": SIMULATOR_VERSION,
            "microSteps": self.cpu.micro_steps,
            "instructions": self.cpu.completed_instructions,
            "inBits": f"{self.cpu.in_unit:08b}",
            "mainMemory": self.cpu.mem,
            "microMemory": self.cpu.micro,
            "logs": logs or [],
        }


SERVICE = SimulatorService()


class WebHandler(BaseHTTPRequestHandler):
    server_version = "TDCMAWeb/1.0"

    def do_GET(self) -> None:
        if self.path == "/api/state":
            self._send_json(SERVICE.snapshot())
            return
        if self.path == "/":
            self._serve_file(STATIC_DIR / "index.html")
            return
        if self.path.startswith("/static/"):
            safe_name = self.path.removeprefix("/static/").split("?", 1)[0]
            safe_path = STATIC_DIR / safe_name
            if not _is_within(safe_path, STATIC_DIR):
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            self._serve_file(safe_path)
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        try:
            body = self._read_json()
            if self.path == "/api/load":
                text = str(body.get("text", ""))
                filename = str(body.get("filename", ""))
                self._send_json(SERVICE.load_text(text, filename))
                return
            if self.path == "/api/load-example":
                self._send_json(SERVICE.load_example())
                return
            if self.path == "/api/reset":
                self._send_json(SERVICE.reset())
                return
            if self.path == "/api/input":
                self._send_json(SERVICE.set_input(str(body.get("bits", ""))))
                return
            if self.path == "/api/interrupt":
                self._send_json(SERVICE.trigger_irq0())
                return
            if self.path == "/api/step":
                count = int(body.get("count", 1))
                raw_limit = body.get("logLimit")
                log_limit = None if raw_limit is None else int(raw_limit)
                self._send_json(SERVICE.step(count=count, log_limit=log_limit))
                return
        except BrokenPipeError:
            return
        except (ParseError, ValueError, OSError, json.JSONDecodeError) as exc:
            try:
                self._send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
            except BrokenPipeError:
                return
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def log_message(self, fmt: str, *args: object) -> None:
        print(f"{self.address_string()} - {fmt % args}")

    def _read_json(self) -> Dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length == 0:
            return {}
        data = self.rfile.read(length)
        return json.loads(data.decode("utf-8"))

    def _send_json(self, payload: Dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _serve_file(self, path: Path) -> None:
        if not path.exists() or not path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        content = path.read_bytes()
        content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        if path.suffix == ".js":
            content_type = "text/javascript; charset=utf-8"
        elif path.suffix in {".html", ".css"}:
            content_type = f"text/{path.suffix[1:]}; charset=utf-8"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(content)


def _is_within(path: Path, parent: Path) -> bool:
    try:
        path.resolve().relative_to(parent.resolve())
        return True
    except ValueError:
        return False


class TdcmaHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


def main() -> None:
    parser = argparse.ArgumentParser(description="TD-CMA 3.3 web simulator")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--auto-port", action="store_true", help="try the next port when the requested port is busy")
    parser.add_argument("--open", action="store_true", help="open the simulator page in the default browser")
    args = parser.parse_args()
    os.chdir(ROOT)
    server = None
    port = args.port
    max_port = args.port + 30 if args.auto_port else args.port
    existing_url = _existing_simulator_url(args.host, args.port)
    if existing_url and args.open:
        print(f"TD-CMA Web simulator is already running: {existing_url}")
        webbrowser.open(existing_url)
        return

    try:
        while port <= max_port:
            try:
                server = TdcmaHTTPServer((args.host, port), WebHandler)
                break
            except OSError as exc:
                if exc.errno == errno.EADDRINUSE and args.auto_port:
                    port += 1
                    continue
                raise
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE:
            print(f"端口 {args.port} 已被占用。")
            print(f"如果模拟器已经打开，请直接访问 http://{args.host}:{args.port}")
            print(f"或者换一个端口启动，例如：/usr/bin/python3 web_app.py --port {args.port + 1}")
            raise SystemExit(1) from exc
        raise
    if server is None:
        print(f"端口 {args.port} 到 {max_port} 都已被占用。")
        raise SystemExit(1)
    host, port = server.server_address
    url = f"http://{host}:{port}"
    print(f"TD-CMA Web simulator: {url}")
    if args.open:
        threading.Timer(0.4, webbrowser.open, args=(url,)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")


def _existing_simulator_url(host: str, port: int) -> Optional[str]:
    url = f"http://{host}:{port}/api/state"
    try:
        with urllib.request.urlopen(url, timeout=0.25) as response:
            if response.status != 200:
                return None
            data = json.loads(response.read().decode("utf-8"))
    except (OSError, urllib.error.URLError, json.JSONDecodeError):
        return None
    if isinstance(data, dict) and data.get("simulatorVersion") == SIMULATOR_VERSION:
        return f"http://{host}:{port}"
    return None


if __name__ == "__main__":
    main()
