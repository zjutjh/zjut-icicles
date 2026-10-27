#!/bin/zsh
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "正在启动 TD-CMA 3.3 复杂模型机模拟器..."
echo "浏览器会自动打开；关闭此终端窗口即可停止服务。"
echo

exec /usr/bin/python3 web_app.py --auto-port --open

