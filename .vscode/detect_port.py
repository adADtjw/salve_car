"""
RT1021 MicroPython 开发板自动检测脚本。
通过 USB VID:PID (F055:9802) 识别逐飞 RT1021 开发板。

用法：
    python detect_port.py      # 自动检测并选择
    python detect_port.py list # 仅列出所有检测到的板子
"""

import serial.tools.list_ports
import sys

TARGET_VID = 0xF055
TARGET_PID = 0x9802


def find_boards():
    boards = []
    for p in serial.tools.list_ports.comports():
        if p.vid == TARGET_VID and p.pid == TARGET_PID:
            boards.append(p.device)
    return boards


def main():
    boards = find_boards()

    if len(boards) == 0:
        print("错误: 未检测到 RT1021 MicroPython 开发板 (VID:PID=F055:9802)", file=sys.stderr)
        print("请检查 USB 连接", file=sys.stderr)
        sys.exit(1)

    if len(sys.argv) > 1 and sys.argv[1] == "list":
        for i, p in enumerate(boards):
            print(f"  [{i+1}] {p}")
        return

    if len(boards) == 1:
        print(boards[0])
        return

    # 多个板子，让用户选择
    print(f"\n检测到 {len(boards)} 个 RT1021 开发板:\n", file=sys.stderr)
    for i, p in enumerate(boards):
        print(f"  [{i+1}] {p}", file=sys.stderr)

    print("", file=sys.stderr)
    try:
        choice = input(f"请选择 (1-{len(boards)}): ").strip()
        idx = int(choice) - 1
        if 0 <= idx < len(boards):
            print(boards[idx])
        else:
            print(f"错误: 无效选择 '{choice}'", file=sys.stderr)
            sys.exit(1)
    except (ValueError, EOFError):
        print("错误: 无效输入", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
