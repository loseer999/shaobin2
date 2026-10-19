#!/usr/bin/env python3
"""
串口 TX 调试工具 — 按现有协议发送自定义控制帧
=================================================
TX 帧格式 (NUC → MCU, 11 字节):
  [0]   head      0xAA
  [1]   type      0x01
  [2]   arrive    0=导航中 / 1=已到达
  [3-4] vx        float→2字节压缩 (高:符号+整数6bit, 低:小数×100)
  [5-6] vy        同上
  [7-8] wz        同上 (°/s)
  [9]   stance    1:进攻 2:防御 3:移动 4:强化进攻 5:强化防御 6:强化移动
  [10]  tail      0xDD

用法:
  python3 serial_debug.py /dev/ttyUSB1              # 交互模式
  python3 serial_debug.py /dev/ttyUSB1 --once 3 1.0 0 0 4  # 单次发送
"""

import sys
import struct
import serial
import argparse
import os


# ========== 协议常量 ==========
FRAME_HEAD = 0xAA
FRAME_TYPE = 0x01
FRAME_TAIL = 0xDD
FRAME_LEN  = 11

STANCE_MAP = {
    1: "进攻",
    2: "防御",
    3: "移动",
    4: "强化进攻",
    5: "强化防御",
    6: "强化移动",
}


# ========== 浮点压缩 (复用现有算法) ==========
def compress_float(val: float) -> tuple:
    """float → (high_byte, low_byte)"""
    integer_part = int(abs(val))
    fraction_part = abs(val) - integer_part

    int_val = min(integer_part, 63)
    if val < 0:
        high = 0x40 | int_val
    else:
        high = int_val

    low = int(round(fraction_part * 100.0))
    return (high, low)


def decompress_float(high: int, low: int) -> float:
    """(high_byte, low_byte) → float"""
    sign = -1.0 if (high & 0x40) else 1.0
    int_val = high & 0x3F
    frac_val = low / 100.0
    return sign * (int_val + frac_val)


# ========== 帧操作 ==========
class TxFrame:
    def __init__(self):
        self.arrive = 0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.stance = 3  # 默认移动

    def pack(self) -> bytes:
        buf = bytearray(FRAME_LEN)
        buf[0] = FRAME_HEAD
        buf[1] = FRAME_TYPE
        buf[2] = self.arrive
        buf[3], buf[4] = compress_float(self.vx)
        buf[5], buf[6] = compress_float(self.vy)
        buf[7], buf[8] = compress_float(self.wz)
        buf[9] = self.stance
        buf[10] = FRAME_TAIL
        return bytes(buf)

    def hex(self) -> str:
        return " ".join(f"{b:02X}" for b in self.pack())

    def summary(self) -> str:
        return (
            f"arrive={self.arrive}  "
            f"vx={self.vx:+.2f} vy={self.vy:+.2f} wz={self.wz:+.2f}  "
            f"stance={self.stance}({STANCE_MAP.get(self.stance, '?')})"
        )


# ========== 串口操作 ==========
def open_port(port: str, baud: int = 500000):
    ser = serial.Serial(port, baud, timeout=1)
    if not ser.is_open:
        raise RuntimeError(f"无法打开串口: {port}")
    return ser


def send_frame(ser, frame: TxFrame):
    data = frame.pack()
    ser.write(data)
    print(f"  → 已发送: {frame.hex()}")
    print(f"  → {frame.summary()}")


# ========== 交互模式 ==========
def interactive_mode(ser, port: str):
    frame = TxFrame()
    os.system("clear" if os.name != "nt" else "cls")

    while True:
        print(f"\n{'='*55}")
        print(f"  串口 TX 调试 — {port} (500000 baud)")
        print(f"{'='*55}")
        print(f"  当前帧: {frame.summary()}")
        print(f"          {frame.hex()}")
        print(f"{'='*55}")
        print("  [1] arrive  = 0/1          (导航中/已到达)")
        print("  [2] vx      =  float       (m/s, ±63.99)")
        print("  [3] vy      =  float       (m/s, ±63.99)")
        print("  [4] wz      =  float       (°/s, ±63.99)")
        print("  [5] stance  =  1-6         (1进攻2防御3移动4-6强化)")
        print("  [s] 发送当前帧")
        print("  [r] 连续发送 (Ctrl+C 停止)")
        print("  [z] 全部归零")
        print("  [q] 退出")
        print(f"{'='*55}")

        try:
            cmd = input("  > ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\n退出")
            break

        if cmd == "q":
            break
        elif cmd == "s":
            send_frame(ser, frame)
        elif cmd == "r":
            print("  连续发送中 (10Hz), Ctrl+C 停止...")
            try:
                import time
                while True:
                    send_frame(ser, frame)
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\n  已停止")
        elif cmd == "z":
            frame.arrive = 0
            frame.vx = 0.0
            frame.vy = 0.0
            frame.wz = 0.0
            frame.stance = 3
            print("  ✓ 已归零")
        elif cmd == "1":
            try:
                frame.arrive = int(input("  arrive (0/1): "))
            except ValueError:
                print("  ✗ 无效")
        elif cmd == "2":
            try:
                frame.vx = float(input("  vx (m/s): "))
            except ValueError:
                print("  ✗ 无效")
        elif cmd == "3":
            try:
                frame.vy = float(input("  vy (m/s): "))
            except ValueError:
                print("  ✗ 无效")
        elif cmd == "4":
            try:
                frame.wz = float(input("  wz (°/s): "))
            except ValueError:
                print("  ✗ 无效")
        elif cmd == "5":
            try:
                val = int(input("  stance (1-6): "))
                if 1 <= val <= 6:
                    frame.stance = val
                else:
                    print("  ✗ 范围 1-6")
            except ValueError:
                print("  ✗ 无效")
        else:
            print("  ✗ 未知命令")


# ========== 一次性发送 ==========
def once_mode(ser, args):
    frame = TxFrame()
    frame.stance = args.stance
    frame.vx = args.vx
    frame.vy = args.vy
    frame.wz = args.wz
    frame.arrive = args.arrive
    send_frame(ser, frame)


# ========== 入口 ==========
def main():
    parser = argparse.ArgumentParser(description="串口 TX 调试工具")
    parser.add_argument("port", nargs="?", default="/dev/ttyUSB1", help="串口设备")
    parser.add_argument("--baud", type=int, default=500000, help="波特率 (默认 500000)")
    parser.add_argument("--once", action="store_true", help="单次发送模式")

    # 单次发送参数
    parser.add_argument("stance", nargs="?", type=int, default=3)
    parser.add_argument("vx", nargs="?", type=float, default=0.0)
    parser.add_argument("vy", nargs="?", type=float, default=0.0)
    parser.add_argument("wz", nargs="?", type=float, default=0.0)
    parser.add_argument("--arrive", type=int, default=0, help="到达标志 (0/1)")

    args = parser.parse_args()

    ser = open_port(args.port, args.baud)
    print(f"串口 {args.port} 已打开 ({args.baud} baud)")

    try:
        if args.once:
            once_mode(ser, args)
        else:
            interactive_mode(ser, args.port)
    finally:
        ser.close()
        print("串口已关闭")


if __name__ == "__main__":
    main()
