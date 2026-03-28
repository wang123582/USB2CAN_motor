#!/usr/bin/env python3
"""
STM32F407VE 底盘定位系统 - 上位机串口测试工具

通信接口默认: /dev/ttyUSB0

协议:
1) 下发命令帧（8字节）
   - 普通命令: 0x0F + 6字节 + 0xAA
   - 复位命令: 0xBB + 6字节 + 0xCC

2) 上行数据（ASCII）
   - "bc X Y Yaw Roll\r\n"
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Optional

import serial

NORMAL_HEAD = 0x0F
NORMAL_TAIL = 0xAA
RESET_HEAD = 0xBB
RESET_TAIL = 0xCC
DATA_LEN = 6
FRAME_LEN = 8


@dataclass
class BcPose:
    x: float
    y: float
    yaw: float
    roll: float


def parse_6bytes(items: Iterable[str]) -> bytes:
    vals = []
    for s in items:
        token = s.strip()
        if not token:
            raise ValueError("存在空字节参数")

        try:
            # 优先支持十进制 / 0x十六进制
            v = int(token, 0)
        except ValueError:
            # 兼容无前缀十六进制，如 B / BB
            if all(c in "0123456789abcdefABCDEF" for c in token) and len(token) <= 2:
                v = int(token, 16)
            else:
                raise ValueError(
                    f"无法解析字节: '{s}'，请使用十进制(如 11) 或十六进制(如 0x0B/0B/B/BB)"
                )

        if not (0 <= v <= 0xFF):
            raise ValueError(f"字节超范围: {s} (应在 0~255)")
        vals.append(v)

    if len(vals) != DATA_LEN:
        raise ValueError(f"必须提供 {DATA_LEN} 个字节，当前 {len(vals)}")
    return bytes(vals)


def build_frame(head: int, payload6: bytes) -> bytes:
    if len(payload6) != DATA_LEN:
        raise ValueError(f"payload 必须是 {DATA_LEN} 字节")
    if head == NORMAL_HEAD:
        return bytes([NORMAL_HEAD]) + payload6 + bytes([NORMAL_TAIL])
    if head == RESET_HEAD:
        return bytes([RESET_HEAD]) + payload6 + bytes([RESET_TAIL])
    raise ValueError(f"未知帧头: 0x{head:02X}")


def _decode_i16_le(lo: int, hi: int) -> int:
    v = (lo & 0xFF) | ((hi & 0xFF) << 8)
    if v & 0x8000:
        v -= 0x10000
    return v


def decode_xy_from_payload(payload6: bytes) -> tuple[int, int]:
    """按固件 DATARELOAD 约定解析 payload 前4字节为 X/Y (int16 little-endian)。"""
    if len(payload6) != DATA_LEN:
        raise ValueError(f"payload 必须是 {DATA_LEN} 字节")
    x = _decode_i16_le(payload6[0], payload6[1])
    y = _decode_i16_le(payload6[2], payload6[3])
    return x, y


def parse_bc_line(line: str) -> Optional[BcPose]:
    # 期望: bc X Y yaw roll
    parts = line.strip().split()
    if len(parts) < 5 or parts[0].lower() != "bc":
        return None

    try:
        return BcPose(
            x=float(parts[1]),
            y=float(parts[2]),
            yaw=float(parts[3]),
            roll=float(parts[4]),
        )
    except ValueError:
        return None


class ChassisTester:
    def __init__(self, port: str, baud: int, timeout: float = 0.02):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self._line_buf = bytearray()

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send_normal(self, payload6: bytes) -> None:
        self.ser.write(build_frame(NORMAL_HEAD, payload6))

    def send_reset(self, payload6: bytes) -> None:
        self.ser.write(build_frame(RESET_HEAD, payload6))

    def read_once(self, raw: bool = False) -> Optional[BcPose]:
        waiting = self.ser.in_waiting
        if waiting <= 0:
            time.sleep(0.001)
            return None

        chunk = self.ser.read(waiting)
        
        self._line_buf.extend(chunk)

        while b"\n" in self._line_buf:
            line, _, rest = self._line_buf.partition(b"\n")
            self._line_buf = bytearray(rest)

            try:
                text = line.decode("utf-8", errors="ignore").strip()
            except Exception:
                continue

            if not text:
                continue

            pose = parse_bc_line(text)
            if pose is not None:
                return pose

            # 不是 bc 格式也打印，方便调试
            print(f"RX: {text}")

        return None


def main() -> int:
    parser = argparse.ArgumentParser(description="底盘定位串口测试工具")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="串口设备，默认 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    parser.add_argument("--raw", action="store_true", help="打印原始字节")

    parser.add_argument("--send", nargs=6, metavar="BYTE", help="发送普通命令 6 字节")
    parser.add_argument(
        "--reset",
        nargs="*",
        metavar="BYTE",
        help=(
            "发送复位命令；可不带参数(默认 00 00 00 00 00 00)，"
            f"或提供 {DATA_LEN} 字节"
        ),
    )
    parser.add_argument(
        "--reset-zero",
        action="store_true",
        help="发送复位并回零（等价于 --reset 00 00 00 00 00 00）",
    )
    parser.add_argument(
        "--reset-repeat",
        type=int,
        default=1,
        help="复位帧重复发送次数（默认 1）",
    )
    parser.add_argument(
        "--reset-interval",
        type=float,
        default=0.05,
        help="复位重复发送间隔秒数（默认 0.05）",
    )

    parser.add_argument("--listen", action="store_true", help="持续监听上行 bc 数据")
    parser.add_argument("--duration", type=float, default=0.0, help="监听时长(秒)，0 表示无限")
    args = parser.parse_args()

    try:
        tester = ChassisTester(port=args.port, baud=args.baud)
        print(f"✅ 串口已打开: {args.port} @ {args.baud}")
    except Exception as e:
        print(f"❌ 打开串口失败: {e}")
        return 1

    try:
        has_send = args.send is not None
        has_reset = (args.reset is not None) or args.reset_zero

        if args.reset_repeat < 1:
            raise ValueError("--reset-repeat 不能小于 1")
        if args.reset_interval < 0:
            raise ValueError("--reset-interval 不能小于 0")

        if has_send:
            payload = parse_6bytes(args.send)
            tester.send_normal(payload)
            print("TX normal:", " ".join(f"{b:02X}" for b in build_frame(NORMAL_HEAD, payload)))

        if has_reset:
            if args.reset_zero and args.reset is not None and len(args.reset) > 0:
                raise ValueError("--reset-zero 与 --reset 参数不能同时使用")

            if args.reset_zero:
                payload = bytes([0] * DATA_LEN)
            elif len(args.reset) == 0:
                payload = bytes([0] * DATA_LEN)
            elif len(args.reset) == DATA_LEN:
                payload = parse_6bytes(args.reset)
            else:
                raise ValueError(f"--reset 需要 0 或 {DATA_LEN} 个字节，当前 {len(args.reset)}")

            if payload != bytes([0] * DATA_LEN):
                x_cmd, y_cmd = decode_xy_from_payload(payload)
                print(
                    f"⚠️ 当前 reset 载荷不会回零，将设置 X={x_cmd}, Y={y_cmd}"
                )

            frame_hex = " ".join(f"{b:02X}" for b in build_frame(RESET_HEAD, payload))
            for i in range(args.reset_repeat):
                tester.send_reset(payload)
                print(f"TX reset [{i + 1}/{args.reset_repeat}]:", frame_hex)
                if i < args.reset_repeat - 1 and args.reset_interval > 0:
                    time.sleep(args.reset_interval)

        need_listen = args.listen or (not has_send and not has_reset)
        if need_listen:
            print("👂 监听中... 按 Ctrl+C 退出")
            start = time.time()
            while True:
                pose = tester.read_once(raw=args.raw)
                if pose is not None:
                    print(
                        f"BC -> X={pose.x:.4f}, Y={pose.y:.4f}, "
                        f"Yaw={pose.yaw:.3f}, Roll={pose.roll:.3f}"
                    )

                if args.duration > 0 and (time.time() - start) >= args.duration:
                    break

        return 0
    except ValueError as e:
        print(f"❌ 参数错误: {e}")
        return 2
    except KeyboardInterrupt:
        print("\n🛑 用户中断")
        return 0
    finally:
        tester.close()


if __name__ == "__main__":
    sys.exit(main())
