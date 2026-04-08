#!/usr/bin/env python3
"""
20260328-cc-pen_keyboard_client.py
===================================
力反馈笔键盘模拟测试客户端

通过键盘控制虚拟笔的位置，发送 UDP 数据到 MuJoCo 仿真器。
用于在没有力反馈笔硬件时测试仿真环境。

发送格式 (JSON over UDP):
  {"pos": [x, y, z], "vel": [0,0,0], "buttons": [b1, b2]}

接收格式 (JSON over UDP):
  {"force": [fx, fy, fz]}

操作说明:
  W/S  - 前进/后退 (笔 Z 轴，对应机器人 -X 轴)
  A/D  - 左移/右移 (笔 X 轴，对应机器人 Y 轴)
  Q/E  - 上升/下降 (笔 Y 轴，对应机器人 Z 轴)
  1    - 闭合夹爪 (模拟灰色按钮 Button1)
  2    - 张开夹爪 (模拟白色按钮 Button2)
  +/-  - 增大/减小步长
  R    - 重置位置到原点
  Ctrl+C - 退出
"""

import sys
import os
import json
import socket
import time
import tty
import termios
import select
import signal
import argparse


def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description="力反馈笔键盘模拟测试客户端"
    )
    parser.add_argument("--send-port", type=int, default=12345,
                        help="发送目标端口 (仿真器接收端口), 默认 12345")
    parser.add_argument("--recv-port", type=int, default=12346,
                        help="本地接收端口 (接收力反馈), 默认 12346")
    parser.add_argument("--send-ip", default="127.0.0.1",
                        help="发送目标 IP, 默认 127.0.0.1")
    parser.add_argument("--step", type=float, default=5.0,
                        help="每次按键移动的距离 (mm), 默认 5.0")
    return parser.parse_args()


class KeyboardPenClient:
    """
    键盘控制的虚拟力反馈笔

    模拟 Touch 设备的输出：
    - 位置 (mm)：通过 WASDQE 键控制
    - 按钮：通过 1/2 键控制
    """

    def __init__(self, args):
        """初始化 UDP 通信和键盘设置"""
        # ---- UDP 发送 ----
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.send_addr = (args.send_ip, args.send_port)

        # ---- UDP 接收（非阻塞） ----
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.bind(("0.0.0.0", args.recv_port))
        self.recv_sock.setblocking(False)

        # ---- 虚拟笔状态 ----
        self.pos = [0.0, 0.0, 0.0]    # 虚拟笔位置 (mm)
        self.buttons = [0, 0]          # [button1, button2]
        self.step_size = args.step     # 每次移动步长 (mm)
        self.last_force = [0.0, 0.0, 0.0]  # 最近收到的力反馈

        # ---- 终端原始模式设置 ----
        self.old_settings = None
        self.running = True

        # 信号处理
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, sig, frame):
        """优雅退出"""
        self.running = False

    def _enable_raw_mode(self):
        """将终端设为非阻塞原始模式（无需回车即可读取按键）"""
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def _disable_raw_mode(self):
        """恢复终端正常模式"""
        if self.old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def _get_key(self):
        """非阻塞读取单个按键（超时 50ms）"""
        if select.select([sys.stdin], [], [], 0.05)[0]:
            return sys.stdin.read(1)
        return None

    def _send_state(self):
        """发送当前笔状态到仿真器"""
        msg = json.dumps({
            "pos": self.pos,
            "vel": [0.0, 0.0, 0.0],
            "buttons": self.buttons,
        })
        try:
            self.send_sock.sendto(msg.encode("utf-8"), self.send_addr)
        except OSError:
            pass

    def _recv_force(self):
        """接收仿真器发来的力反馈数据"""
        try:
            while True:
                try:
                    data, _ = self.recv_sock.recvfrom(4096)
                    msg = json.loads(data.decode("utf-8"))
                    self.last_force = msg.get("force", [0, 0, 0])
                except BlockingIOError:
                    break
        except Exception:
            pass

    def _print_help(self):
        """打印操作说明"""
        print("\n" + "=" * 60)
        print("  力反馈笔键盘模拟客户端")
        print("=" * 60)
        print("  移动控制:")
        print("    W / S  - 前进 / 后退  (Z 轴)")
        print("    A / D  - 左移 / 右移  (X 轴)")
        print("    Q / E  - 上升 / 下降  (Y 轴)")
        print()
        print("  夹爪控制:")
        print("    1      - 闭合夹爪 (Button1)")
        print("    2      - 张开夹爪 (Button2)")
        print()
        print("  其他:")
        print("    + / -  - 增大 / 减小 移动步长")
        print("    R      - 重置位置到原点")
        print("    H      - 显示帮助")
        print("    Ctrl+C - 退出")
        print("=" * 60)
        print()

    def _print_status(self):
        """打印当前状态"""
        force_mag = (self.last_force[0]**2 +
                     self.last_force[1]**2 +
                     self.last_force[2]**2) ** 0.5
        contact = " [接触!]" if force_mag > 0.01 else ""

        btn_str = ""
        if self.buttons[0]:
            btn_str = " [夹爪闭合]"
        elif self.buttons[1]:
            btn_str = " [夹爪张开]"

        sys.stdout.write(
            f"\r  位置(mm): X={self.pos[0]:+7.1f}  "
            f"Y={self.pos[1]:+7.1f}  "
            f"Z={self.pos[2]:+7.1f}  "
            f"| 步长:{self.step_size:.1f}mm"
            f"  | 力:{force_mag:.3f}N{contact}{btn_str}    "
        )
        sys.stdout.flush()

    def run(self):
        """主循环"""
        self._print_help()
        self._enable_raw_mode()

        try:
            while self.running:
                # 读取按键
                key = self._get_key()

                if key:
                    moved = False

                    # 移动控制
                    if key in ("w", "W"):
                        self.pos[2] += self.step_size  # Z+（前进）
                        moved = True
                    elif key in ("s", "S"):
                        self.pos[2] -= self.step_size  # Z-（后退）
                        moved = True
                    elif key in ("a", "A"):
                        self.pos[0] -= self.step_size  # X-（左移）
                        moved = True
                    elif key in ("d", "D"):
                        self.pos[0] += self.step_size  # X+（右移）
                        moved = True
                    elif key in ("q", "Q"):
                        self.pos[1] += self.step_size  # Y+（上升）
                        moved = True
                    elif key in ("e", "E"):
                        self.pos[1] -= self.step_size  # Y-（下降）
                        moved = True

                    # 夹爪控制
                    elif key == "1":
                        self.buttons = [1, 0]  # 闭合
                    elif key == "2":
                        self.buttons = [0, 1]  # 张开

                    # 步长调整
                    elif key in ("+", "="):
                        self.step_size = min(50.0, self.step_size + 1.0)
                    elif key in ("-", "_"):
                        self.step_size = max(0.5, self.step_size - 1.0)

                    # 重置位置
                    elif key in ("r", "R"):
                        self.pos = [0.0, 0.0, 0.0]
                        self.buttons = [0, 0]
                        moved = True

                    # 帮助
                    elif key in ("h", "H"):
                        self._print_help()

                # 发送状态（即使没有按键，也定期发送保持连接）
                self._send_state()

                # 接收力反馈
                self._recv_force()

                # 打印状态
                self._print_status()

                # 释放按钮（按钮为瞬时触发，持续按住需要持续按键）
                # 注意：这里按钮保持状态直到另一个按钮被按下
                # （与真实 Touch 设备行为一致）

        finally:
            self._disable_raw_mode()
            self.send_sock.close()
            self.recv_sock.close()
            print("\n\n  客户端已退出。")


if __name__ == "__main__":
    args = parse_args()
    client = KeyboardPenClient(args)
    client.run()
