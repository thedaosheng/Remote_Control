#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
====================================================================
  keyboard_teleop_node — WASD 键盘 → /cmd_vel (geometry_msgs/Twist)
====================================================================

用途: 在 Orin 上跑, 人按键盘发 Twist, 跨机 DDS 到 CangJie 底盘 Pi 驱动舵轮.

键位:
  W / S   : +vx / -vx   (前进 / 后退)
  A / D   : +vy / -vy   (左横移 / 右横移)
  Q / E   : +wz / -wz   (左转 / 右转)
  SPACE   : 归零        (安全停车)
  Ctrl+C  : 退出节点

设计:
  - cbreak 模式读 stdin, 一次一字符, 无需 Enter
  - 累加步长, 按 Space 清零
  - @ 20Hz 持续发 /cmd_vel, 保证 chassis 端 watchdog 可判断"有人在操作"
  - 即使没按键也刷当前值, 断开 (ssh 挂) 时 CangJie 可超时归零自保
"""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ============================================================
# 可调参数
# ============================================================
LIN_STEP = 0.05      # m/s 每次按键增量
ANG_STEP = 0.20      # rad/s 每次按键增量
LIN_MAX  = 0.30      # 线速度上限 (安全)
ANG_MAX  = 1.20      # 角速度上限
PUB_HZ   = 20.0      # /cmd_vel 发布频率

# ============================================================
# 节点
# ============================================================
class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        # /cmd_vel 是 swerve chassis 的标准入口
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        # 定时发布: 保证稳定 stream, chassis 可 watchdog 检测链路
        self.timer = self.create_timer(1.0 / PUB_HZ, self._pub_tick)
        self.get_logger().info(
            f'[keyboard_teleop] ready. '
            f'WASD=平移 / QE=转向 / Space=停 / Ctrl+C=退出 '
            f'(lin_step={LIN_STEP} ang_step={ANG_STEP})'
        )

    def _pub_tick(self):
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.angular.z = self.wz
        self.pub.publish(msg)

    def on_key(self, ch):
        changed = True
        if   ch == 'w': self.vx = min(LIN_MAX, self.vx + LIN_STEP)
        elif ch == 's': self.vx = max(-LIN_MAX, self.vx - LIN_STEP)
        elif ch == 'a': self.vy = min(LIN_MAX, self.vy + LIN_STEP)
        elif ch == 'd': self.vy = max(-LIN_MAX, self.vy - LIN_STEP)
        elif ch == 'q': self.wz = min(ANG_MAX, self.wz + ANG_STEP)
        elif ch == 'e': self.wz = max(-ANG_MAX, self.wz - ANG_STEP)
        elif ch == ' ': self.vx, self.vy, self.wz = 0.0, 0.0, 0.0
        else: changed = False
        if changed:
            self.get_logger().info(
                f'cmd_vel: vx={self.vx:+.2f} vy={self.vy:+.2f} wz={self.wz:+.2f}')


# ============================================================
# 非阻塞读一个字符
# ============================================================
def read_key_nonblock(timeout_s=0.05):
    dr, _, _ = select.select([sys.stdin], [], [], timeout_s)
    if dr:
        return sys.stdin.read(1)
    return None


# ============================================================
# main
# ============================================================
def main():
    rclpy.init()
    node = KeyboardTeleop()

    # 保存 stdin 原始属性, 结束时还原
    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    try:
        # cbreak: 不等 Enter, 但保留 Ctrl+C 等特殊信号
        tty.setcbreak(fd)
        while rclpy.ok():
            # 推进 ROS2 回调 (包括 publisher timer)
            rclpy.spin_once(node, timeout_sec=0.01)
            ch = read_key_nonblock(timeout_s=0.05)
            if ch is None:
                continue
            if ch == '\x03':  # Ctrl+C
                node.get_logger().info('[keyboard_teleop] Ctrl+C, shutting down')
                break
            node.on_key(ch)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
        # 最后发一次全零, 避免 chassis 端最后一条是非零指令
        stop = Twist()
        node.pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
