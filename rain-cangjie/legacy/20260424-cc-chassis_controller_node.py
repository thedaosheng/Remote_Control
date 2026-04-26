#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
====================================================================
  chassis_controller_node — /cmd_vel → 舵轮 IK → (mock / CAN)
====================================================================

用途: 在 CangJie 底盘 Pi 上跑, 订阅跨机来的 /cmd_vel (Twist),
      做舵轮逆运动学解算 → 4 轮 (steer_rad, drive_rad_s),
      当前阶段走 MOCK (打印日志), 等 USB-CAN + 电机接上后切 REAL.

参考: 181 上 ~/teleop/scripts/20260420-cc-pi_kinematics_demo.py
      几何参数与 >90° 翻转优化逻辑完全一致, 便于切 REAL 时行为不变.

几何:
  FL (+0.16, +0.15) | FR (+0.16, -0.15)
  RL (-0.16, +0.15) | RR (-0.16, -0.15)
  轮径 0.06 m, MIT 舵向 @ 200Hz, ZLAC 驱动 @ 50Hz (real 模式)

运行:
  # mock 模式 (当前, 无 CAN)
  python3 20260424-cc-chassis_controller_node.py
  # 或者通过 ROS2 参数切:
  python3 20260424-cc-chassis_controller_node.py --ros-args -p mock:=true

  # real 模式 (USB-CAN 插好后)
  python3 20260424-cc-chassis_controller_node.py --ros-args -p mock:=false

Watchdog:
  - /cmd_vel 超过 WATCHDOG_TIMEOUT 秒没新消息 → 自动归零 (防 ssh 挂掉飞车)
"""

import json
import math
import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt64


# ============================================================
# 几何与控制参数 (与 pi_kinematics_demo.py 一致)
# ============================================================
WHEEL_POS = np.array([
    [+0.16, +0.15],   # 0: FL
    [+0.16, -0.15],   # 1: FR
    [-0.16, +0.15],   # 2: RL
    [-0.16, -0.15],   # 3: RR
], dtype=np.float64)
WHEEL_R = 0.06  # m

WHEEL_NAMES = ['FL', 'FR', 'RL', 'RR']

WATCHDOG_TIMEOUT = 0.5   # 秒, /cmd_vel 超时自动归零
# (MOCK_LOG_HZ 已移除 — 现在在 _on_cmd_vel 回调里即时打印, 仅在值变化时)
LOG_DEADBAND     = 1e-3  # 速度变化小于此不打印 (避免 20Hz 稳态刷屏)


# ============================================================
# 舵轮逆运动学 (与 181 demo 完全一致)
# ============================================================
def swerve_ik(vx, vy, omega):
    """
    车身系速度 (vx, vy, omega) → 每轮 (steer_rad, drive_rad_s)
    返回: steer[4], rad_s[4]
    """
    steer = np.zeros(4)
    rad_s = np.zeros(4)
    for i, (lx, ly) in enumerate(WHEEL_POS):
        wx = vx - omega * ly
        wy = vy + omega * lx
        v = math.hypot(wx, wy)
        if v > 1e-4:
            steer[i] = math.atan2(wy, wx)
            rad_s[i] = v / WHEEL_R
    return steer, rad_s


def optimize_wheel(new_steer, prev_steer, speed):
    """
    舵轮 >90° 翻转优化: 新目标若和当前差 >90°, 则旋转 180° + 反向驱动.
    物理等效, 但减少舵向大角度反转.
    """
    delta = (new_steer - prev_steer + math.pi) % (2 * math.pi) - math.pi
    if abs(delta) > math.pi / 2:
        new_steer = (new_steer + 2 * math.pi) % (2 * math.pi) - math.pi
        new_steer = new_steer - math.pi if new_steer > 0 else new_steer + math.pi
        speed = -speed
    return new_steer, speed


# ============================================================
# 节点
# ============================================================
class ChassisController(Node):
    def __init__(self):
        super().__init__('chassis_controller')

        # 参数
        self.declare_parameter('mock', True)
        self.mock_mode = self.get_parameter('mock').get_parameter_value().bool_value

        # 订阅 /cmd_vel
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self._on_cmd_vel, 10)

        # 发布 /chassis_state (JSON String) —— 供 cloud_teleop_bridge 回拉给 182
        self._latest_t_sent_ns = 0
        self.create_subscription(UInt64, '/cmd_t_sent_ns', lambda m: setattr(self, '_latest_t_sent_ns', int(m.data)), 10)
        self.state_pub = self.create_publisher(String, '/chassis_state', 10)

        # 共享状态 (latest cmd_vel)
        self._state_lock = threading.Lock()
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._last_rx_ts = 0.0
        self._cur_steer = np.zeros(4)  # >90° 优化需要记上次
        self._last_logged = (None, None, None)  # 上次打印的值, 用于 deadband

        # Watchdog 定时器 (20Hz 检查)
        self.wd_timer = self.create_timer(0.05, self._watchdog_tick)

        mode_str = 'MOCK (log only, NO CAN)' if self.mock_mode else 'REAL (CAN enabled)'
        self.get_logger().info(
            f'[chassis_controller] ready. mode={mode_str}  '
            f'listening /cmd_vel @ DOMAIN={self._domain()}')

        if not self.mock_mode:
            self.get_logger().warn(
                '[chassis_controller] REAL 模式未实现! 需要:\n'
                '  1. ZLAC 驱动板的 ACM0 CANopen 初始化\n'
                '  2. DM4310 MIT 舵向 ACM1 CAN 初始化\n'
                '  3. 200Hz steer_loop + 50Hz drive_loop (见 181 demo)\n'
                '  当前仅打印, 等 USB-CAN 接好再写.')

    def _domain(self):
        import os
        return os.environ.get('ROS_DOMAIN_ID', '0 (default)')

    def _on_cmd_vel(self, msg: Twist):
        t_rx = time.perf_counter_ns()
        with self._state_lock:
            self._vx = msg.linear.x
            self._vy = msg.linear.y
            self._wz = msg.angular.z
            self._last_rx_ts = time.monotonic()

        # 即时 IK 解算 (不再走 5Hz 节流)
        t_ik0 = time.perf_counter_ns()
        steer, rad_s = swerve_ik(self._vx, self._vy, self._wz)
        for i in range(4):
            steer[i], rad_s[i] = optimize_wheel(steer[i], self._cur_steer[i], rad_s[i])
        self._cur_steer = steer.copy()
        ik_us = (time.perf_counter_ns() - t_ik0) / 1000.0

        # 发布 /chassis_state (JSON) —— 总是发, 给 cloud bridge 回拉给 182
        state = {
            'type': 'chassis_state',
            'vx': float(self._vx), 'vy': float(self._vy), 'wz': float(self._wz),
            'wheels': [
                {
                    'name': WHEEL_NAMES[i],
                    'steer_deg': float(math.degrees(steer[i])),
                    'rad_s': float(rad_s[i]),
                    'rpm': float(rad_s[i] * 60.0 / (2 * math.pi)),
                }
                for i in range(4)
            ],
            'ik_us': round(ik_us, 1),
            't_cangjie_ns': time.time_ns(),
            't_sent_ns': self._latest_t_sent_ns,
            'mock': bool(self.mock_mode),
        }
        state_msg = String()
        state_msg.data = json.dumps(state)
        self.state_pub.publish(state_msg); self.get_logger().info(f"[state_pub] published seq (nothing to track)")

        # Deadband: 只在速度显著变化时打印, 避免 20Hz 稳态刷屏
        if self.mock_mode:
            cur = (round(self._vx, 3), round(self._vy, 3), round(self._wz, 3))
            if cur != self._last_logged:
                self._last_logged = cur
                cb_us = (time.perf_counter_ns() - t_rx) / 1000.0
                lines = [f'[MOCK] vx={self._vx:+.2f} vy={self._vy:+.2f} wz={self._wz:+.2f}  '
                         f'(IK {ik_us:.0f}μs, cb {cb_us:.0f}μs)']
                for i in range(4):
                    deg = math.degrees(steer[i])
                    rpm = rad_s[i] * 60.0 / (2 * math.pi)
                    lines.append(f'  {WHEEL_NAMES[i]}: steer={deg:+6.1f}°  '
                                 f'drive={rad_s[i]:+.2f} rad/s ({rpm:+5.1f} rpm)')
                self.get_logger().info('\n'.join(lines))

    def _watchdog_tick(self):
        if self._last_rx_ts == 0.0:
            return  # 还没收到任何 cmd_vel
        if time.monotonic() - self._last_rx_ts > WATCHDOG_TIMEOUT:
            with self._state_lock:
                if abs(self._vx) + abs(self._vy) + abs(self._wz) > 1e-6:
                    self.get_logger().warn(
                        f'[watchdog] /cmd_vel stale >{WATCHDOG_TIMEOUT}s, zeroing')
                    self._vx = self._vy = self._wz = 0.0


# ============================================================
# main
# ============================================================
def main():
    rclpy.init()
    node = ChassisController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('[chassis_controller] shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
