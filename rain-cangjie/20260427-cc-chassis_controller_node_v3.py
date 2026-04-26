#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
====================================================================
  chassis_controller v3 — /cmd_vel + /lift_cmd → 舵轮 IK + DM3519 lift
====================================================================

用途: 在 CangJie 底盘 Pi 上跑, 在 v2 基础上**新增 lift (升降) 集成**.
      v2 只处理底盘 (4 轮舵向 + 驱动); v3 把 lift 也接进同一个 ROS 2 节点,
      让 ASUS GUI 的 G/H 键 (升 / 降) 能跟 WSAD/QE 走完全相同的链路.

订阅:
  /cmd_vel       (geometry_msgs/msg/Twist)   — 底盘 (vx, vy, wz) → 4 轮舵向 + 驱动
  /lift_cmd      (std_msgs/msg/Float32)      — 升降速度归一化 [-1.0, +1.0]
                                                 +1.0 = max 上行, -1.0 = max 下行, 0 = 停

发布:
  /chassis_state (std_msgs/msg/String, JSON) — v2 已有, v3 在 wheels 之外多带
                                                 lift_current_a 字段; 给 ros_to_cloud_bridge
                                                 回拉给 ASUS GUI 显示

硬件映射 (运行时不变, 留底):
  ACM0 @ 500 kbit CANopen → 2 × ZLAC8015D 驱动器 → 4 个驱动轮 (FL/FR/RL/RR)
  ACM1 @ 1 Mbit MIT/一拖四 → 4 × DM4310 (舵向, MIT 协议) + 1 × DM3519 (升降, 一拖四协议)
                            ↑ MIT 和 一拖四 共用一根 1M 总线, ID 空间不冲突

模式:
  --ros-args -p mock:=true   (默认; 只打印 CAN 帧, 不真发)
  --ros-args -p mock:=false  (USB-CAN 插好后; 真发, 当前 NotImplementedError)

Watchdog (互相独立, 都默认 0.5 s 超时归零):
  /cmd_vel  超时 → 4 轮 drive_rad_s 全部归零 (但保留舵向, 避免飞车)
  /lift_cmd 超时 → lift current 归零 (升降立即停)

参考几何 (与 181 上 ~/teleop/scripts/20260420-cc-pi_kinematics_demo.py 一致):
  WHEEL_POS = [(+0.16,+0.15), (+0.16,-0.15), (-0.16,+0.15), (-0.16,-0.15)]  # FL FR RL RR
  WHEEL_R   = 0.06 m

DM3519 一拖四协议 (来自 scripts/20260420-cc-pi_3519_test.py 的 verbatim 验证):
  发送帧 (无需使能, 直接发):
    CAN ID = 0x200, 8 字节 data
    Data[0:2] = M1 电流 int16 大端  (此处 M1 = lift 升降电机)
    Data[2:4] = M2 电流 = 0          (一拖四里的另外 3 个通道未用)
    Data[4:6] = M3 电流 = 0
    Data[6:8] = M4 电流 = 0
  映射:    int16 = current_A × 16384 / 20
  范围:    ±20 A 满量程 → int16 ±16384

  反馈帧 (电机主动广播, 留作 future feedback loop 引用):
    ID 0x201..0x204 = M1..M4
    Data[0:2] pos int16 → val/8192 × 360 = pos_deg
    Data[2:4] spd int16 (RPM)
    Data[4:6] cur int16 → val × 20 / 16384 = cur_A
    Data[6]   Tcoil °C
    Data[7]   Tmos  °C

>90° 翻转优化 (optimize_wheel):
  v3 用 181 demo 里的**正确版本**: 单次 flip_by_π + speed 反向.
  v2 (现在 legacy/) 有 bug: 先 wrap-into-[-π,π] (其实等于 flip), 再按符号 ± π,
  双 flip 等于 no-op 舵向, 只剩 speed 被反, 物理等效但优化意图丢失.
  详见 v3 commit message + agent-bus feedback.

运行:
  # mock (默认, 当前 USB-CAN 未插)
  python3 20260427-cc-chassis_controller_node_v3.py
  # 或显式
  python3 20260427-cc-chassis_controller_node_v3.py --ros-args -p mock:=true

  # 调 lift 上限 (默认 5 A, DM3519 ±20 A 满量程)
  python3 20260427-cc-chassis_controller_node_v3.py \
      --ros-args -p lift_max_current_a:=8.0

直测 (不通过 LiveKit):
  # 终端 A
  python3 20260427-cc-chassis_controller_node_v3.py
  # 终端 B
  ros2 topic pub -r 5 /lift_cmd std_msgs/msg/Float32 "data: 0.30"
  # A 应看到 [lift] norm=+0.30 → I=+1.50 A → CAN 0x200 [...]
"""

import json
import math
import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String, UInt64


# ============================================================
# 几何与控制参数 (与 v2 / 181 demo 一致)
# ============================================================
WHEEL_POS = np.array([
    [+0.16, +0.15],   # 0: FL
    [+0.16, -0.15],   # 1: FR
    [-0.16, +0.15],   # 2: RL
    [-0.16, -0.15],   # 3: RR
], dtype=np.float64)
WHEEL_R = 0.06  # m

WHEEL_NAMES = ['FL', 'FR', 'RL', 'RR']

# Watchdog 默认 0.5 s; ROS 2 param 可覆盖
DEFAULT_CMD_VEL_WATCHDOG_S = 0.5
DEFAULT_LIFT_WATCHDOG_S    = 0.5

# Lift 默认满量程 5 A (DM3519 物理上限 ±20 A); 可被 ROS 2 param 调
DEFAULT_LIFT_MAX_CURRENT_A = 5.0

# 速度变化 deadband (避免 20 Hz 稳态刷屏 chassis 日志)
LOG_DEADBAND = 1e-3


# ============================================================
# 舵轮逆运动学 (与 v2 / 181 demo 一致)
# ============================================================
def swerve_ik(vx, vy, omega):
    """
    车身系速度 (vx, vy, omega) → 每轮 (steer_rad, drive_rad_s).
    车身坐标: x 向前, y 向左, ω 逆时针为正.
    返回: steer[4], rad_s[4] (顺序 FL FR RL RR).
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
    舵轮 >90° 翻转优化 (canonical 版本, 与 181 kinematics_demo.py 一致):
      - 如果新目标和当前舵向差 > 90°, 整体反转 180° 同时 speed 取反 (物理等效)
      - 这样最大舵向跨度永远 ≤ 90°, 物理舵机不用跨大角度旋转 (节省时间 + 减少抖动)

    与 v2 (现在 legacy/) 的差异:
      v2:  先 (x + 2π) % 2π - π wrap (其实等价于 flip_by_π), 再按 sign ± π,
           双 flip = no-op 舵向, 只 speed 反 → 优化意图丢失.
      v3:  flip 一次 (x + 2π) % 2π - π, speed 反 → 净效果 = 舵向真翻 180°.
    """
    delta = (new_steer - prev_steer + math.pi) % (2 * math.pi) - math.pi
    if abs(delta) > math.pi / 2:
        new_steer = (new_steer + math.pi + math.pi) % (2 * math.pi) - math.pi
        speed = -speed
    return new_steer, speed


# ============================================================
# DM3519 一拖四 frame 构造
# ============================================================
def dm3519_lift_frame(lift_current_a, m2_a=0.0, m3_a=0.0, m4_a=0.0):
    """
    构造 DM3519 一拖四 0x200 电流控制帧 (8 byte data) — M1 = 升降.

    映射:  int16 = current_A × 16384 / 20  (满量程 ±20 A → ±16384)
    顺序:  [M1_hi, M1_lo, M2_hi, M2_lo, M3_hi, M3_lo, M4_hi, M4_lo]  (大端)

    返回: (data: bytes 8B, m1_int16: int) — int16 用于日志显示
    """
    def to_int16(a):
        v = int(round(a * 16384.0 / 20.0))
        v = max(-32768, min(32767, v))
        return v

    v1 = to_int16(lift_current_a)
    v2 = to_int16(m2_a); v3 = to_int16(m3_a); v4 = to_int16(m4_a)

    data = bytes([
        (v1 >> 8) & 0xFF, v1 & 0xFF,
        (v2 >> 8) & 0xFF, v2 & 0xFF,
        (v3 >> 8) & 0xFF, v3 & 0xFF,
        (v4 >> 8) & 0xFF, v4 & 0xFF,
    ])
    return data, v1


# ============================================================
# 节点
# ============================================================
class ChassisControllerV3(Node):
    def __init__(self):
        super().__init__('chassis_controller_v3')

        # ---- 参数 ----
        self.declare_parameter('mock', True)
        self.declare_parameter('cmd_vel_watchdog_s', DEFAULT_CMD_VEL_WATCHDOG_S)
        self.declare_parameter('lift_watchdog_s',    DEFAULT_LIFT_WATCHDOG_S)
        self.declare_parameter('lift_max_current_a', DEFAULT_LIFT_MAX_CURRENT_A)

        self.mock_mode      = self.get_parameter('mock').get_parameter_value().bool_value
        self.cmd_vel_wdt_s  = float(self.get_parameter('cmd_vel_watchdog_s').value)
        self.lift_wdt_s     = float(self.get_parameter('lift_watchdog_s').value)
        self.lift_max_a     = float(self.get_parameter('lift_max_current_a').value)

        # ---- 共享状态 (单一锁; chassis + lift 都很轻, 不必拆) ----
        self._state_lock = threading.Lock()
        # chassis
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._cur_steer = np.zeros(4)         # 上次舵向, 给 optimize_wheel 用
        self._cur_rad_s = np.zeros(4)         # 上次驱动速度, watchdog 归零判断用
        self._last_cmd_vel_t = 0.0            # monotonic 秒, 0 = 还没收到
        self._last_logged_chassis = (None, None, None)  # deadband
        # lift
        self._lift_norm = 0.0                 # 归一化 [-1, +1]
        self._lift_current_a = 0.0            # 实际电流值 A (= norm × max_a)
        self._last_lift_t = 0.0
        # cmd_vel 时戳直通 (供 ros_to_cloud_bridge 计延时)
        self._latest_t_sent_ns = 0

        # ---- ROS 2 接口 ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self._on_cmd_vel, qos)
        self.sub_lift = self.create_subscription(
            Float32, '/lift_cmd', self._on_lift_cmd, qos)
        self.sub_t_sent = self.create_subscription(
            UInt64, '/cmd_t_sent_ns',
            lambda m: setattr(self, '_latest_t_sent_ns', int(m.data)), 10)

        self.state_pub = self.create_publisher(String, '/chassis_state', 10)

        # ---- 定时器 ----
        # Watchdog @ 20 Hz (50 ms tick) — 同时检 cmd_vel + lift
        self.create_timer(0.05, self._watchdog_tick)
        # 状态 publish @ 10 Hz — 给云端 GUI 回显; chassis_safety 也可订
        self.create_timer(0.1, self._publish_state)

        # ---- 启动日志 ----
        mode_str = 'MOCK (log only, NO CAN)' if self.mock_mode else 'REAL (CAN enabled)'
        self.get_logger().info(
            f'[chassis_v3] ready · mode={mode_str} · domain={self._domain()}')
        self.get_logger().info(
            f'[chassis_v3] subs: /cmd_vel /lift_cmd · pub: /chassis_state · '
            f'cmd_vel_wdt={self.cmd_vel_wdt_s}s · lift_wdt={self.lift_wdt_s}s · '
            f'lift_max={self.lift_max_a:.1f} A')

        if not self.mock_mode:
            self._init_can_real()  # 真模式占位, 当前 raise NotImplementedError
            self.get_logger().warn(
                '[chassis_v3] REAL 模式还没接 CAN! 等 USB-CAN 插好后另起一个 PR 实装:\n'
                '  - ACM0 ZLAC8015D 4 轮驱动 (CANopen 500 kbit, 见 scripts/...zlac_*.py)\n'
                '  - ACM1 DM4310 舵向 (MIT 协议) + DM3519 lift (一拖四) 共享 1 Mbit\n'
                '  - 200 Hz steer_loop + 50 Hz drive_loop + 100 Hz lift_loop')

    # ---------------------------------------------------------------
    # /cmd_vel 处理 (与 v2 行为一致, 但用了 canonical optimize_wheel)
    # ---------------------------------------------------------------
    def _on_cmd_vel(self, msg: Twist):
        t_rx = time.perf_counter_ns()
        with self._state_lock:
            self._vx = msg.linear.x
            self._vy = msg.linear.y
            self._wz = msg.angular.z
            self._last_cmd_vel_t = time.monotonic()

        # IK 解算 (即时, 不节流)
        t_ik0 = time.perf_counter_ns()
        steer, rad_s = swerve_ik(self._vx, self._vy, self._wz)
        for i in range(4):
            steer[i], rad_s[i] = optimize_wheel(steer[i], self._cur_steer[i], rad_s[i])
        with self._state_lock:
            self._cur_steer = steer.copy()
            self._cur_rad_s = rad_s.copy()
        ik_us = (time.perf_counter_ns() - t_ik0) / 1000.0

        # MOCK: deadband 后打印
        if self.mock_mode:
            cur = (round(self._vx, 3), round(self._vy, 3), round(self._wz, 3))
            if cur != self._last_logged_chassis:
                self._last_logged_chassis = cur
                cb_us = (time.perf_counter_ns() - t_rx) / 1000.0
                lines = [f'[chassis] vx={self._vx:+.2f} vy={self._vy:+.2f} '
                         f'wz={self._wz:+.2f}  (IK {ik_us:.0f}μs, cb {cb_us:.0f}μs)']
                for i in range(4):
                    deg = math.degrees(steer[i])
                    rpm = rad_s[i] * 60.0 / (2 * math.pi)
                    lines.append(f'  {WHEEL_NAMES[i]}: steer={deg:+6.1f}°  '
                                 f'drive={rad_s[i]:+.2f} rad/s ({rpm:+5.1f} rpm)')
                self.get_logger().info('\n'.join(lines))
        else:
            self._send_chassis_can_real(steer, rad_s)

    # ---------------------------------------------------------------
    # /lift_cmd 处理 (v3 新增)
    # ---------------------------------------------------------------
    def _on_lift_cmd(self, msg: Float32):
        t_rx = time.perf_counter_ns()
        # clamp 归一化值, 避免上游错发 ±10
        norm = max(-1.0, min(1.0, float(msg.data)))
        with self._state_lock:
            self._lift_norm = norm
            self._lift_current_a = norm * self.lift_max_a
            self._last_lift_t = time.monotonic()

        # 构造 DM3519 0x200 帧
        data, m1_int16 = dm3519_lift_frame(self._lift_current_a)
        cb_us = (time.perf_counter_ns() - t_rx) / 1000.0

        if self.mock_mode:
            hex_bytes = ' '.join(f'0x{b:02x}' for b in data)
            self.get_logger().info(
                f'[lift] norm={norm:+.2f} → I={self._lift_current_a:+.2f} A → '
                f'int16={m1_int16:+d} → CAN 0x200 [{hex_bytes}]  (cb {cb_us:.0f}μs)')
        else:
            self._send_lift_can_real(data)

    # ---------------------------------------------------------------
    # Watchdog: 0.5 s 没新消息 → 归零 (chassis + lift 互相独立)
    # ---------------------------------------------------------------
    def _watchdog_tick(self):
        now = time.monotonic()
        with self._state_lock:
            cmd_active = self._last_cmd_vel_t > 0.0
            lift_active = self._last_lift_t > 0.0
            cmd_stale = cmd_active and (now - self._last_cmd_vel_t) > self.cmd_vel_wdt_s
            lift_stale = lift_active and (now - self._last_lift_t) > self.lift_wdt_s

            cmd_nonzero = abs(self._vx) + abs(self._vy) + abs(self._wz) > 1e-6
            lift_nonzero = abs(self._lift_current_a) > 1e-6

            do_zero_cmd = cmd_stale and cmd_nonzero
            do_zero_lift = lift_stale and lift_nonzero

            if do_zero_cmd:
                self._vx = self._vy = self._wz = 0.0
                self._cur_rad_s = np.zeros(4)
                # 注意: 不动 _cur_steer, 防止舵向乱跳; 只让驱动停
            if do_zero_lift:
                self._lift_norm = 0.0
                self._lift_current_a = 0.0

        if do_zero_cmd:
            self.get_logger().warn(
                f'[watchdog] /cmd_vel stale > {self.cmd_vel_wdt_s}s → wheels zeroed')
            if not self.mock_mode:
                self._send_chassis_can_real(self._cur_steer, np.zeros(4))
        if do_zero_lift:
            self.get_logger().warn(
                f'[watchdog] /lift_cmd stale > {self.lift_wdt_s}s → lift zeroed')
            if not self.mock_mode:
                data, _ = dm3519_lift_frame(0.0)
                self._send_lift_can_real(data)

    # ---------------------------------------------------------------
    # 状态 publisher (10 Hz; 给云端回显)
    # ---------------------------------------------------------------
    def _publish_state(self):
        with self._state_lock:
            steer = self._cur_steer.copy()
            rad_s = self._cur_rad_s.copy()
            vx, vy, wz = self._vx, self._vy, self._wz
            lift_norm = self._lift_norm
            lift_a = self._lift_current_a
            t_sent = self._latest_t_sent_ns

        state = {
            'type': 'chassis_state',
            'vx': float(vx), 'vy': float(vy), 'wz': float(wz),
            'wheels': [
                {
                    'name': WHEEL_NAMES[i],
                    'steer_deg': float(math.degrees(steer[i])),
                    'rad_s': float(rad_s[i]),
                    'rpm': float(rad_s[i] * 60.0 / (2 * math.pi)),
                }
                for i in range(4)
            ],
            'lift': {
                'norm': float(lift_norm),
                'current_a': float(lift_a),
                'max_current_a': float(self.lift_max_a),
            },
            't_cangjie_ns': time.time_ns(),
            't_sent_ns': int(t_sent),
            'mock': bool(self.mock_mode),
        }
        m = String(); m.data = json.dumps(state)
        self.state_pub.publish(m)

    # ---------------------------------------------------------------
    # REAL 模式占位 (USB-CAN 插好后另起 PR 实装)
    # ---------------------------------------------------------------
    def _init_can_real(self):
        raise NotImplementedError(
            'REAL 模式未接线: 需要 ACM0 (ZLAC CANopen) + ACM1 (DM4310 MIT + '
            'DM3519 一拖四) 都 enumerate 后另起 PR. 当前留空只能跑 mock=true.')

    def _send_chassis_can_real(self, steer, rad_s):
        raise NotImplementedError('REAL 模式 chassis CAN 还没实装')

    def _send_lift_can_real(self, data: bytes):
        raise NotImplementedError('REAL 模式 lift CAN 还没实装')

    def _domain(self):
        import os
        return os.environ.get('ROS_DOMAIN_ID', '0 (default)')


# ============================================================
# main
# ============================================================
def main():
    rclpy.init()
    node = ChassisControllerV3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('[chassis_v3] shutting down')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
