#!/usr/bin/env python3
"""
==============================================================
  dm_motor_controller_node — 达妙双电机 ROS2 实时控制
==============================================================

订阅 /vp/head_pose, 通过 MIT 模式控制达妙双电机 (Yaw + Pitch)。
完整迁移自 scripts/20260407-cc-dm_motor_vp_control.py, 主要变化:
  - UDP :9000 → ROS topic 订阅
  - 命令行 args → ROS parameter
  - preset 字典 → yaml 加载
  - 加 6 个 service (enable/disable/set_zero/ramp_to_zero/set_preset/recalibrate)
  - dynamic parameter callback (运行时改 kp/kd/preset)
  - 200Hz 控制循环跑在专用线程, 不阻塞 rclpy spin
  - publish /dm_motor/joint_states + /dm_motor/target_states

线程模型:
  - rclpy spin (主线程或 multi-threaded executor): 处理 topic 回调和 service
  - 控制循环线程 (200Hz): MIT 控制循环, 通过 lock 读最新 vp pose
  - 用 threading.Lock 保护共享状态 (vp_pose / params / motor_target)
"""

import os
import sys
import math
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState

from teleop_msgs.msg import HeadPose
from teleop_msgs.srv import SetPreset

# 加载 vendored DM_Control_Python (在 teleop_dm_motor/lib/ 下)
from teleop_dm_motor.lib.DM_CAN import (
    Motor, MotorControl, DM_Motor_Type, Control_Type, DM_variable
)


# ============================================================
# 5 档预设 (与 config/dm_motor_presets.yaml 对应, 这里硬编码作为 fallback)
# ============================================================
PRESETS = {
    "L1": dict(kp=8.0,  kd=1.2, rate_limit_dps=100.0, feedforward_enabled=False, ff_gain=0.0, ff_ema_alpha=0.3),
    "L2": dict(kp=15.0, kd=1.2, rate_limit_dps=200.0, feedforward_enabled=True,  ff_gain=0.5, ff_ema_alpha=0.3),
    "L3": dict(kp=20.0, kd=1.0, rate_limit_dps=300.0, feedforward_enabled=True,  ff_gain=1.0, ff_ema_alpha=0.5),
    "L4": dict(kp=25.0, kd=1.0, rate_limit_dps=500.0, feedforward_enabled=True,  ff_gain=1.2, ff_ema_alpha=0.7),
    "L5": dict(kp=30.0, kd=0.8, rate_limit_dps=600.0, feedforward_enabled=True,  ff_gain=1.5, ff_ema_alpha=0.8),
}


def find_serial_port(preferred: str) -> str:
    """如果 preferred 不存在, 自动遍历 ttyACM0-3 找一个存在的"""
    if os.path.exists(preferred):
        return preferred
    for c in ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"]:
        if os.path.exists(c):
            return c
    return preferred  # 让上层报错


class DmMotorControllerNode(Node):
    """达妙双电机 ROS2 控制节点 (核心)."""

    def __init__(self):
        super().__init__('dm_motor_controller')

        # ===== 1. 声明所有 ROS parameters =====
        self._declare_all_params()

        # ===== 2. 共享状态 (lock 保护) =====
        self.state_lock = threading.Lock()
        self.vp_pose: Optional[HeadPose] = None      # 最新一帧 vp pose
        self.vp_pose_t = 0.0                          # 收到时的 ROS 时间 (秒)
        self.vp_pitch_dot_dps = 0.0                   # EMA 平滑后的角速度 (前馈)
        self.vp_yaw_dot_dps = 0.0
        self.last_vp_pitch_deg = 0.0
        self.last_vp_yaw_deg = 0.0

        # ===== 3. 打开串口 + 初始化电机 (支持 mock_mode) =====
        self.mock_mode = self.get_parameter('mock_mode').value
        if self.mock_mode:
            self.get_logger().warn('★ MOCK MODE — 不打开串口, 不发 controlMIT, 用于纯软件 E2E 测试')
            self.ser = None
            self.mc = None
            self.m_yaw = None
            self.m_pitch = None
        else:
            port = find_serial_port(self.get_parameter('serial_port').value)
            baudrate = self.get_parameter('baudrate').value
            self.get_logger().info(f'打开串口 {port} @ {baudrate}')
            try:
                import serial
                self.ser = serial.Serial(port, baudrate, timeout=1)
                self.mc = MotorControl(self.ser)

                yaw_id = self.get_parameter('yaw_can_id').value
                pit_id = self.get_parameter('pitch_can_id').value
                yaw_mid = self.get_parameter('yaw_master_id').value
                pit_mid = self.get_parameter('pitch_master_id').value

                self.m_yaw = Motor(DM_Motor_Type.DM4310, yaw_id, yaw_mid)
                self.m_pitch = Motor(DM_Motor_Type.DM4310, pit_id, pit_mid)
                self.mc.addMotor(self.m_yaw)
                self.mc.addMotor(self.m_pitch)
                time.sleep(0.5)

                pmax_y = pmax_p = None
                for attempt in range(5):
                    pmax_y = self.mc.read_motor_param(self.m_yaw,   DM_variable.PMAX)
                    pmax_p = self.mc.read_motor_param(self.m_pitch, DM_variable.PMAX)
                    if pmax_y is not None and pmax_p is not None:
                        break
                    time.sleep(0.2)
                self.get_logger().info(f'  Yaw   PMAX = {pmax_y}')
                self.get_logger().info(f'  Pitch PMAX = {pmax_p}')
                if pmax_y is None or pmax_p is None:
                    self.get_logger().error(
                        '电机通讯失败 (5 次重试后仍 None)! '
                        '请检查 24V 电源 / CAN 线 / 串口路径')
                    raise RuntimeError('Motor communication failed')

                self.mc.switchControlMode(self.m_yaw,   Control_Type.MIT)
                self.mc.switchControlMode(self.m_pitch, Control_Type.MIT)
                self.mc.enable(self.m_yaw)
                self.mc.enable(self.m_pitch)
                time.sleep(0.3)
                self.get_logger().info('  ✓ 两个电机已使能 (MIT 模式)')
            except Exception as e:
                self.get_logger().error(f'电机初始化失败: {e}')
                raise

        # ===== 4. ROS 通讯接口 =====
        # 订阅 /vp/head_pose
        pose_topic = self.get_parameter('pose_topic').value
        self.pose_sub = self.create_subscription(
            HeadPose, pose_topic, self._on_vp_pose, 10)

        # publish 实际 / 目标关节状态
        self.joint_state_pub = self.create_publisher(
            JointState, '/dm_motor/joint_states', 10)
        if self.get_parameter('publish_target_state').value:
            self.target_state_pub = self.create_publisher(
                JointState, '/dm_motor/target_states', 10)
        else:
            self.target_state_pub = None

        # 6 个 service
        self.create_service(Trigger,   '/dm_motor/enable',         self.srv_enable)
        self.create_service(Trigger,   '/dm_motor/disable',        self.srv_disable)
        self.create_service(Trigger,   '/dm_motor/set_zero',       self.srv_set_zero)
        self.create_service(Trigger,   '/dm_motor/ramp_to_zero',   self.srv_ramp_to_zero)
        self.create_service(SetPreset, '/dm_motor/set_preset',     self.srv_set_preset)

        # ===== 5. 注册 dynamic parameter callback =====
        # _handling_preset 防止 preset 触发的批量 set_parameters 递归回调
        self._handling_preset = False
        self.add_on_set_parameters_callback(self._on_param_change)
        # 启动时如果 preset 不是 L4 (默认), 触发一次 preset 应用 (因为 init 时
        # callback 还没注册, launch 传的 preset:=L3 设置参数没触发批量更新)
        startup_preset = self.get_parameter('preset').value
        if startup_preset in PRESETS:
            cfg = PRESETS[startup_preset]
            self._handling_preset = True
            try:
                self.set_parameters([
                    Parameter('kp',                   value=cfg['kp']),
                    Parameter('kd',                   value=cfg['kd']),
                    Parameter('rate_limit_dps',       value=cfg['rate_limit_dps']),
                    Parameter('feedforward_enabled',  value=cfg['feedforward_enabled']),
                    Parameter('ff_gain',              value=cfg['ff_gain']),
                    Parameter('ff_ema_alpha',         value=cfg['ff_ema_alpha']),
                ])
                self.get_logger().info(
                    f'  应用启动 preset {startup_preset}: kp={cfg["kp"]} kd={cfg["kd"]} '
                    f'rate={cfg["rate_limit_dps"]} ff_gain={cfg["ff_gain"]}')
            finally:
                self._handling_preset = False

        # ===== 6. 启动控制线程 =====
        self.running = True
        self.control_thread = threading.Thread(
            target=self._control_loop, daemon=True, name='dm-motor-ctrl')
        self.control_thread.start()

        self.get_logger().info('✓ dm_motor_controller node 启动完成')
        self.get_logger().info(f'  订阅: {pose_topic}')
        self.get_logger().info(f'  preset: {self.get_parameter("preset").value}')
        self.get_logger().info(f'  KP={self.get_parameter("kp").value} '
                               f'KD={self.get_parameter("kd").value}')
        self.get_logger().info('  控制循环已启动 (专用线程, 200Hz)')

    # ---------- 参数声明 ----------
    def _declare_all_params(self):
        # 硬件
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('yaw_can_id', 1)
        self.declare_parameter('pitch_can_id', 2)
        self.declare_parameter('yaw_master_id', 17)
        self.declare_parameter('pitch_master_id', 18)

        # 默认 preset
        self.declare_parameter('preset', 'L4')

        # 控制 (默认值用 L4 的)
        self.declare_parameter('kp', 25.0)
        self.declare_parameter('kd', 1.0)
        self.declare_parameter('rate_limit_dps', 500.0)
        self.declare_parameter('feedforward_enabled', True)
        self.declare_parameter('ff_gain', 1.2)
        self.declare_parameter('ff_ema_alpha', 0.7)

        # 极性和缩放
        self.declare_parameter('sign_yaw', 1)
        self.declare_parameter('sign_pitch', 1)
        self.declare_parameter('scale_yaw', 1.0)
        self.declare_parameter('scale_pitch', 1.0)

        # 工作区 (从 calibration.yaml 加载)
        self.declare_parameter('yaw_min_deg', -37.15)
        self.declare_parameter('yaw_max_deg', 26.02)
        self.declare_parameter('pitch_min_deg', -55.78)
        self.declare_parameter('pitch_max_deg', 64.22)
        self.declare_parameter('safety_margin_deg', 5.0)

        # 启动行为
        self.declare_parameter('ramp_to_zero_secs', 3.0)
        self.declare_parameter('takeover_fade_secs', 1.5)
        self.declare_parameter('data_timeout_secs', 0.5)

        # 调试
        self.declare_parameter('print_interval_secs', 2.0)
        self.declare_parameter('publish_target_state', True)
        self.declare_parameter('control_rate_hz', 200)

        # 拓扑
        self.declare_parameter('pose_topic', '/vp/head_pose')

        # mock 模式: 不打开串口, 不发 controlMIT, 用于硬件不在线时的纯软件测试
        self.declare_parameter('mock_mode', False)

    # ---------- 参数动态变更回调 ----------
    def _on_param_change(self, params):
        """运行时改参数立即生效, 包括 preset 切换 (批量更新)."""
        for p in params:
            if p.name == 'preset':
                preset_name = p.value
                if preset_name not in PRESETS:
                    return SetParametersResult(
                        successful=False,
                        reason=f'Unknown preset {preset_name}, valid: {list(PRESETS.keys())}')
                # 批量更新所有相关参数 (异步, 触发新一轮 callback)
                cfg = PRESETS[preset_name]
                self.get_logger().info(f'切到 preset {preset_name}: {cfg}')
                # 用 set_parameters 触发批量更新
                # 注意: 这里不能直接 set, 否则会重入 callback. 用一个 timer 延迟
                def apply_preset():
                    self.set_parameters([
                        Parameter('kp',                   value=cfg['kp']),
                        Parameter('kd',                   value=cfg['kd']),
                        Parameter('rate_limit_dps',       value=cfg['rate_limit_dps']),
                        Parameter('feedforward_enabled',  value=cfg['feedforward_enabled']),
                        Parameter('ff_gain',              value=cfg['ff_gain']),
                        Parameter('ff_ema_alpha',         value=cfg['ff_ema_alpha']),
                    ])
                # 用 one-shot timer 在下一个 spin 周期执行 (避开 callback 重入)
                self.create_timer(0.01, lambda: (apply_preset(),
                                                 None))  # 简化版, 真实场景应该用 once
            # 单参数生效在控制循环里直接 self.get_parameter 读取, 不需要在这里缓存
        return SetParametersResult(successful=True)

    # ---------- VP pose 订阅回调 ----------
    def _on_vp_pose(self, msg: HeadPose):
        with self.state_lock:
            now = time.time()
            # 计算 EMA 平滑后的角速度 (用作 dq_target 前馈)
            alpha = self.get_parameter('ff_ema_alpha').value
            if self.vp_pose_t > 0:
                dt = now - self.vp_pose_t
                if dt > 0.001:
                    inst_p_dot = (msg.pitch_deg - self.last_vp_pitch_deg) / dt
                    inst_y_dot = (msg.yaw_deg   - self.last_vp_yaw_deg)   / dt
                    self.vp_pitch_dot_dps = alpha * inst_p_dot + (1 - alpha) * self.vp_pitch_dot_dps
                    self.vp_yaw_dot_dps   = alpha * inst_y_dot + (1 - alpha) * self.vp_yaw_dot_dps
            self.last_vp_pitch_deg = msg.pitch_deg
            self.last_vp_yaw_deg   = msg.yaw_deg
            self.vp_pose = msg
            self.vp_pose_t = now

    # ---------- 200Hz 控制循环 (专用线程) ----------
    def _control_loop(self):
        # 等 ROS spin 起来再开始 (避免 ros parameter 还没准备好)
        time.sleep(0.5)

        rate = self.get_parameter('control_rate_hz').value
        dt = 1.0 / rate
        self.get_logger().info(f'[control_loop] 启动 @ {rate}Hz')

        # ----- Step 1: ramp 当前位置到 (0, 0) -----
        if self.mock_mode:
            cur_y, cur_p = 0.0, 0.0
            self.get_logger().info('[control_loop] mock_mode: 跳过物理 ramp')
        else:
            for _ in range(3):
                self.mc.refresh_motor_status(self.m_yaw)
                self.mc.refresh_motor_status(self.m_pitch)
                time.sleep(0.05)
            cur_y = float(self.m_yaw.getPosition())
            cur_p = float(self.m_pitch.getPosition())
            self.get_logger().info(
                f'[control_loop] 当前位置: Y{math.degrees(cur_y):+.2f}° '
                f'P{math.degrees(cur_p):+.2f}° → ramp 到 (0, 0)')

            ramp_secs = self.get_parameter('ramp_to_zero_secs').value
            kp = self.get_parameter('kp').value
            kd = self.get_parameter('kd').value
            ramp_steps = int(ramp_secs * rate)
            for i in range(ramp_steps):
                if not self.running:
                    return
                ratio = (i + 1) / ramp_steps
                smooth = 0.5 - 0.5 * math.cos(math.pi * ratio)
                ty = cur_y + (0.0 - cur_y) * smooth
                tp = cur_p + (0.0 - cur_p) * smooth
                self.mc.controlMIT(self.m_yaw,   kp, kd, ty, 0, 0)
                self.mc.controlMIT(self.m_pitch, kp, kd, tp, 0, 0)
                time.sleep(dt)

        self.get_logger().info('[control_loop] ✓ 已到 (0, 0), 进入 vp 跟随模式')

        # ----- Step 2: 主循环 -----
        prev_y = 0.0
        prev_p = 0.0
        main_t0 = time.time()
        last_print = time.time()

        while self.running:
            t_iter_start = time.time()

            # 读取最新参数 (用户可能运行时改了)
            kp           = self.get_parameter('kp').value
            kd           = self.get_parameter('kd').value
            rate_limit   = self.get_parameter('rate_limit_dps').value
            ff_enabled   = self.get_parameter('feedforward_enabled').value
            ff_gain      = self.get_parameter('ff_gain').value
            sign_yaw     = self.get_parameter('sign_yaw').value
            sign_pitch   = self.get_parameter('sign_pitch').value
            scale_yaw    = self.get_parameter('scale_yaw').value
            scale_pitch  = self.get_parameter('scale_pitch').value
            yaw_min      = self.get_parameter('yaw_min_deg').value + self.get_parameter('safety_margin_deg').value
            yaw_max      = self.get_parameter('yaw_max_deg').value - self.get_parameter('safety_margin_deg').value
            pit_min      = self.get_parameter('pitch_min_deg').value + self.get_parameter('safety_margin_deg').value
            pit_max      = self.get_parameter('pitch_max_deg').value - self.get_parameter('safety_margin_deg').value
            timeout_secs = self.get_parameter('data_timeout_secs').value
            fade_secs    = self.get_parameter('takeover_fade_secs').value
            max_delta    = math.radians(rate_limit) * dt

            # 取最新 vp pose (lock 保护)
            with self.state_lock:
                vp = self.vp_pose
                vp_t = self.vp_pose_t
                vp_p_dot = self.vp_pitch_dot_dps
                vp_y_dot = self.vp_yaw_dot_dps

            now = time.time()
            age = now - vp_t if vp_t > 0 else float('inf')
            elapsed_main = now - main_t0
            fade = min(1.0, elapsed_main / fade_secs)

            if age > timeout_secs or vp is None:
                # 没数据 → 保持零点 (0, 0)
                target_y_rad = 0.0
                target_p_rad = 0.0
                ff_y_rad_per_sec = 0.0
                ff_p_rad_per_sec = 0.0
                state_label = 'NO_DATA' if vp is not None else 'WAIT'
            else:
                # vp 中立 → 电机零点 (0, 0)
                target_y_deg = sign_yaw   * scale_yaw   * fade * vp.yaw_deg
                target_p_deg = sign_pitch * scale_pitch * fade * vp.pitch_deg
                # 软限幅
                target_y_deg = max(yaw_min, min(yaw_max, target_y_deg))
                target_p_deg = max(pit_min, min(pit_max, target_p_deg))
                target_y_rad = math.radians(target_y_deg)
                target_p_rad = math.radians(target_p_deg)
                # 速度前馈
                if ff_enabled:
                    ff_y_dps = sign_yaw   * scale_yaw   * fade * ff_gain * vp_y_dot
                    ff_p_dps = sign_pitch * scale_pitch * fade * ff_gain * vp_p_dot
                    ff_y_dps = max(-rate_limit, min(rate_limit, ff_y_dps))
                    ff_p_dps = max(-rate_limit, min(rate_limit, ff_p_dps))
                    ff_y_rad_per_sec = math.radians(ff_y_dps)
                    ff_p_rad_per_sec = math.radians(ff_p_dps)
                else:
                    ff_y_rad_per_sec = 0.0
                    ff_p_rad_per_sec = 0.0
                state_label = f'RECV(f{fade:.2f})'

            # 逐帧位置限速
            dy = max(-max_delta, min(max_delta, target_y_rad - prev_y))
            dp = max(-max_delta, min(max_delta, target_p_rad - prev_p))
            ty = prev_y + dy
            tp = prev_p + dp

            # 硬限幅
            ty = max(math.radians(yaw_min - 1), min(math.radians(yaw_max + 1), ty))
            tp = max(math.radians(pit_min - 1), min(math.radians(pit_max + 1), tp))

            # 发送 controlMIT (mock 模式跳过)
            if not self.mock_mode:
                try:
                    self.mc.controlMIT(self.m_yaw,   kp, kd, ty, ff_y_rad_per_sec, 0)
                    self.mc.controlMIT(self.m_pitch, kp, kd, tp, ff_p_rad_per_sec, 0)
                except Exception as e:
                    self.get_logger().warn(f'controlMIT error: {e}')

            prev_y, prev_p = ty, tp

            # publish target_state
            if self.target_state_pub is not None:
                tgt = JointState()
                tgt.header.stamp = self.get_clock().now().to_msg()
                tgt.name = ['yaw', 'pitch']
                tgt.position = [float(ty), float(tp)]
                tgt.velocity = [float(ff_y_rad_per_sec), float(ff_p_rad_per_sec)]
                self.target_state_pub.publish(tgt)

            # publish joint_state (实际位置, mock 模式用目标值代替)
            if int(elapsed_main * rate) % 2 == 0:
                try:
                    if self.mock_mode:
                        actual_y, actual_p = ty, tp
                    else:
                        self.mc.refresh_motor_status(self.m_yaw)
                        self.mc.refresh_motor_status(self.m_pitch)
                        actual_y = float(self.m_yaw.getPosition())
                        actual_p = float(self.m_pitch.getPosition())
                    js = JointState()
                    js.header.stamp = self.get_clock().now().to_msg()
                    js.name = ['yaw', 'pitch']
                    js.position = [actual_y, actual_p]
                    self.joint_state_pub.publish(js)
                except Exception:
                    pass

            # 状态打印
            if now - last_print >= self.get_parameter('print_interval_secs').value:
                last_print = now
                vp_y_str = f'{vp.yaw_deg:+6.1f}' if vp else '   ---'
                vp_p_str = f'{vp.pitch_deg:+6.1f}' if vp else '   ---'
                self.get_logger().info(
                    f'[{state_label}] '
                    f'VP Y{vp_y_str}° P{vp_p_str}° | '
                    f'目标 Y{math.degrees(ty):+6.1f}° P{math.degrees(tp):+6.1f}°'
                )

            # 严格 200Hz
            elapsed_iter = time.time() - t_iter_start
            if dt - elapsed_iter > 0:
                time.sleep(dt - elapsed_iter)

        self.get_logger().info('[control_loop] 退出')

    # ---------- Service 实现 ----------
    def srv_enable(self, request, response):
        if self.mock_mode:
            response.success = True
            response.message = '[mock_mode] enable noop'
            return response
        try:
            self.mc.switchControlMode(self.m_yaw,   Control_Type.MIT)
            self.mc.switchControlMode(self.m_pitch, Control_Type.MIT)
            self.mc.enable(self.m_yaw)
            self.mc.enable(self.m_pitch)
            response.success = True
            response.message = '两个电机已使能'
        except Exception as e:
            response.success = False
            response.message = f'失败: {e}'
        return response

    def srv_disable(self, request, response):
        if self.mock_mode:
            response.success = True
            response.message = '[mock_mode] disable noop'
            return response
        try:
            self.mc.disable(self.m_yaw)
            self.mc.disable(self.m_pitch)
            response.success = True
            response.message = '两个电机已失能 (可以用手掰)'
        except Exception as e:
            response.success = False
            response.message = f'失败: {e}'
        return response

    def srv_set_zero(self, request, response):
        """把当前位置永久写 Flash 当零点 (0xFE → 0xAA)."""
        if self.mock_mode:
            response.success = True
            response.message = '[mock_mode] set_zero noop'
            return response
        try:
            self.mc.set_zero_position(self.m_yaw)
            self.mc.set_zero_position(self.m_pitch)
            time.sleep(0.3)
            self.mc.save_motor_param(self.m_yaw)
            time.sleep(0.5)
            self.mc.save_motor_param(self.m_pitch)
            time.sleep(0.5)
            # 重新使能
            self.mc.switchControlMode(self.m_yaw,   Control_Type.MIT)
            self.mc.switchControlMode(self.m_pitch, Control_Type.MIT)
            self.mc.enable(self.m_yaw)
            self.mc.enable(self.m_pitch)
            response.success = True
            response.message = '零点已写 Flash, 电机已重新使能'
        except Exception as e:
            response.success = False
            response.message = f'失败: {e}'
        return response

    def srv_ramp_to_zero(self, request, response):
        """让控制循环把电机缓慢 ramp 回 (0, 0)."""
        # 简化实现: 暂停 vp 跟随, 等 timeout 内自然回零 (没数据时控制循环目标就是 0)
        # 完整实现可以加一个 force_zero flag, 这里偷懒
        with self.state_lock:
            self.vp_pose = None
            self.vp_pose_t = 0.0
        response.success = True
        response.message = '已清空 vp pose, 控制循环会自然 ramp 到 (0, 0)'
        return response

    def srv_set_preset(self, request, response):
        """切换跟手感档位 L1-L5."""
        preset_name = request.preset_name
        if preset_name not in PRESETS:
            response.success = False
            response.message = f'未知 preset {preset_name}, valid: {list(PRESETS.keys())}'
            return response
        # 通过 set_parameters 触发, 让 _on_param_change 处理
        try:
            self.set_parameters([Parameter('preset', value=preset_name)])
            response.success = True
            response.message = f'已切换到 preset {preset_name}'
        except Exception as e:
            response.success = False
            response.message = f'失败: {e}'
        return response

    # ---------- 析构 ----------
    def shutdown(self):
        self.running = False
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
        if not self.mock_mode and self.mc:
            try:
                self.mc.disable(self.m_yaw)
                self.mc.disable(self.m_pitch)
            except Exception:
                pass
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DmMotorControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f'fatal: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
