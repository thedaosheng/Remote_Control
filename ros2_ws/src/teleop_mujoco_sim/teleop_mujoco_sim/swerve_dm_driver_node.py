#!/usr/bin/env python3
"""
==============================================================
  舵轮底盘 DM4310 电机驱动节点
==============================================================

ROS2 节点：订阅 /mujoco/swerve_cmd → 驱动真实达妙 DM4310 电机

数据流:
  keyboard_teleop_node → /cmd_vel (Twist)
  mujoco_sim_node      → /mujoco/swerve_cmd (Float64MultiArray[8])
  本节点               → USB-CAN → DM4310 × 4

swerve_cmd 格式 (8 个 double):
  [0..3] steer: FL, FR, RL, RR 转向角 (rad)
  [4..7] drive: FL, FR, RL, RR 驱动速度 (rad/s)

电机布局:
  FL = SlaveID 0x03  (MasterID 0x13)  左前
  RL = SlaveID 0x04  (MasterID 0x14)  左后 ← 当前故障, 自动跳过
  RR = SlaveID 0x05  (MasterID 0x15)  右后
  FR = SlaveID 0x06  (MasterID 0x16)  右前

控制模式:
  所有电机使用 MIT 阻抗控制 (位置+速度+力矩+Kp+Kd)
  舵轮电机同时承担转向和驱动功能, 用 MIT 模式统一控制

运行:
  ros2 run teleop_mujoco_sim swerve_dm_driver_node
  ros2 run teleop_mujoco_sim swerve_dm_driver_node --ros-args -p serial_port:=/dev/ttyACM2
"""

import sys
import time
import math
import threading

# 达妙 CAN 库
sys.path.insert(0, '/home/rhz/teleop/DM_Control_Python')
from DM_CAN import (Motor, MotorControl, DM_Motor_Type, DM_variable,
                     Control_Type)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


# ========================= 默认参数 =========================

# 电机布局: 索引 0=FL, 1=FR, 2=RL, 3=RR
# swerve_cmd 的顺序是 [FL, FR, RL, RR]，这里映射到实际 SlaveID
# SlaveID / MasterID
MOTOR_CONFIG = [
    (0x03, 0x13),   # FL — 左前
    (0x06, 0x16),   # FR — 右前
    (0x04, 0x14),   # RL — 左后 (当前故障, 自动跳过)
    (0x05, 0x15),   # RR — 右后
]

DEFAULT_PORT = '/dev/ttyACM2'
BAUD = 921600

# MIT 控制参数
# 转向: 高 Kp (位置跟踪) + 中等 Kd (阻尼)
STEER_KP = 15.0     # 位置刚度 — 控制转向角精度
STEER_KD = 1.5      # 速度阻尼 — 防止转向振荡

# 驱动: 低 Kp + 高 Kd (速度跟踪)
# MIT 模式下速度控制: kp=0, kd=高, q=0, dq=目标速度, tau=0
DRIVE_KD = 3.0       # 速度跟踪增益

# 安全限幅
STEER_POS_LIMIT = 3.14   # rad — 转向角限幅 (±180°)
DRIVE_VEL_LIMIT = 20.0   # rad/s — 驱动速度限幅

# 超时保护: 多久没收到指令就停止电机
CMD_TIMEOUT = 0.5    # 秒


class SwerveDMDriverNode(Node):
    """
    舵轮 DM4310 电机驱动节点

    订阅 /mujoco/swerve_cmd, 解析转向角和驱动速度,
    通过 MIT 阻抗控制发送给 4 个 DM4310 电机。
    """

    def __init__(self):
        super().__init__('swerve_dm_driver_node')

        # ---- ROS2 参数 ----
        self.declare_parameter('serial_port', DEFAULT_PORT)
        self.declare_parameter('steer_kp', STEER_KP)
        self.declare_parameter('steer_kd', STEER_KD)
        self.declare_parameter('drive_kd', DRIVE_KD)

        port = self.get_parameter('serial_port').value
        self.steer_kp = self.get_parameter('steer_kp').value
        self.steer_kd = self.get_parameter('steer_kd').value
        self.drive_kd = self.get_parameter('drive_kd').value

        self.get_logger().info('=' * 50)
        self.get_logger().info('舵轮 DM4310 电机驱动节点')
        self.get_logger().info('  串口: %s' % port)
        self.get_logger().info('  转向 Kp=%.1f Kd=%.1f  驱动 Kd=%.1f' % (
            self.steer_kp, self.steer_kd, self.drive_kd))
        self.get_logger().info('=' * 50)

        # ---- 达妙电机初始化 ----
        import serial
        self.ser = serial.Serial(port, BAUD, timeout=0.5)
        self.mc = MotorControl(self.ser)

        # 创建 4 个电机对象, 检测哪些在线
        self.motors = []         # Motor 对象列表 (4 个, 离线的为 None)
        self.motor_online = []   # 布尔列表
        labels = ['FL', 'FR', 'RL', 'RR']

        for i, (sid, mid) in enumerate(MOTOR_CONFIG):
            motor = Motor(DM_Motor_Type.DM4310, sid, mid)
            self.mc.addMotor(motor)

            # 尝试读参数确认在线
            pmax = self.mc.read_motor_param(motor, DM_variable.PMAX)
            if pmax is not None:
                self.motors.append(motor)
                self.motor_online.append(True)
                self.get_logger().info(
                    '  ✓ %s (ID=0x%02X): 在线  PMAX=%.1f' % (labels[i], sid, pmax))
            else:
                self.motors.append(None)
                self.motor_online.append(False)
                self.get_logger().warn(
                    '  ✗ %s (ID=0x%02X): 离线, 跳过' % (labels[i], sid))

        online_count = sum(self.motor_online)
        if online_count == 0:
            self.get_logger().error('没有在线电机, 退出!')
            raise RuntimeError('No motors online')

        self.get_logger().info('  在线: %d/4' % online_count)

        # ---- 设零位 → 切模式 → 使能 (必须按此顺序) ----
        # set_zero_position 必须在使能之前调用
        # 调用后电机会失能，需要重新 switchControlMode + enable
        self.get_logger().info('设置零位 (使能前)...')
        for i, motor in enumerate(self.motors):
            if motor is None:
                continue
            self.mc.set_zero_position(motor)
            time.sleep(0.15)
            self.get_logger().info('  %s (0x%02X): 零位已设' % (
                labels[i], MOTOR_CONFIG[i][0]))

        self.get_logger().info('切换 MIT 模式并使能...')
        for i, motor in enumerate(self.motors):
            if motor is None:
                continue
            self.mc.switchControlMode(motor, Control_Type.MIT)
            time.sleep(0.05)
            self.mc.enable(motor)
            time.sleep(0.05)
            self.get_logger().info('  %s (0x%02X): 已使能' % (
                labels[i], MOTOR_CONFIG[i][0]))

        # ---- 状态变量 ----
        self.lock = threading.Lock()
        self.target_steer = [0.0] * 4    # 4 个转向角目标
        self.target_drive = [0.0] * 4    # 4 个驱动速度目标
        self.last_cmd_time = 0.0
        self.running = True

        # ---- ROS2 订阅: swerve_cmd ----
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.sub_cmd = self.create_subscription(
            Float64MultiArray, '/mujoco/swerve_cmd',
            self.cmd_callback, qos)

        # ---- ROS2 发布: 电机反馈 ----
        self.pub_feedback = self.create_publisher(
            JointState, '/swerve/motor_feedback', qos)

        # ---- 控制循环定时器 (200Hz) ----
        self.ctrl_timer = self.create_timer(1.0 / 200.0, self.control_loop)

        # ---- 反馈发布定时器 (20Hz) ----
        self.fb_timer = self.create_timer(1.0 / 20.0, self.publish_feedback)

        self.get_logger().info('✓ 驱动就绪, 等待 /mujoco/swerve_cmd ...')

    def cmd_callback(self, msg):
        """
        接收 swerve_cmd (8 个 double):
          [0..3] steer: FL, FR, RL, RR 转向角 (rad)
          [4..7] drive: FL, FR, RL, RR 驱动速度 (rad/s)
        """
        if len(msg.data) < 8:
            return
        with self.lock:
            for i in range(4):
                self.target_steer[i] = msg.data[i]
                self.target_drive[i] = msg.data[4 + i]
            self.last_cmd_time = time.time()

    def control_loop(self):
        """
        200Hz 电机控制循环。

        这些电机是纯转向电机，只控制舵向角，不负责驱动。
        使用 MIT 模式做位置跟踪:
          controlMIT(kp, kd, q=目标转向角, dq=0, tau=0)

        按 W/S 时转向角为 0°（直行），电机不动。
        按 A/D 时转向角为 ±90°（横移），电机转到对应角度。
        按 Q/E 时各轮交叉角度（原地旋转）。
        """
        if not self.running:
            return

        with self.lock:
            steer = list(self.target_steer)
            elapsed = time.time() - self.last_cmd_time if self.last_cmd_time > 0 else 999

        # 发送 MIT 控制命令: 纯位置跟踪，dq=0
        for i, motor in enumerate(self.motors):
            if motor is None:
                continue

            # 目标转向角 (零位已在启动时设好)
            q = max(-STEER_POS_LIMIT, min(STEER_POS_LIMIT, steer[i]))

            try:
                self.mc.controlMIT(
                    motor,
                    self.steer_kp,   # Kp: 位置刚度 → 转向精度
                    self.steer_kd,   # Kd: 速度阻尼 → 防振荡
                    q,               # q: 目标转向角 (含偏移)
                    0.0,             # dq: 0 — 纯位置控制
                    0.0              # tau: 无前馈力矩
                )
            except Exception:
                pass

    def publish_feedback(self):
        """20Hz 发布电机反馈状态"""
        labels = ['fl_wheel', 'fr_wheel', 'rl_wheel', 'rr_wheel']
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for i, motor in enumerate(self.motors):
            if motor is None:
                # 离线电机填零
                msg.name.append(labels[i])
                msg.position.append(0.0)
                msg.velocity.append(0.0)
                msg.effort.append(0.0)
            else:
                msg.name.append(labels[i])
                msg.position.append(float(motor.getPosition()))
                msg.velocity.append(float(motor.getVelocity()))
                msg.effort.append(float(motor.getTorque()))

        self.pub_feedback.publish(msg)

    def destroy_node(self):
        """安全关闭: 停止并失能所有电机"""
        self.running = False
        self.get_logger().info('关闭中...')

        # 先发零速命令
        for motor in self.motors:
            if motor is None:
                continue
            try:
                self.mc.controlMIT(motor, 0.0, 1.0, 0.0, 0.0, 0.0)
            except Exception:
                pass
        time.sleep(0.05)

        # 失能
        for i, motor in enumerate(self.motors):
            if motor is None:
                continue
            try:
                self.mc.disable(motor)
                self.get_logger().info('  ✓ 0x%02X 已失能' % MOTOR_CONFIG[i][0])
            except Exception:
                pass
            time.sleep(0.02)

        # 关闭串口
        try:
            self.ser.close()
        except Exception:
            pass

        super().destroy_node()
        self.get_logger().info('✓ 驱动节点已关闭')


def main(args=None):
    rclpy.init(args=args)
    node = SwerveDMDriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
