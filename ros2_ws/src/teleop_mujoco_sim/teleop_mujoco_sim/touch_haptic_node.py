#!/usr/bin/env python3
"""
3D Systems Touch 力反馈笔 ROS2 控制节点

核心映射逻辑直接移植自已验证的 airbot_force_sim.py:
  位置: Touch mm → 坐标轴映射+缩放 → Robot m (世界系)
  姿态: Touch transform → 相对旋转 → 坐标系映射 → Robot 四元数
  力:   Robot cfrc_ext → 虚功转置 → Touch set_force()

数据流:
  Touch 笔 (1kHz HD API)
    → /arm/target_pose (PoseStamped)      Layer 2: 位置+姿态跟随
    → /arm/gripper_cmd (Float64)          夹爪控制
  /mujoco/end_effector_force (WrenchStamped)
    → Touch 笔 set_force()                Layer 4: 力反馈

运行:
  ros2 run teleop_mujoco_sim touch_haptic_node
"""

import os
import sys
import math
import time
import threading
import importlib.util

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float64

# MuJoCo 用于四元数转换
import mujoco


# ============== Touch 笔驱动加载 ==============
HAPTIC_DRIVER_PATH = os.path.join(
    os.path.expanduser('~'), 'teleop', 'force-feedback-pen',
    '20260328-cc-haptic_driver.py')

# libncurses 兼容库
_LIB_COMPAT = os.path.join(os.path.expanduser('~'), 'teleop', 'lib')
if os.path.isdir(_LIB_COMPAT) and _LIB_COMPAT not in os.environ.get('LD_LIBRARY_PATH', ''):
    os.environ['LD_LIBRARY_PATH'] = _LIB_COMPAT + ':' + os.environ.get('LD_LIBRARY_PATH', '')
    os.execv(sys.executable, [sys.executable] + sys.argv)


def _load_haptic_driver():
    """动态加载 haptic_driver.py 模块"""
    spec = importlib.util.spec_from_file_location('haptic_driver', HAPTIC_DRIVER_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ============== 坐标映射 (与 airbot_force_sim.py 完全一致) ==============
# Touch 笔坐标系:  X=用户右, Y=上, Z=朝向用户
# 机器人世界坐标系: X=前(远离底座), Y=左, Z=上
#
# 映射关系 (用户面向机器人):
#   Touch X  → Robot -Y  (取反: 笔的右 = 机器人的右 = -Y)
#   Touch Y  → Robot  Z  (上)
#   Touch Z  → Robot -X  (取反: 笔朝向用户 = 机器人的后 = -X)
#
# 旋转映射矩阵 R_map (行=Robot轴, 列=Touch轴):
#   R_map = [[ 0,  0, -1],    Robot X ← -Touch Z
#            [-1,  0,  0],    Robot Y ← -Touch X
#            [ 0,  1,  0]]    Robot Z ←  Touch Y
R_MAP = np.array([[ 0,  0, -1],
                   [-1,  0,  0],
                   [ 0,  1,  0]], dtype=float)

# 默认参数 (与 airbot_force_sim.py 的默认值一致)
DEFAULT_POS_SCALE = 0.003       # 1mm 笔 = 3mm 机器人
DEFAULT_ROT_SCALE = 1.5         # 姿态放大倍率
DEFAULT_FORCE_SCALE = 0.02      # 仿真力→笔力缩放
DEFAULT_MAX_PEN_FORCE = 3.0     # 笔最大输出力 (N)
DEFAULT_FORCE_FILTER_ALPHA = 0.3  # 力反馈低通滤波


class LowPassFilter:
    """一阶指数移动平均低通滤波器"""
    def __init__(self, alpha=0.3, dim=3):
        self.alpha = alpha
        self.value = np.zeros(dim)
        self._init = False

    def filter(self, raw):
        if not self._init:
            self.value = raw.copy()
            self._init = True
        else:
            self.value = self.alpha * raw + (1 - self.alpha) * self.value
        return self.value.copy()


def _scale_rotation(R_delta, scale):
    """对旋转矩阵做轴角放大 (Rodrigues 公式)

    从 airbot_force_sim.py 原封不动移植。
    用于把笔的小幅旋转放大为机器人的大幅旋转。
    """
    trace_val = np.clip((np.trace(R_delta) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(trace_val)
    if angle < 1e-6:
        return R_delta.copy()

    axis = np.array([
        R_delta[2, 1] - R_delta[1, 2],
        R_delta[0, 2] - R_delta[2, 0],
        R_delta[1, 0] - R_delta[0, 1]
    ]) / (2.0 * np.sin(angle))

    scaled_angle = angle * scale
    K = np.array([[0, -axis[2], axis[1]],
                   [axis[2], 0, -axis[0]],
                   [-axis[1], axis[0], 0]])
    return (np.eye(3)
            + np.sin(scaled_angle) * K
            + (1.0 - np.cos(scaled_angle)) * (K @ K))


class TouchHapticNode(Node):
    """Touch 力反馈笔 ROS2 节点

    完整移植 airbot_force_sim.py 的映射逻辑:
      - 位置跟随 (pen_pos_to_robot)
      - 姿态跟随 (transform → 相对旋转 → R_map 映射)
      - 夹爪控制 (Button1 上升沿切换)
      - 位置/姿态重置 (Button2)
      - 力反馈 (robot_force_to_pen)
    """

    def __init__(self):
        super().__init__('touch_haptic_node')

        # ---- 参数 ----
        self.declare_parameter('pos_scale', DEFAULT_POS_SCALE)
        self.declare_parameter('rot_scale', DEFAULT_ROT_SCALE)
        self.declare_parameter('force_scale', DEFAULT_FORCE_SCALE)
        self.declare_parameter('max_pen_force', DEFAULT_MAX_PEN_FORCE)
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('enable_force_feedback', True)

        self.pos_scale = self.get_parameter('pos_scale').value
        self.rot_scale = self.get_parameter('rot_scale').value
        self.force_scale = self.get_parameter('force_scale').value
        self.max_pen_force = self.get_parameter('max_pen_force').value
        control_rate = self.get_parameter('control_rate').value
        self.enable_force_feedback = self.get_parameter('enable_force_feedback').value

        # ---- QoS ----
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)

        # ---- 发布者 ----
        self.pub_target_pose = self.create_publisher(
            PoseStamped, '/arm/target_pose', qos)
        self.pub_gripper = self.create_publisher(
            Float64, '/arm/gripper_cmd', qos)

        # ---- 订阅者 ----
        self.sub_ee_force = self.create_subscription(
            WrenchStamped, '/mujoco/end_effector_force',
            self._ee_force_callback, qos)
        self.sub_ee_pose = self.create_subscription(
            PoseStamped, '/mujoco/end_effector_pose',
            self._ee_pose_callback, qos)

        # ---- 位置跟随状态 (与 airbot_force_sim.py 完全一致) ----
        self.pen_origin = None              # 笔位置零点 (mm)
        self.pen_origin_initialized = False
        self.workspace_center = None        # 机器人末端 home 位置 (m, 世界系)
        self.workspace_initialized = False

        # ---- 姿态跟随状态 (与 airbot_force_sim.py 完全一致) ----
        self.pen_R0 = None                  # 笔初始旋转矩阵 (3x3)
        self.robot_R0 = None                # 机器人初始末端旋转矩阵 (3x3)

        # ---- 夹爪状态 ----
        self.gripper_target = 0.0366        # 默认全开
        self._btn1_last = False             # 灰色按钮上一帧
        self._btn2_last = False             # 白色按钮上一帧

        # ---- 力反馈 ----
        self._contact_force = np.zeros(3)
        self._contact_force_lock = threading.Lock()
        self.force_filter = LowPassFilter(alpha=DEFAULT_FORCE_FILTER_ALPHA, dim=3)

        # ---- 末端实际位姿 (用于重置时获取当前位置) ----
        self._ee_pos = None
        self._ee_quat_mj = None  # MuJoCo 格式 [w,x,y,z]

        # ---- 加载 Touch 驱动 ----
        self.haptic = None
        try:
            driver_mod = _load_haptic_driver()
            self.haptic = driver_mod.HapticDevice()
            self.haptic.initialize()
            self.haptic.start_scheduler()
            info = self.haptic.info
            self.get_logger().info(f'Touch 笔已连接: {info.model}')
            self.get_logger().info(f'  最大力: {info.max_force:.1f}N, 更新率: {info.update_rate}Hz')
        except Exception as e:
            self.get_logger().error(f'Touch 笔初始化失败: {e}')
            self.get_logger().error('节点继续运行但无输入')

        # ---- 控制回路 ----
        self._timer = self.create_timer(1.0 / control_rate, self._control_loop)
        self._frame = 0

        self.get_logger().info('Touch 力反馈笔节点已启动')
        self.get_logger().info(f'  pos_scale={self.pos_scale}, rot_scale={self.rot_scale}')
        self.get_logger().info(f'  force_scale={self.force_scale}, force_fb={self.enable_force_feedback}')

    # ---- ROS2 回调 ----
    def _ee_force_callback(self, msg: WrenchStamped):
        with self._contact_force_lock:
            self._contact_force = np.array([
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z])

    def _ee_pose_callback(self, msg: PoseStamped):
        """末端实际位姿回调 — 首帧用于初始化 workspace_center + robot_R0"""
        self._ee_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z])
        # ROS (x,y,z,w) → MuJoCo (w,x,y,z)
        self._ee_quat_mj = np.array([
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z])

        if not self.workspace_initialized:
            self.workspace_center = self._ee_pos.copy()
            # 初始化机器人初始旋转矩阵
            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, self._ee_quat_mj)
            self.robot_R0 = R.reshape(3, 3).copy()
            self.workspace_initialized = True
            self.get_logger().info(
                f'workspace_center: [{self.workspace_center[0]:.3f}, '
                f'{self.workspace_center[1]:.3f}, {self.workspace_center[2]:.3f}]')

    # ---- 坐标映射 (从 airbot_force_sim.py 原封不动移植) ----
    def _pen_pos_to_robot(self, pen_relative_mm):
        """Touch 笔相对位移 (mm) → 机器人末端目标位置 (m, 世界系)

        映射关系:
          Touch X (用户右)    → Robot -Y (取反)
          Touch Y (上)       → Robot  Z
          Touch Z (朝向用户)  → Robot -X (取反)
        """
        return np.array([
            self.workspace_center[0] - pen_relative_mm[2] * self.pos_scale,
            self.workspace_center[1] - pen_relative_mm[0] * self.pos_scale,
            self.workspace_center[2] + pen_relative_mm[1] * self.pos_scale,
        ])

    def _robot_force_to_pen(self, robot_force):
        """机器人接触力 (N) → Touch 笔力输出 (N), 虚功转置"""
        pen_force = np.array([
            self.force_scale * (-robot_force[1]),
            self.force_scale * robot_force[2],
            self.force_scale * (-robot_force[0]),
        ])
        mag = np.linalg.norm(pen_force)
        if mag > self.max_pen_force:
            pen_force *= self.max_pen_force / mag
        return pen_force

    def _extract_pen_rotation(self, state):
        """从 Touch 笔 state 中提取 3x3 旋转矩阵

        HD API 的 transform 是 4x4 列主序矩阵,
        在 Python 中以扁平 16-double 存储。
        """
        tf = getattr(state, 'transform', None)
        if tf is None:
            return None
        tf = np.array(tf)
        if tf.size != 16:
            return None
        # HD API 返回列主序 4x4 → reshape 后需要转置
        T_raw = tf.reshape(4, 4)       # = M^T (行主序存储的列主序矩阵)
        R = T_raw.T[:3, :3]            # = M[:3,:3] 真实旋转矩阵
        # 验证有效性
        det = np.linalg.det(R)
        if abs(det - 1.0) > 0.15:
            return None
        return R

    # ---- 主控制回路 ----
    def _control_loop(self):
        if self.haptic is None:
            return

        state = self.haptic.get_state()
        if state is None or not hasattr(state, 'position') or state.position is None:
            return

        pen_pos = np.array(state.position)  # mm

        # ---- 首次: 记录笔位置零点 ----
        if not self.pen_origin_initialized:
            self.pen_origin = pen_pos.copy()
            self.pen_origin_initialized = True
            self.get_logger().info(
                f'笔位置零点: [{pen_pos[0]:.1f}, {pen_pos[1]:.1f}, {pen_pos[2]:.1f}] mm')

        # 等 workspace_center 初始化
        if not self.workspace_initialized:
            return

        # ---- 姿态标定: 首次有效 transform → pen_R0 ----
        if self.pen_R0 is None:
            R = self._extract_pen_rotation(state)
            if R is not None:
                self.pen_R0 = R.copy()
                self.get_logger().info(f'姿态标定完成 (det={np.linalg.det(R):.4f})')

        # ---- 位置跟随 ----
        relative_mm = pen_pos - self.pen_origin
        robot_target_pos = self._pen_pos_to_robot(relative_mm)

        # ---- 姿态跟随 (完整移植自 airbot_force_sim.py) ----
        robot_target_quat_ros = [0.0, 0.0, 0.0, 1.0]  # 默认单位四元数
        if self.pen_R0 is not None and self.robot_R0 is not None:
            R_pen_cur = self._extract_pen_rotation(state)
            if R_pen_cur is not None:
                # 笔相对初始姿态的旋转
                delta_R_pen = R_pen_cur @ self.pen_R0.T
                # 姿态放大
                delta_R_pen = _scale_rotation(delta_R_pen, self.rot_scale)
                # 映射到机器人坐标系
                delta_R_robot = R_MAP @ delta_R_pen @ R_MAP.T
                # 叠加到机器人初始姿态
                R_robot_des = delta_R_robot @ self.robot_R0
                # SVD 正交化
                U, _, Vt = np.linalg.svd(R_robot_des)
                R_robot_des = U @ Vt
                # → MuJoCo 四元数
                quat_mj = np.zeros(4)
                mujoco.mju_mat2Quat(quat_mj, R_robot_des.flatten())
                # MuJoCo (w,x,y,z) → ROS (x,y,z,w)
                robot_target_quat_ros = [
                    float(quat_mj[1]), float(quat_mj[2]),
                    float(quat_mj[3]), float(quat_mj[0])]

        # ---- 发布 /arm/target_pose ----
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = float(robot_target_pos[0])
        msg.pose.position.y = float(robot_target_pos[1])
        msg.pose.position.z = float(robot_target_pos[2])
        msg.pose.orientation.x = robot_target_quat_ros[0]
        msg.pose.orientation.y = robot_target_quat_ros[1]
        msg.pose.orientation.z = robot_target_quat_ros[2]
        msg.pose.orientation.w = robot_target_quat_ros[3]
        self.pub_target_pose.publish(msg)

        # ---- 夹爪: Button1 上升沿切换 ----
        btn1 = getattr(state, 'button1', False)
        btn2 = getattr(state, 'button2', False)

        if btn1 and not self._btn1_last:
            self.gripper_target = 0.0 if self.gripper_target > 0.018 else 0.0366
            self.get_logger().info(f'夹爪: {"闭合" if self.gripper_target < 0.01 else "张开"}')
        self._btn1_last = btn1

        # ---- Button2: 重置位置+姿态 (与 airbot_force_sim.py 一致) ----
        if btn2 and not self._btn2_last:
            # 位置零点 = 当前笔位置
            self.pen_origin = pen_pos.copy()
            # workspace_center = 当前机器人末端实际位置
            if self._ee_pos is not None:
                self.workspace_center = self._ee_pos.copy()
            # 姿态标定更新: 当前笔姿态 = 当前机器人姿态
            R = self._extract_pen_rotation(state)
            if R is not None:
                self.pen_R0 = R.copy()
            if self._ee_quat_mj is not None:
                R_robot = np.zeros(9)
                mujoco.mju_quat2Mat(R_robot, self._ee_quat_mj)
                self.robot_R0 = R_robot.reshape(3, 3).copy()
            self.get_logger().info(
                f'★ 重置! 笔零点=[{pen_pos[0]:.0f},{pen_pos[1]:.0f},{pen_pos[2]:.0f}]mm '
                f'workspace=[{self.workspace_center[0]:.3f},'
                f'{self.workspace_center[1]:.3f},{self.workspace_center[2]:.3f}]')
        self._btn2_last = btn2

        # 发布夹爪
        grip_msg = Float64()
        grip_msg.data = float(self.gripper_target)
        self.pub_gripper.publish(grip_msg)

        # ---- 力反馈 (Layer 4) ----
        if self.enable_force_feedback:
            with self._contact_force_lock:
                raw_force = self._contact_force.copy()
            smoothed = self.force_filter.filter(raw_force)
            pen_force = self._robot_force_to_pen(smoothed)
            try:
                self.haptic.set_force(pen_force[0], pen_force[1], pen_force[2])
            except Exception:
                pass

        # ---- 日志 (每 5 秒) ----
        self._frame += 1
        if self._frame % 500 == 0:
            force_mag = np.linalg.norm(self._contact_force)
            self.get_logger().info(
                f'[Touch] pen=[{pen_pos[0]:+.0f},{pen_pos[1]:+.0f},{pen_pos[2]:+.0f}]mm '
                f'target=[{robot_target_pos[0]:.3f},{robot_target_pos[1]:.3f},'
                f'{robot_target_pos[2]:.3f}] force={force_mag:.2f}N')

    def destroy_node(self):
        if self.haptic:
            try:
                self.haptic.set_force(0, 0, 0)
                self.haptic.stop()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TouchHapticNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
