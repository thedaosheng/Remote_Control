#!/usr/bin/env python3
"""
MuJoCo 四舵轮仿真 — ROS2 消费端节点

职责: 纯仿真消费端。订阅 ROS2 话题 → IK 解算 → 驱动 MuJoCo 物理 → 发布状态

数据流:
  控制层 (键盘/鼠标/VP/导航)
    ↓ /cmd_vel (Twist)           底盘速度指令 (车身系)
    ↓ /mujoco/lift_cmd (Float64)  升降目标
    ↓ /mujoco/head_cmd (Vector3)  云台指令
  本节点 (IK + 仿真)
    ↓ /mujoco/swerve_cmd (Float64MultiArray)  8个电机指令 → MC02
    ↓ /mujoco/joint_states (JointState)       全部关节状态
    ↓ /mujoco/odom (Odometry)                 里程计
    ↓ /tf (odom→base_link)

运行:
  ros2 run teleop_mujoco_sim mujoco_sim_node
  ros2 run teleop_mujoco_sim mujoco_sim_node --ros-args -p enable_touch:=false
"""

import os
import sys
import math
import time
import threading
import importlib.util

import numpy as np
import mujoco
import mujoco.viewer

# ---- ROS2 ----
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, Vector3Stamped, PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray
import tf2_ros


# ============== 路径 ==============
# MJCF 模型路径: 优先从 RemoteControl 加载，回退到 scripts
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_CANDIDATES = [
    os.path.join(os.path.expanduser('~'), 'teleop', 'RemoteControl',
                 'swerve_chassis', '20260408-cc-swerve_chassis.xml'),
    os.path.join(os.path.expanduser('~'), 'teleop', 'scripts',
                 '20260408-cc-swerve_chassis.xml'),
]
MJCF_PATH = next((p for p in _CANDIDATES if os.path.exists(p)), _CANDIDATES[-1])

# Touch 笔驱动路径
TOUCH_DRIVER_PATH = os.path.join(
    os.path.expanduser('~'), 'teleop', 'force-feedback-pen',
    '20260328-cc-haptic_driver.py')

# Touch 笔 libncurses 兼容库
_LIB_COMPAT = os.path.join(os.path.expanduser('~'), 'teleop', 'lib')
if os.path.isdir(_LIB_COMPAT) and _LIB_COMPAT not in os.environ.get('LD_LIBRARY_PATH', ''):
    os.environ['LD_LIBRARY_PATH'] = _LIB_COMPAT + ':' + os.environ.get('LD_LIBRARY_PATH', '')
    # 必须 re-exec 使 LD_LIBRARY_PATH 对 dlopen 生效
    os.execv(sys.executable, [sys.executable] + sys.argv)

# ============== 舵轮参数 ==============
WHEEL_RADIUS = 0.06
WHEEL_POSITIONS = np.array([
    [0.16, 0.15], [0.16, -0.15],    # FL, FR
    [-0.16, 0.15], [-0.16, -0.15],  # RL, RR
])
MAX_VX, MAX_VY, MAX_OMEGA = 1.0, 1.0, 2.0
MAX_WHEEL_SPEED = MAX_VX / WHEEL_RADIUS * 1.5
STEER_RATE_LIMIT = 8.0
DRIVE_ACCEL_LIMIT = 30.0

# 升降 / 云台参数
LIFT_MIN, LIFT_MAX, LIFT_INIT, LIFT_RATE = 0.0, 0.8, 0.0, 0.5
HEAD_YAW_MIN, HEAD_YAW_MAX = -1.5708, 1.5708
HEAD_PITCH_MIN, HEAD_PITCH_MAX = -0.7854, 0.7854
HEAD_RATE = 2.5
HEAD_STEM_MIN, HEAD_STEM_MAX, HEAD_STEM_RATE = -0.05, 0.40, 0.30

# 双臂 home
ARM_HOME_LEFT = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ARM_HOME_RIGHT = [0.0, -0.5, 0.3, 1.0, 0.0, 0.0]

# 夹爪
GRIPPER_OPEN, GRIPPER_CLOSE = 0.0366, 0.0

# ============== 执行器索引 (与 XML 一致) ==============
CTRL_VX, CTRL_VY, CTRL_OMEGA = 0, 1, 2
CTRL_STEER = 3      # 3,4,5,6
CTRL_DRIVE = 7      # 7,8,9,10
CTRL_LIFT = 11
CTRL_LEFT_ARM = 12  # 12~17
CTRL_RIGHT_ARM = 18 # 18~23
CTRL_HEAD_YAW = 24
CTRL_HEAD_PITCH = 25
CTRL_HEAD_STEM = 26
CTRL_GRIPPER = 27

# ============== IK 矩阵 ==============
_IK_MATRIX = np.zeros((8, 3))
for _i, (_mx, _my) in enumerate(WHEEL_POSITIONS):
    _IK_MATRIX[2 * _i, :] = [1.0, 0.0, -_my]
    _IK_MATRIX[2 * _i + 1, :] = [0.0, 1.0, _mx]


def _normalize_angle(a):
    """角度归一化到 [-pi, pi]"""
    a = a % (2 * math.pi)
    if a > math.pi:
        a -= 2 * math.pi
    return a


def swerve_ik(vx, vy, omega, prev_steer):
    """底盘速度 (车身系) → 4 组 (转向角, 驱动转速)"""
    if abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(omega) < 1e-6:
        return prev_steer.copy(), np.zeros(4)
    vecs = _IK_MATRIX @ np.array([vx, vy, omega])
    steer, drive = np.zeros(4), np.zeros(4)
    for i in range(4):
        wx, wy = vecs[2*i], vecs[2*i+1]
        speed = math.hypot(wx, wy)
        if speed < 1e-4:
            steer[i] = prev_steer[i]
        else:
            steer[i] = math.atan2(wy, wx)
            drive[i] = speed / WHEEL_RADIUS
    return steer, drive


def optimize_module(speed, angle, prev_angle):
    """>90° 翻转优化: 避免舵轮大角度旋转"""
    delta = _normalize_angle(angle - prev_angle)
    if abs(delta) > math.pi / 2:
        angle = _normalize_angle(angle + math.pi)
        speed = -speed
    return speed, angle


def desaturate(speeds, max_speed):
    """轮速归一化: 等比缩放防止任一轮超限"""
    peak = np.max(np.abs(speeds))
    return speeds * (max_speed / peak) if peak > max_speed else speeds


def rate_limit(target, current, limit, dt):
    """速率限幅: 限制每步最大变化量"""
    delta = np.clip(target - current, -limit * dt, limit * dt)
    return current + delta


# ============== 右臂阻抗控制器 ==============
# 从 airbot_force_sim.py 移植的任务空间阻抗控制算法
# 核心原理: 在笛卡尔空间计算力/力矩 → 雅可比转置映射到关节力矩
# 这比关节空间 PD 控制更适合力反馈场景（力可直接在任务空间解释）

class ImpedanceController:
    """任务空间阻抗控制器

    算法:
      1. pos_err = desired_pos - current_pos
      2. F = Kp * pos_err - Kd * vel       (笛卡尔空间弹簧-阻尼器)
      3. rot_err = quat_diff(desired, current)
      4. T = Kp_rot * rot_err - Kd_rot * omega
      5. tau = Jp.T @ F + Jr.T @ T + gravity_compensation
      6. tau -= null_damping * qvel         (零空间阻尼防漂移)
    """

    def __init__(self,
                 kp_pos=5000.0,    # 位置刚度 (N/m) — 越大跟踪越紧
                 kd_pos=150.0,     # 位置阻尼 (N·s/m) — 越大运动越平滑
                 kp_rot=300.0,     # 姿态刚度 (N·m/rad)
                 kd_rot=30.0,      # 姿态阻尼 (N·m·s/rad)
                 null_damping=5.0  # 零空间阻尼 — 防止冗余自由度漂移
                 ):
        """初始化阻抗控制器增益

        参数说明:
          kp_pos: 位置 P 增益。5000 N/m 意味着 1mm 误差产生 5N 力
          kd_pos: 位置 D 增益。150 N·s/m 在 0.1m/s 时产生 15N 阻尼力
          kp_rot: 姿态 P 增益。300 N·m/rad ≈ 0.1rad 误差产生 30 N·m 力矩
          kd_rot: 姿态 D 增益。
          null_damping: 零空间阻尼系数，防止不影响末端的关节运动
        """
        self.Kp_pos = np.diag([kp_pos] * 3)
        self.Kd_pos = np.diag([kd_pos] * 3)
        self.Kp_rot = np.diag([kp_rot] * 3)
        self.Kd_rot = np.diag([kd_rot] * 3)
        self.null_damping = null_damping

    def compute(self, model, data, site_id, n_arm, qpos_offset, qvel_offset,
                desired_pos, desired_quat):
        """计算关节力矩

        参数:
          model, data:     MuJoCo 模型和数据
          site_id:         末端 site 的 MuJoCo ID
          n_arm:           臂关节数 (6)
          qpos_offset:     右臂 qpos 在全局数组中的起始索引
          qvel_offset:     右臂 qvel 在全局数组中的起始索引
          desired_pos:     目标末端位置 (3,) 世界系
          desired_quat:    目标末端四元数 (4,) MuJoCo 格式 [w,x,y,z]

        返回:
          tau: (n_arm,) 关节力矩向量
        """
        nv = model.nv

        # ---- 当前末端状态 ----
        pos_cur = data.site_xpos[site_id].copy()
        xmat_cur = data.site_xmat[site_id].reshape(3, 3)

        # ---- 雅可比矩阵 (3 x nv) ----
        jacp_full = np.zeros((3, nv))
        jacr_full = np.zeros((3, nv))
        mujoco.mj_jacSite(model, data, jacp_full, jacr_full, site_id)

        # 只取右臂对应的 6 列
        Jp = jacp_full[:, qvel_offset:qvel_offset + n_arm]
        Jr = jacr_full[:, qvel_offset:qvel_offset + n_arm]

        # ---- 位置误差力 ----
        pos_err = desired_pos - pos_cur
        qvel_arm = data.qvel[qvel_offset:qvel_offset + n_arm]
        vel_cur = Jp @ qvel_arm
        F_pos = self.Kp_pos @ pos_err - self.Kd_pos @ vel_cur

        # ---- 姿态误差力矩 ----
        quat_cur = np.zeros(4)
        mujoco.mju_mat2Quat(quat_cur, xmat_cur.flatten())
        rot_err = np.zeros(3)
        if desired_quat is not None:
            mujoco.mju_subQuat(rot_err, desired_quat, quat_cur)
        omega_cur = Jr @ qvel_arm
        T_rot = self.Kp_rot @ rot_err - self.Kd_rot @ omega_cur

        # ---- 关节力矩 = J^T @ F + 重力补偿 ----
        tau = Jp.T @ F_pos + Jr.T @ T_rot
        tau += data.qfrc_bias[qvel_offset:qvel_offset + n_arm]

        # ---- 零空间阻尼 ----
        if self.null_damping > 0:
            tau -= self.null_damping * qvel_arm

        return tau


class LowPassFilter:
    """一阶指数移动平均低通滤波器

    用于平滑接触力信号，滤除单帧碰撞噪声。
    alpha 越大 → 响应越快但越noisy; 越小 → 越平滑但有延迟。
    """

    def __init__(self, alpha=0.3, dim=3):
        self.alpha = alpha
        self.value = np.zeros(dim)
        self._initialized = False

    def filter(self, raw):
        """输入原始值，返回滤波后的值"""
        if not self._initialized:
            self.value = raw.copy()
            self._initialized = True
        else:
            self.value = self.alpha * raw + (1 - self.alpha) * self.value
        return self.value.copy()


# ============== 关节名称列表 (用于 JointState 消息) ==============
JOINT_NAMES = (
    ['chassis_x', 'chassis_y', 'chassis_yaw'] +
    ['fl_steer', 'fr_steer', 'rl_steer', 'rr_steer'] +
    ['fl_drive', 'fr_drive', 'rl_drive', 'rr_drive'] +
    ['lift_joint'] +
    [f'left_joint{i}' for i in range(1, 7)] +
    [f'right_joint{i}' for i in range(1, 7)] +
    ['head_yaw_joint', 'head_pitch_joint', 'head_stem'] +
    ['right_g2_left_joint', 'right_g2_right_joint']
)


class MujocoSimNode(Node):
    """MuJoCo 仿真 ROS2 消费端节点

    纯消费端: 不包含任何输入设备代码 (键盘/鼠标/摇杆)。
    所有控制指令通过 ROS2 话题订阅获取。
    """

    def __init__(self):
        super().__init__('mujoco_sim_node')

        # ---- 参数 ----
        self.declare_parameter('enable_touch', True)
        self.declare_parameter('publish_rate', 50.0)  # Hz, 话题发布频率

        self.enable_touch = self.get_parameter('enable_touch').value
        pub_rate = self.get_parameter('publish_rate').value

        # ---- 加载 MuJoCo 模型 ----
        self.get_logger().info(f'加载 MJCF: {MJCF_PATH}')
        self.model = mujoco.MjModel.from_xml_path(MJCF_PATH)
        self.data = mujoco.MjData(self.model)
        self.get_logger().info(f'模型: nq={self.model.nq}, nv={self.model.nv}, nu={self.model.nu}')

        # ---- 关节索引缓存 ----
        self.joint_qpos = {}  # name → qpos index
        self.joint_qvel = {}  # name → qvel index
        for name in JOINT_NAMES:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                self.joint_qpos[name] = self.model.jnt_qposadr[jid]
                self.joint_qvel[name] = self.model.jnt_dofadr[jid]

        # ---- 底盘控制状态 ----
        # target_vx/vy/omega 始终是车身系 (X=车头前方, Y=车身左侧)
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_omega = 0.0
        self.target_lift = LIFT_INIT
        self.smoothed_lift = LIFT_INIT
        self.target_head_yaw = 0.0
        self.target_head_pitch = 0.0
        self.target_head_stem = 0.0
        self.current_steer = np.zeros(4)
        self.current_drive = np.zeros(4)

        # /cmd_vel 超时检测: 超过此时间无输入 → 速度归零 (安全保护)
        self.last_cmd_vel_time = 0.0
        self.CMD_VEL_TIMEOUT = 0.5  # 0.5s

        # ---- ROS2 发布者 ----
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)

        # /mujoco/swerve_cmd: 舵轮 IK 解算后的 8 个电机指令
        #   data[0:4] = 4 个舵向角 (rad)  — FL, FR, RL, RR
        #   data[4:8] = 4 个驱动轮速 (rad/s) — FL, FR, RL, RR
        #   消费端: MC02 达妙控制板 / 其他电机驱动节点
        self.pub_swerve_cmd = self.create_publisher(
            Float64MultiArray, '/mujoco/swerve_cmd', qos)
        # /mujoco/joint_states: 全部关节状态 (仿真反馈)
        self.pub_joint_states = self.create_publisher(
            JointState, '/mujoco/joint_states', qos)
        # /mujoco/odom: 底盘里程计
        self.pub_odom = self.create_publisher(Odometry, '/mujoco/odom', qos)
        # TF: odom → base_link
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ---- ROS2 订阅者 ----
        # /cmd_vel: 底盘速度指令 (车身系 Twist)
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, qos)
        # /mujoco/lift_cmd: 升降目标位置 (m)
        self.sub_lift = self.create_subscription(
            Float64, '/mujoco/lift_cmd', self._lift_callback, qos)
        # /mujoco/head_cmd: 云台指令 (x=yaw, y=pitch, z=stem)
        self.sub_head = self.create_subscription(
            Vector3Stamped, '/mujoco/head_cmd', self._head_callback, qos)

        # ---- 右臂力控话题 (Layer 1-4) ----
        # /arm/target_pose: 右臂末端目标位姿 (世界系)
        #   控制层发布: Touch 笔节点 / VP 节点 / 运动规划器
        self.sub_arm_pose = self.create_subscription(
            PoseStamped, '/arm/target_pose', self._arm_pose_callback, qos)
        # /arm/gripper_cmd: 夹爪指令 (0=闭合, 0.0366=全开)
        self.sub_gripper = self.create_subscription(
            Float64, '/arm/gripper_cmd', self._gripper_callback, qos)
        # /mujoco/end_effector_force: 末端接触力 (世界系, Layer 3)
        #   消费端: Touch 笔力反馈节点
        self.pub_ee_force = self.create_publisher(
            WrenchStamped, '/mujoco/end_effector_force', qos)
        # /mujoco/end_effector_pose: 当前末端实际位姿 (世界系, 反馈)
        self.pub_ee_pose = self.create_publisher(
            PoseStamped, '/mujoco/end_effector_pose', qos)

        # ---- 发布定时器 ----
        self.pub_timer = self.create_timer(1.0 / pub_rate, self._publish_state)

        # ---- 右臂阻抗控制 (Layer 1-4) ----
        self.impedance = ImpedanceController(
            kp_pos=5000.0, kd_pos=150.0,
            kp_rot=300.0, kd_rot=30.0,
            null_damping=5.0)
        self.force_filter = LowPassFilter(alpha=0.3, dim=3)

        # 右臂 site / body ID (在 run() 中查找)
        self.right_eef_site_id = -1
        self.right_contact_body_ids = []
        # 右臂关节在 qpos/qvel 中的偏移 (在 run() 中计算)
        self.right_arm_qpos_offset = 0
        self.right_arm_qvel_offset = 0
        self.right_arm_n = 6

        # 右臂控制目标 (None = 保持 home, 收到话题后激活阻抗控制)
        self.arm_desired_pos = None
        self.arm_desired_quat = None
        self.arm_gripper_target = GRIPPER_CLOSE
        self.arm_pose_active = False       # 是否收到过 /arm/target_pose
        self.last_arm_pose_time = 0.0
        self.ARM_POSE_TIMEOUT = 1.0        # 1s 超时回 home

        # HUD 帧计数器
        self._hud_frame = 0

        self.get_logger().info('MuJoCo 仿真消费端已初始化 (无内置键盘, 需外部控制节点)')
        self.get_logger().info('话题:')
        self.get_logger().info('  发布: /mujoco/swerve_cmd, /mujoco/joint_states, /mujoco/odom, /tf')
        self.get_logger().info('         /mujoco/end_effector_force, /mujoco/end_effector_pose')
        self.get_logger().info('  订阅: /cmd_vel, /mujoco/lift_cmd, /mujoco/head_cmd')
        self.get_logger().info('         /arm/target_pose, /arm/gripper_cmd')
        self.get_logger().info('控制节点:')
        self.get_logger().info('  底盘: ros2 run teleop_mujoco_sim keyboard_teleop_node')
        self.get_logger().info('  右臂: ros2 run teleop_mujoco_sim touch_haptic_node')

    # ---- ROS2 回调 ----
    def _cmd_vel_callback(self, msg: Twist):
        """/cmd_vel 回调: 接收车身系速度指令, clamp 到物理极限"""
        self.target_vx = float(np.clip(msg.linear.x, -MAX_VX, MAX_VX))
        self.target_vy = float(np.clip(msg.linear.y, -MAX_VY, MAX_VY))
        self.target_omega = float(np.clip(msg.angular.z, -MAX_OMEGA, MAX_OMEGA))
        self.last_cmd_vel_time = time.time()

    def _lift_callback(self, msg: Float64):
        """升降指令回调"""
        self.target_lift = float(np.clip(msg.data, LIFT_MIN, LIFT_MAX))

    def _head_callback(self, msg: Vector3Stamped):
        """云台指令回调"""
        self.target_head_yaw = float(np.clip(msg.vector.x, HEAD_YAW_MIN, HEAD_YAW_MAX))
        self.target_head_pitch = float(np.clip(msg.vector.y, HEAD_PITCH_MIN, HEAD_PITCH_MAX))
        self.target_head_stem = float(np.clip(msg.vector.z, HEAD_STEM_MIN, HEAD_STEM_MAX))

    def _arm_pose_callback(self, msg: PoseStamped):
        """右臂末端目标位姿回调 (Layer 1-2)

        接收来自 Touch 笔 / VP / 运动规划器的末端期望位姿。
        坐标系: 世界系 (与 MuJoCo 一致)
        """
        self.arm_desired_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z])
        # ROS 四元数 (x,y,z,w) → MuJoCo 四元数 (w,x,y,z)
        self.arm_desired_quat = np.array([
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z])
        self.arm_pose_active = True
        self.last_arm_pose_time = time.time()

    def _gripper_callback(self, msg: Float64):
        """夹爪指令回调 (0=闭合, 0.0366=全开)"""
        self.arm_gripper_target = float(np.clip(msg.data, GRIPPER_CLOSE, GRIPPER_OPEN))

    # ---- 发布状态 ----
    def _publish_state(self):
        """定时发布关节状态 + 里程计 + TF"""
        now = self.get_clock().now().to_msg()

        # --- JointState ---
        js = JointState()
        js.header.stamp = now
        js.header.frame_id = 'base_link'
        for name in JOINT_NAMES:
            if name in self.joint_qpos:
                js.name.append(name)
                js.position.append(float(self.data.qpos[self.joint_qpos[name]]))
                js.velocity.append(float(self.data.qvel[self.joint_qvel[name]]))
        self.pub_joint_states.publish(js)

        # --- Odometry ---
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        cx = self.data.qpos[self.joint_qpos.get('chassis_x', 0)]
        cy = self.data.qpos[self.joint_qpos.get('chassis_y', 0)]
        cyaw = self.data.qpos[self.joint_qpos.get('chassis_yaw', 0)]

        odom.pose.pose.position.x = float(cx)
        odom.pose.pose.position.y = float(cy)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = float(math.sin(cyaw / 2))
        odom.pose.pose.orientation.w = float(math.cos(cyaw / 2))

        # 世界速度 → body frame
        vx_w = self.data.qvel[self.joint_qvel.get('chassis_x', 0)]
        vy_w = self.data.qvel[self.joint_qvel.get('chassis_y', 0)]
        wz = self.data.qvel[self.joint_qvel.get('chassis_yaw', 0)]
        c, s = math.cos(cyaw), math.sin(cyaw)
        odom.twist.twist.linear.x = float(c * vx_w + s * vy_w)
        odom.twist.twist.linear.y = float(-s * vx_w + c * vy_w)
        odom.twist.twist.angular.z = float(wz)
        self.pub_odom.publish(odom)

        # --- TF: odom → base_link ---
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(cx)
        t.transform.translation.y = float(cy)
        t.transform.translation.z = 0.16
        t.transform.rotation.z = float(math.sin(cyaw / 2))
        t.transform.rotation.w = float(math.cos(cyaw / 2))
        self.tf_broadcaster.sendTransform(t)

        # --- 末端接触力 (Layer 3) ---
        if self.right_eef_site_id >= 0 and len(self.right_contact_body_ids) > 0:
            # 从 cfrc_ext 提取接触力
            raw_force = np.zeros(3)
            for bid in self.right_contact_body_ids:
                # cfrc_ext[bid] = [tx, ty, tz, fx, fy, fz]
                raw_force += self.data.cfrc_ext[bid, 3:6]
            # 低通滤波
            smoothed = self.force_filter.filter(raw_force)

            wrench = WrenchStamped()
            wrench.header.stamp = now
            wrench.header.frame_id = 'world'
            wrench.wrench.force.x = float(smoothed[0])
            wrench.wrench.force.y = float(smoothed[1])
            wrench.wrench.force.z = float(smoothed[2])
            self.pub_ee_force.publish(wrench)

        # --- 当前末端实际位姿 (反馈) ---
        if self.right_eef_site_id >= 0:
            ee_pos = self.data.site_xpos[self.right_eef_site_id]
            ee_mat = self.data.site_xmat[self.right_eef_site_id].reshape(3, 3)
            ee_quat = np.zeros(4)
            mujoco.mju_mat2Quat(ee_quat, ee_mat.flatten())

            ee_msg = PoseStamped()
            ee_msg.header.stamp = now
            ee_msg.header.frame_id = 'world'
            ee_msg.pose.position.x = float(ee_pos[0])
            ee_msg.pose.position.y = float(ee_pos[1])
            ee_msg.pose.position.z = float(ee_pos[2])
            # MuJoCo (w,x,y,z) → ROS (x,y,z,w)
            ee_msg.pose.orientation.x = float(ee_quat[1])
            ee_msg.pose.orientation.y = float(ee_quat[2])
            ee_msg.pose.orientation.z = float(ee_quat[3])
            ee_msg.pose.orientation.w = float(ee_quat[0])
            self.pub_ee_pose.publish(ee_msg)

    # ---- viewer HUD: 在 3D 场景中每个轮子上方显示电机指令值 ----
    _WHEEL_NAMES = ['FL', 'FR', 'RL', 'RR']

    def _render_swerve_hud(self, viewer, cmd_steer, cmd_drive):
        """在 MuJoCo viewer 的 3D 场景中渲染舵轮电机指令 HUD

        原理: 利用 viewer.user_scn 添加带 label 的微小透明球体 (mjvGeom),
        放在每个轮子正上方。label 文字跟随轮子位置显示在 3D 视图中。

        显示内容 (每个轮子):
          - S: 舵向角 (rad), 正值=逆时针
          - D: 驱动轮速 (rad/s), 正值=前进
          - deg: 舵向角的角度制 (方便直观判断)
        """
        scn = viewer.user_scn
        scn.ngeom = 0  # 清空上一帧

        # 读取底盘世界位姿
        cx = float(self.data.qpos[self.joint_qpos.get('chassis_x', 0)])
        cy = float(self.data.qpos[self.joint_qpos.get('chassis_y', 0)])
        cyaw = float(self.data.qpos[self.joint_qpos.get('chassis_yaw', 0)])
        cos_y, sin_y = math.cos(cyaw), math.sin(cyaw)

        eye3 = np.eye(3).flatten()

        # 4 个轮子各一个标签
        for i, (lx, ly) in enumerate(WHEEL_POSITIONS):
            if scn.ngeom >= scn.maxgeom:
                break
            wx = cx + lx * cos_y - ly * sin_y
            wy = cy + lx * sin_y + ly * cos_y
            wz = 0.35

            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(
                g, mujoco.mjtGeom.mjGEOM_SPHERE,
                np.array([0.001, 0.001, 0.001]),
                np.array([wx, wy, wz]), eye3,
                np.array([1.0, 1.0, 1.0, 0.0]))
            steer_deg = math.degrees(cmd_steer[i])
            g.label = (f'{self._WHEEL_NAMES[i]} '
                       f'S:{cmd_steer[i]:+.2f}({steer_deg:+.0f}\u00b0) '
                       f'D:{cmd_drive[i]:+.1f}')
            scn.ngeom += 1

        # 底盘整体速度标签
        if scn.ngeom < scn.maxgeom:
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(
                g, mujoco.mjtGeom.mjGEOM_SPHERE,
                np.array([0.001, 0.001, 0.001]),
                np.array([cx, cy, 0.75]), eye3,
                np.array([1.0, 1.0, 1.0, 0.0]))
            g.label = (f'Body vx:{self.target_vx:+.2f} '
                       f'vy:{self.target_vy:+.2f} '
                       f'w:{self.target_omega:+.2f}')
            scn.ngeom += 1

    # ---- 仿真主循环 ----
    def run(self):
        """主循环: MuJoCo 仿真 + viewer

        本节点是纯消费端:
          1. 从 /cmd_vel 获取车身系速度指令 (超时→归零)
          2. 舵轮 IK 解算 → 4 组 (舵向角, 轮速)
          3. 车身系→世界系旋转 → 写入 MuJoCo velocity actuator
          4. 物理步进 + 渲染
          5. 发布 /mujoco/swerve_cmd (给 MC02 等真实电机消费端)
        """
        model, data = self.model, self.data
        mujoco.mj_forward(model, data)

        # 初始化 ctrl
        data.ctrl[:] = 0.0
        data.ctrl[CTRL_LIFT] = LIFT_INIT
        data.ctrl[CTRL_LEFT_ARM:CTRL_LEFT_ARM + 6] = ARM_HOME_LEFT
        data.ctrl[CTRL_GRIPPER] = GRIPPER_CLOSE

        # 右臂设非奇异 home pose
        for i in range(1, 7):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'right_joint{i}')
            data.qpos[model.jnt_qposadr[jid]] = ARM_HOME_RIGHT[i - 1]
        mujoco.mj_forward(model, data)

        # Settling: 让物理稳定
        # 注意: 右臂是力矩模式执行器，settling 期间也需要施加力矩保持 home
        self.get_logger().info('物体 settling (1.0s)...')
        for _ in range(500):
            data.ctrl[CTRL_LEFT_ARM:CTRL_LEFT_ARM + 6] = ARM_HOME_LEFT
            data.ctrl[CTRL_GRIPPER] = GRIPPER_CLOSE
            # 右臂: 用重力补偿保持 home（settling 阶段还没初始化 impedance 目标）
            rj1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'right_joint1')
            if rj1_id >= 0:
                qvel_off = model.jnt_dofadr[rj1_id]
                data.ctrl[CTRL_RIGHT_ARM:CTRL_RIGHT_ARM + 6] = \
                    data.qfrc_bias[qvel_off:qvel_off + 6]
            mujoco.mj_step(model, data)
        self.get_logger().info('Settling 完成')

        # ---- 右臂阻抗控制初始化 ----
        # 查找末端 site ID
        self.right_eef_site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, 'right_eef_site')
        if self.right_eef_site_id >= 0:
            self.get_logger().info(f'右臂末端 site: right_eef_site (id={self.right_eef_site_id})')
        else:
            self.get_logger().warn('右臂末端 site "right_eef_site" 未找到!')

        # 查找接触力检测 body (夹爪 + link6)
        for bname in ['right_g2_left', 'right_g2_right', 'right_link6']:
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, bname)
            if bid >= 0:
                self.right_contact_body_ids.append(bid)
        self.get_logger().info(f'接触力检测 body: {len(self.right_contact_body_ids)} 个')

        # 计算右臂关节在 qpos/qvel 中的偏移量
        rj1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'right_joint1')
        if rj1_id >= 0:
            self.right_arm_qpos_offset = model.jnt_qposadr[rj1_id]
            self.right_arm_qvel_offset = model.jnt_dofadr[rj1_id]
            self.get_logger().info(
                f'右臂关节偏移: qpos[{self.right_arm_qpos_offset}], '
                f'qvel[{self.right_arm_qvel_offset}]')

        # 记录 home 位置的末端位姿 (用于超时回退)
        if self.right_eef_site_id >= 0:
            self._arm_home_pos = data.site_xpos[self.right_eef_site_id].copy()
            home_mat = data.site_xmat[self.right_eef_site_id].reshape(3, 3)
            self._arm_home_quat = np.zeros(4)
            mujoco.mju_mat2Quat(self._arm_home_quat, home_mat.flatten())
            self.get_logger().info(
                f'右臂 home 末端位置: [{self._arm_home_pos[0]:.3f}, '
                f'{self._arm_home_pos[1]:.3f}, {self._arm_home_pos[2]:.3f}]')

        init_qpos = data.qpos.copy()
        init_qvel = data.qvel.copy()

        # ROS2 spin 在独立线程 (处理话题回调)
        spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        spin_thread.start()

        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.distance = 3.0
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -25
            viewer.cam.lookat[:] = [0, 0, 0.5]

            while viewer.is_running() and rclpy.ok():
                dt = model.opt.timestep

                # ---- cmd_vel 超时保护: 无输入→速度归零 ----
                if time.time() - self.last_cmd_vel_time > self.CMD_VEL_TIMEOUT:
                    self.target_vx = 0.0
                    self.target_vy = 0.0
                    self.target_omega = 0.0

                # 升降低通滤波
                self.smoothed_lift = 0.05 * self.target_lift + 0.95 * self.smoothed_lift

                # ---- 舵轮 IK: 车身系速度 → 4组(舵向角, 轮速) ----
                raw_steer, raw_drive = swerve_ik(
                    self.target_vx, self.target_vy, self.target_omega,
                    self.current_steer)
                raw_drive = desaturate(raw_drive, MAX_WHEEL_SPEED)

                # 舵向优化 (>90° 翻转) + 速率限幅
                cmd_steer, cmd_drive = np.zeros(4), np.zeros(4)
                for i in range(4):
                    sp, an = optimize_module(raw_drive[i], raw_steer[i],
                                             self.current_steer[i])
                    cmd_steer[i] = rate_limit(an, self.current_steer[i],
                                              STEER_RATE_LIMIT, dt)
                    cmd_drive[i] = rate_limit(sp, self.current_drive[i],
                                              DRIVE_ACCEL_LIMIT, dt)
                self.current_steer[:] = cmd_steer
                self.current_drive[:] = cmd_drive

                # ---- 车身系 → 世界系旋转变换 ----
                # MuJoCo 的 chassis_x/y 是世界系 slide joint
                cyaw = float(data.qpos[self.joint_qpos.get('chassis_yaw', 0)])
                cos_yaw = math.cos(cyaw)
                sin_yaw = math.sin(cyaw)
                # body → world: [vx_w]   [cos  -sin] [vx_b]
                #                [vy_w] = [sin   cos] [vy_b]
                vx_world = self.target_vx * cos_yaw - self.target_vy * sin_yaw
                vy_world = self.target_vx * sin_yaw + self.target_vy * cos_yaw

                # ---- 写入 MuJoCo ctrl ----
                data.ctrl[CTRL_VX] = vx_world
                data.ctrl[CTRL_VY] = vy_world
                data.ctrl[CTRL_OMEGA] = self.target_omega
                data.ctrl[CTRL_STEER:CTRL_STEER + 4] = cmd_steer
                data.ctrl[CTRL_DRIVE:CTRL_DRIVE + 4] = cmd_drive
                data.ctrl[CTRL_LIFT] = self.smoothed_lift
                data.ctrl[CTRL_LEFT_ARM:CTRL_LEFT_ARM + 6] = ARM_HOME_LEFT

                # ---- 右臂阻抗控制 (Layer 1) ----
                # 右臂执行器是 motor (力矩模式)，必须始终输出力矩，
                # 不能写位置值！无目标时也用阻抗控制保持 home 位姿。
                if self.right_eef_site_id >= 0:
                    # 超时保护: 无输入 → 回 home
                    if (self.arm_pose_active and
                            time.time() - self.last_arm_pose_time > self.ARM_POSE_TIMEOUT):
                        self.arm_desired_pos = self._arm_home_pos.copy()
                        self.arm_desired_quat = self._arm_home_quat.copy()

                    # 默认目标 = home 位姿 (首次或超时时)
                    des_pos = (self.arm_desired_pos if self.arm_desired_pos is not None
                               else self._arm_home_pos)
                    des_quat = (self.arm_desired_quat if self.arm_desired_quat is not None
                                else self._arm_home_quat)

                    # 阻抗控制: 计算关节力矩
                    tau = self.impedance.compute(
                        model, data,
                        self.right_eef_site_id,
                        self.right_arm_n,
                        self.right_arm_qvel_offset,
                        self.right_arm_qvel_offset,
                        des_pos, des_quat)
                    # 写入右臂执行器 (力矩模式)
                    data.ctrl[CTRL_RIGHT_ARM:CTRL_RIGHT_ARM + 6] = tau

                # 夹爪
                data.ctrl[CTRL_GRIPPER] = self.arm_gripper_target

                data.ctrl[CTRL_HEAD_YAW] = self.target_head_yaw
                data.ctrl[CTRL_HEAD_PITCH] = self.target_head_pitch
                data.ctrl[CTRL_HEAD_STEM] = self.target_head_stem

                # ---- 发布舵轮 IK 指令 (MC02 等真实电机消费端订阅) ----
                swerve_msg = Float64MultiArray()
                swerve_msg.data = [float(v) for v in cmd_steer] + \
                                  [float(v) for v in cmd_drive]
                self.pub_swerve_cmd.publish(swerve_msg)

                # ---- viewer HUD ----
                self._render_swerve_hud(viewer, cmd_steer, cmd_drive)

                # ---- 终端 HUD: 每 250 帧打印一次 ----
                self._hud_frame += 1
                if self._hud_frame % 250 == 0:
                    self.get_logger().info(
                        f'[Swerve] '
                        f'FL({cmd_steer[0]:+.2f},{cmd_drive[0]:+.1f}) '
                        f'FR({cmd_steer[1]:+.2f},{cmd_drive[1]:+.1f}) '
                        f'RL({cmd_steer[2]:+.2f},{cmd_drive[2]:+.1f}) '
                        f'RR({cmd_steer[3]:+.2f},{cmd_drive[3]:+.1f}) '
                        f'| body({self.target_vx:+.2f},{self.target_vy:+.2f},{self.target_omega:+.2f})')

                # 物理步进
                mujoco.mj_step(model, data)

                # 渲染 (sync 会合并 user_scn 中的 HUD geom)
                viewer.sync()

        self.get_logger().info('仿真结束')


def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
