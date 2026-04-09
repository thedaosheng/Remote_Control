#!/usr/bin/env python3
"""
MuJoCo 四舵轮仿真 — ROS2 桥节点

将 MuJoCo 仿真与 ROS2 话题系统打通：
  - 键盘 WASD 输入 → /cmd_vel 发布
  - 外部 /cmd_vel 订阅 → 底盘控制（外部优先）
  - /mujoco/joint_states 发布全部关节状态
  - /mujoco/odom 发布底盘里程计
  - /tf (odom → base_link)

运行:
  ros2 run teleop_mujoco_sim mujoco_sim_node
  # 或带参数:
  ros2 run teleop_mujoco_sim mujoco_sim_node --ros-args -p enable_touch:=false
"""

import os
import sys
import math
import time
import threading
import subprocess
import atexit
import importlib.util

import numpy as np
import mujoco
import mujoco.viewer

# ---- ROS2 ----
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, Vector3Stamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import tf2_ros

# ---- pynput (键盘监听) ----
try:
    from pynput import keyboard as pynput_keyboard
    HAS_PYNPUT = True
except ImportError:
    HAS_PYNPUT = False
    print("[WARN] pynput 未安装，键盘控制不可用，仅接受 ROS2 话题输入")

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
    [0.16, 0.15], [0.16, -0.15],
    [-0.16, 0.15], [-0.16, -0.15],
])
MAX_VX, MAX_VY, MAX_OMEGA = 1.0, 1.0, 2.0
MAX_WHEEL_SPEED = MAX_VX / WHEEL_RADIUS * 1.5
STEER_RATE_LIMIT = 8.0
DRIVE_ACCEL_LIMIT = 30.0
CHASSIS_VX_SLEW = 3.0
CHASSIS_VY_SLEW = 3.0
CHASSIS_OMEGA_SLEW = 6.0
CHASSIS_SPEED = 0.6
CHASSIS_OMEGA_SPEED = 1.5

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
    """底盘速度 → 4 组 (转向角, 驱动转速)"""
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
    """>90° 翻转优化"""
    delta = _normalize_angle(angle - prev_angle)
    if abs(delta) > math.pi / 2:
        angle = _normalize_angle(angle + math.pi)
        speed = -speed
    return speed, angle


def desaturate(speeds, max_speed):
    """轮速归一化"""
    peak = np.max(np.abs(speeds))
    return speeds * (max_speed / peak) if peak > max_speed else speeds


def rate_limit(target, current, limit, dt):
    """速率限幅"""
    delta = np.clip(target - current, -limit * dt, limit * dt)
    return current + delta


def slew(current, target, slew_rate, dt):
    """斜率限幅"""
    step = slew_rate * dt
    d = target - current
    return current + (math.copysign(step, d) if abs(d) > step else d)


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
    """MuJoCo 仿真 ROS2 桥节点"""

    def __init__(self):
        super().__init__('mujoco_sim_node')

        # ---- 参数 ----
        self.declare_parameter('enable_touch', True)
        self.declare_parameter('enable_keyboard', True)
        self.declare_parameter('publish_rate', 50.0)  # Hz, 话题发布频率

        self.enable_touch = self.get_parameter('enable_touch').value
        self.enable_keyboard = self.get_parameter('enable_keyboard').value
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

        # ---- 底盘状态 ----
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

        # 外部 cmd_vel 输入（优先级高于键盘）
        self.ext_cmd_vel = None
        self.ext_cmd_vel_time = 0.0
        self.EXT_CMD_TIMEOUT = 0.5  # 0.5s 无输入 → 回退键盘

        # ---- ROS2 发布者 ----
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)

        # /mujoco/cmd_vel_out: 当前实际底盘指令（键盘或外部输入经过限幅后）
        self.pub_cmd_vel = self.create_publisher(Twist, '/mujoco/cmd_vel_out', qos)
        # /mujoco/joint_states: 全部关节状态
        self.pub_joint_states = self.create_publisher(JointState, '/mujoco/joint_states', qos)
        # /mujoco/odom: 底盘里程计
        self.pub_odom = self.create_publisher(Odometry, '/mujoco/odom', qos)
        # TF: odom → base_link
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ---- ROS2 订阅者 ----
        # /cmd_vel: 外部底盘速度指令（导航/摇杆/全身运动学）
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, qos)
        # /mujoco/lift_cmd: 外部升降指令
        self.sub_lift = self.create_subscription(
            Float64, '/mujoco/lift_cmd', self._lift_callback, qos)
        # /mujoco/head_cmd: 外部云台指令 (x=yaw, y=pitch, z=stem)
        self.sub_head = self.create_subscription(
            Vector3Stamped, '/mujoco/head_cmd', self._head_callback, qos)

        # ---- 发布定时器 ----
        self.pub_timer = self.create_timer(1.0 / pub_rate, self._publish_state)

        # ---- 键盘状态 ----
        self.key_held = {k: False for k in 'wasaqdeghikjlty'}
        self.reset_request = False
        self.exit_request = False
        self.toggle_fpv = False
        self.key_lock = threading.Lock()

        self.get_logger().info('MuJoCo 仿真 ROS2 桥已初始化')
        self.get_logger().info('话题:')
        self.get_logger().info('  发布: /mujoco/cmd_vel_out, /mujoco/joint_states, /mujoco/odom, /tf')
        self.get_logger().info('  订阅: /cmd_vel, /mujoco/lift_cmd, /mujoco/head_cmd')

    # ---- ROS2 回调 ----
    def _cmd_vel_callback(self, msg: Twist):
        """外部 /cmd_vel 输入（优先于键盘）"""
        self.ext_cmd_vel = msg
        self.ext_cmd_vel_time = time.time()

    def _lift_callback(self, msg: Float64):
        """外部升降指令"""
        self.target_lift = np.clip(msg.data, LIFT_MIN, LIFT_MAX)

    def _head_callback(self, msg: Vector3Stamped):
        """外部云台指令"""
        self.target_head_yaw = np.clip(msg.vector.x, HEAD_YAW_MIN, HEAD_YAW_MAX)
        self.target_head_pitch = np.clip(msg.vector.y, HEAD_PITCH_MIN, HEAD_PITCH_MAX)
        self.target_head_stem = np.clip(msg.vector.z, HEAD_STEM_MIN, HEAD_STEM_MAX)

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

        # 位置: chassis_x, chassis_y
        cx = self.data.qpos[self.joint_qpos.get('chassis_x', 0)]
        cy = self.data.qpos[self.joint_qpos.get('chassis_y', 0)]
        cyaw = self.data.qpos[self.joint_qpos.get('chassis_yaw', 0)]

        odom.pose.pose.position.x = float(cx)
        odom.pose.pose.position.y = float(cy)
        odom.pose.pose.position.z = 0.0
        # yaw → 四元数
        odom.pose.pose.orientation.z = float(math.sin(cyaw / 2))
        odom.pose.pose.orientation.w = float(math.cos(cyaw / 2))

        # 速度 (body frame)
        vx_w = self.data.qvel[self.joint_qvel.get('chassis_x', 0)]
        vy_w = self.data.qvel[self.joint_qvel.get('chassis_y', 0)]
        wz = self.data.qvel[self.joint_qvel.get('chassis_yaw', 0)]
        # 世界速度 → body frame
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
        t.transform.translation.z = 0.16  # chassis height
        t.transform.rotation.z = float(math.sin(cyaw / 2))
        t.transform.rotation.w = float(math.cos(cyaw / 2))
        self.tf_broadcaster.sendTransform(t)

    # ---- 键盘处理 ----
    def _on_key_press(self, key):
        try:
            ch = key.char.lower() if hasattr(key, 'char') and key.char else ''
        except AttributeError:
            ch = ''
        with self.key_lock:
            if ch in self.key_held:
                self.key_held[ch] = True
            elif ch == 'r':
                self.reset_request = True
            elif ch == 'f':
                self.toggle_fpv = True
            elif key == pynput_keyboard.Key.esc:
                self.exit_request = True
                return False

    def _on_key_release(self, key):
        try:
            ch = key.char.lower() if hasattr(key, 'char') and key.char else ''
        except AttributeError:
            ch = ''
        with self.key_lock:
            if ch in self.key_held:
                self.key_held[ch] = False

    def _is_key(self, ch):
        with self.key_lock:
            return self.key_held.get(ch, False)

    # ---- 速度更新 ----
    def update_targets(self, dt):
        """整合键盘 + 外部 cmd_vel → 目标速度"""

        # 外部 cmd_vel 优先（0.5s 超时后回退键盘）
        ext_active = (self.ext_cmd_vel is not None and
                      time.time() - self.ext_cmd_vel_time < self.EXT_CMD_TIMEOUT)

        if ext_active:
            desired_vx = np.clip(self.ext_cmd_vel.linear.x, -MAX_VX, MAX_VX)
            desired_vy = np.clip(self.ext_cmd_vel.linear.y, -MAX_VY, MAX_VY)
            desired_omega = np.clip(self.ext_cmd_vel.angular.z, -MAX_OMEGA, MAX_OMEGA)
        else:
            # 键盘开关量
            w, s = self._is_key('w'), self._is_key('s')
            a, d = self._is_key('a'), self._is_key('d')
            q, e = self._is_key('q'), self._is_key('e')

            desired_vx = (CHASSIS_SPEED if w and not s else
                          (-CHASSIS_SPEED if s and not w else 0.0))
            desired_vy = (CHASSIS_SPEED if a and not d else
                          (-CHASSIS_SPEED if d and not a else 0.0))
            desired_omega = (CHASSIS_OMEGA_SPEED if q and not e else
                             (-CHASSIS_OMEGA_SPEED if e and not q else 0.0))

        # 斜率限幅
        self.target_vx = slew(self.target_vx, desired_vx, CHASSIS_VX_SLEW, dt)
        self.target_vy = slew(self.target_vy, desired_vy, CHASSIS_VY_SLEW, dt)
        self.target_omega = slew(self.target_omega, desired_omega, CHASSIS_OMEGA_SLEW, dt)

        # 死区
        if abs(self.target_vx) < 0.005: self.target_vx = 0.0
        if abs(self.target_vy) < 0.005: self.target_vy = 0.0
        if abs(self.target_omega) < 0.005: self.target_omega = 0.0

        # 键盘升降 (外部 lift_cmd 已在回调里处理)
        if not ext_active:
            g, h = self._is_key('g'), self._is_key('h')
            if g and not h:
                self.target_lift = min(self.target_lift + LIFT_RATE * dt, LIFT_MAX)
            elif h and not g:
                self.target_lift = max(self.target_lift - LIFT_RATE * dt, LIFT_MIN)

        # 升降低通滤波
        self.smoothed_lift = 0.05 * self.target_lift + 0.95 * self.smoothed_lift

        # 键盘云台 (外部 head_cmd 已在回调里处理)
        if not ext_active:
            step = HEAD_RATE * dt
            if self._is_key('i') and not self._is_key('k'):
                self.target_head_pitch = max(self.target_head_pitch - step, HEAD_PITCH_MIN)
            elif self._is_key('k') and not self._is_key('i'):
                self.target_head_pitch = min(self.target_head_pitch + step, HEAD_PITCH_MAX)
            if self._is_key('j') and not self._is_key('l'):
                self.target_head_yaw = min(self.target_head_yaw + step, HEAD_YAW_MAX)
            elif self._is_key('l') and not self._is_key('j'):
                self.target_head_yaw = max(self.target_head_yaw - step, HEAD_YAW_MIN)
            ss = HEAD_STEM_RATE * dt
            if self._is_key('t') and not self._is_key('y'):
                self.target_head_stem = min(self.target_head_stem + ss, HEAD_STEM_MAX)
            elif self._is_key('y') and not self._is_key('t'):
                self.target_head_stem = max(self.target_head_stem - ss, HEAD_STEM_MIN)

        # 发布当前 cmd_vel (让外部节点知道底盘指令)
        twist = Twist()
        twist.linear.x = self.target_vx
        twist.linear.y = self.target_vy
        twist.angular.z = self.target_omega
        self.pub_cmd_vel.publish(twist)

    # ---- 仿真主循环 ----
    def run(self):
        """主循环: MuJoCo 仿真 + viewer"""

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

        # Settling
        self.get_logger().info('物体 settling (1.0s)...')
        for _ in range(500):
            data.ctrl[CTRL_LEFT_ARM:CTRL_LEFT_ARM + 6] = ARM_HOME_LEFT
            data.ctrl[CTRL_GRIPPER] = GRIPPER_CLOSE
            mujoco.mj_step(model, data)
        self.get_logger().info('Settling 完成')

        init_qpos = data.qpos.copy()
        init_qvel = data.qvel.copy()

        # 键盘监听
        listener = None
        if self.enable_keyboard and HAS_PYNPUT:
            try:
                subprocess.run(['xset', '-r'], check=False, timeout=2,
                               env={**os.environ, 'DISPLAY': ':0'})
            except Exception:
                pass
            listener = pynput_keyboard.Listener(
                on_press=self._on_key_press, on_release=self._on_key_release)
            listener.start()
            self.get_logger().info('键盘监听已启动')

            def _restore():
                try:
                    subprocess.run(['xset', 'r', 'on'], check=False, timeout=2,
                                   env={**os.environ, 'DISPLAY': ':0'})
                except Exception:
                    pass
            atexit.register(_restore)

        # ROS2 spin 在独立线程
        spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        spin_thread.start()

        # 查找 head_cam
        head_cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'head_cam')
        fpv_mode = False

        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.distance = 3.0
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -25
            viewer.cam.lookat[:] = [0, 0, 0.5]

            while viewer.is_running() and not self.exit_request and rclpy.ok():
                dt = model.opt.timestep

                # FPV 切换
                if self.toggle_fpv:
                    self.toggle_fpv = False
                    fpv_mode = not fpv_mode
                    if fpv_mode and head_cam_id >= 0:
                        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                        viewer.cam.fixedcamid = head_cam_id
                    else:
                        fpv_mode = False
                        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                        viewer.cam.distance = 3.0
                        viewer.cam.azimuth = 135
                        viewer.cam.elevation = -25
                        viewer.cam.lookat[:] = [0, 0, 0.5]

                # 重置
                if self.reset_request:
                    data.qpos[:] = init_qpos
                    data.qvel[:] = init_qvel
                    data.ctrl[:] = 0.0
                    data.ctrl[CTRL_LIFT] = LIFT_INIT
                    data.ctrl[CTRL_LEFT_ARM:CTRL_LEFT_ARM + 6] = ARM_HOME_LEFT
                    data.ctrl[CTRL_GRIPPER] = GRIPPER_CLOSE
                    self.target_vx = self.target_vy = self.target_omega = 0.0
                    self.target_lift = LIFT_INIT
                    self.smoothed_lift = LIFT_INIT
                    self.target_head_yaw = self.target_head_pitch = 0.0
                    self.target_head_stem = 0.0
                    self.current_steer[:] = 0.0
                    self.current_drive[:] = 0.0
                    data.xfrc_applied[:] = 0.0
                    mujoco.mj_forward(model, data)
                    self.reset_request = False
                    self.get_logger().info('RESET')

                # 1. 更新目标
                self.update_targets(dt)

                # 2. 舵轮 IK
                raw_steer, raw_drive = swerve_ik(
                    self.target_vx, self.target_vy, self.target_omega, self.current_steer)
                raw_drive = desaturate(raw_drive, MAX_WHEEL_SPEED)

                cmd_steer, cmd_drive = np.zeros(4), np.zeros(4)
                for i in range(4):
                    sp, an = optimize_module(raw_drive[i], raw_steer[i], self.current_steer[i])
                    cmd_steer[i] = rate_limit(an, self.current_steer[i], STEER_RATE_LIMIT, dt)
                    cmd_drive[i] = rate_limit(sp, self.current_drive[i], DRIVE_ACCEL_LIMIT, dt)
                self.current_steer[:] = cmd_steer
                self.current_drive[:] = cmd_drive

                # 3. 写入 MuJoCo ctrl
                data.ctrl[CTRL_VX] = self.target_vx
                data.ctrl[CTRL_VY] = self.target_vy
                data.ctrl[CTRL_OMEGA] = self.target_omega
                data.ctrl[CTRL_STEER:CTRL_STEER + 4] = cmd_steer
                data.ctrl[CTRL_DRIVE:CTRL_DRIVE + 4] = cmd_drive
                data.ctrl[CTRL_LIFT] = self.smoothed_lift
                data.ctrl[CTRL_LEFT_ARM:CTRL_LEFT_ARM + 6] = ARM_HOME_LEFT
                data.ctrl[CTRL_HEAD_YAW] = self.target_head_yaw
                data.ctrl[CTRL_HEAD_PITCH] = self.target_head_pitch
                data.ctrl[CTRL_HEAD_STEM] = self.target_head_stem

                # 4. 物理步进
                mujoco.mj_step(model, data)

                # 5. 渲染
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
