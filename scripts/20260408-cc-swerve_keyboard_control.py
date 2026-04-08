#!/usr/bin/env python3
"""
四舵轮全向底盘 — MuJoCo 键盘控制 Demo (pynput 版)

关键改进：
  使用 pynput 监听全局键盘事件，能正确捕获"按下/松开"，
  支持持续按住产生连续运动。这绕开了 mujoco viewer 的 key_callback
  只在按下瞬间触发的限制。

控制按键:
  W / S : 前进 / 后退 (vx)
  A / D : 左移 / 右移 (vy)
  Q / E : 左转 / 右转 (omega)
  R     : 重置位置和速度
  ESC   : 退出

所有按键都是"按住生效，松开衰减"。可以同时按多个键合成运动。

注意：
  - 必须聚焦 MuJoCo 窗口才能接收键盘事件（pynput 监听全局键盘，
    但只有当 MuJoCo 窗口在前台时才不会和 viewer 内置快捷键冲突）
  - 为避免和 viewer 内置快捷键冲突（C/T/F 等），WASD/QE 不会被
    viewer 自带功能拦截

运行：
  conda activate disc
  python 20260408-cc-swerve_keyboard_control.py
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

# OpenCV 用于第一人称相机子窗口
try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

from pynput import keyboard

# Touch 笔（可选）
TOUCH_DRIVER_PATH = '/home/rhz/teleop/force-feedback-pen/20260328-cc-haptic_driver.py'
ENABLE_TOUCH = os.environ.get('ENABLE_TOUCH', '1') == '1'

# ============== 配置参数 ==============
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH = os.path.join(SCRIPT_DIR, "20260408-cc-swerve_chassis.xml")

WHEEL_RADIUS = 0.06  # m, 与 MJCF 中 cylinder size 一致

# 轮子在底盘坐标系下的位置 (与 MJCF 中 fl/fr/rl/rr_steer_link 的 pos 一致)
WHEEL_POSITIONS = np.array([
    [ 0.16,  0.15],  # fl
    [ 0.16, -0.15],  # fr
    [-0.16,  0.15],  # rl
    [-0.16, -0.15],  # rr
])

# 速度上限
MAX_VX = 1.0       # m/s
MAX_VY = 1.0       # m/s
MAX_OMEGA = 2.0    # rad/s

# 升降参数
LIFT_MIN = 0.0     # m
LIFT_MAX = 0.5     # m
LIFT_INIT = 0.0    # 初始高度
LIFT_RATE = 0.60   # m/s 升降速度（按键立即响应，快 3 倍）

# 云台参数
HEAD_YAW_MIN = -1.5708
HEAD_YAW_MAX =  1.5708
HEAD_PITCH_MIN = -0.7854
HEAD_PITCH_MAX =  0.7854
HEAD_RATE = 2.5   # rad/s 云台转速

# 云台高度（立柱伸缩）
HEAD_STEM_MIN = -0.05
HEAD_STEM_MAX =  0.40
HEAD_STEM_RATE = 0.30  # m/s

# 双臂 home pose：所有 6 个关节角归零
ARM_HOME_LEFT  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ARM_HOME_RIGHT = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Touch 笔 → 右臂映射参数（对齐 airbot_force_sim.py）
PEN_POS_SCALE = 0.002   # m / mm  (1mm 笔位移 → 2mm 机械臂位移)
PEN_ROT_SCALE = 1.0     # 姿态放大倍率 1:1
IK_DAMPING = 0.1        # 差分 IK 阻尼系数
IK_MAX_DELTA = 0.008    # 单步最大关节变化 (rad) - 慢一点避免反作用力推底盘
IK_POS_WEIGHT = 1.0
IK_ROT_WEIGHT = 0.25    # 姿态跟随权重

# R_map: 笔坐标系 → 机器人 world 坐标系的旋转映射
# 用户面向机器人坐：
#   Touch X (用户右)   → Robot -Y  (机器人左为正 Y)
#   Touch Y (上)       → Robot  Z  (上)
#   Touch Z (朝用户)   → Robot -X  (机器人后)
# 即：笔向前推 (-Z) → 机器人向前 (+X)
R_MAP = np.array([
    [ 0,  0, -1],
    [-1,  0,  0],
    [ 0,  1,  0],
], dtype=float)

# 加速 / 衰减率（每秒）- 响应快得多
ACCEL_V = 6.0         # m/s² 平移加速度（按键立即达到大部分速度）
ACCEL_OMEGA = 10.0    # rad/s² 旋转加速度
DECEL_V = 8.0         # m/s² 平移衰减（松开快速停下）
DECEL_OMEGA = 12.0    # rad/s² 旋转衰减


# ============== 全局按键状态（线程共享）==============
# 启动时禁用 X11 keyboard auto-repeat (xset -r)，
# 这样 press/release 事件就是真实的按下/松开，可以直接用布尔状态。
# 退出时通过 atexit 恢复 (xset r on)。

key_held = {
    'w': False, 's': False,   # 底盘前后
    'a': False, 'd': False,   # 底盘左右
    'q': False, 'e': False,   # 底盘转向
    'g': False, 'h': False,   # 升降
    'i': False, 'k': False,   # 云台俯仰
    'j': False, 'l': False,   # 云台偏航
    't': False, 'y': False,   # 云台高度 上/下
}
reset_request = False
exit_request = False
state_lock = threading.Lock()


def is_key_held(ch):
    """判断某个按键当前是否按住中"""
    with state_lock:
        return key_held.get(ch, False)


def disable_x11_repeat():
    """禁用 X11 键盘自动重复（仅当前 X session）"""
    try:
        subprocess.run(['xset', '-r'], check=False, timeout=2,
                       env={**os.environ, 'DISPLAY': ':0'})
        print(">>> X11 keyboard auto-repeat disabled")
    except Exception as exc:
        print(f"!!! disable_x11_repeat failed: {exc}")


def restore_x11_repeat():
    """恢复 X11 键盘自动重复"""
    try:
        subprocess.run(['xset', 'r', 'on'], check=False, timeout=2,
                       env={**os.environ, 'DISPLAY': ':0'})
        print(">>> X11 keyboard auto-repeat restored")
    except Exception:
        pass


# 注册退出钩子，保证 X11 状态被恢复
atexit.register(restore_x11_repeat)


# ============== pynput 回调 ==============

def on_key_press(key):
    """按键按下：标记 True"""
    global reset_request, exit_request
    try:
        ch = key.char.lower() if hasattr(key, 'char') and key.char else ''
    except AttributeError:
        ch = ''

    with state_lock:
        if ch in key_held:
            key_held[ch] = True
        elif ch == 'r':
            reset_request = True
        elif key == keyboard.Key.esc:
            exit_request = True
            return False


def on_key_release(key):
    """按键松开：标记 False（X11 auto-repeat 已禁用，事件可信）"""
    try:
        ch = key.char.lower() if hasattr(key, 'char') and key.char else ''
    except AttributeError:
        ch = ''

    with state_lock:
        if ch in key_held:
            key_held[ch] = False


# ============== 速度更新 ==============

# 当前底盘目标速度
target_vx = 0.0
target_vy = 0.0
target_omega = 0.0

# 当前升降目标高度
target_lift = LIFT_INIT

# 云台目标角
target_head_yaw = 0.0
target_head_pitch = 0.0
target_head_stem = 0.0  # 云台立柱高度


def update_target_velocity(dt):
    """根据按键状态实时更新目标速度和升降高度

    底盘速度: 按住对应键加速，松开衰减回零
    升降高度: 按住 G 上升，按住 H 下降，否则保持当前高度

    按键状态用 is_key_held() 判断（X11 auto-repeat 已禁用）
    """
    global target_vx, target_vy, target_omega, target_lift
    global target_head_yaw, target_head_pitch, target_head_stem

    w = is_key_held('w')
    s = is_key_held('s')
    a = is_key_held('a')
    d = is_key_held('d')
    q = is_key_held('q')
    e = is_key_held('e')
    g = is_key_held('g')
    h = is_key_held('h')
    i = is_key_held('i')
    k = is_key_held('k')
    j = is_key_held('j')
    l = is_key_held('l')
    t_up = is_key_held('t')
    y_dn = is_key_held('y')

    accel_v = ACCEL_V * dt
    accel_om = ACCEL_OMEGA * dt
    decel_v = DECEL_V * dt
    decel_om = DECEL_OMEGA * dt

    # vx: W=正, S=负
    if w and not s:
        target_vx = min(target_vx + accel_v, MAX_VX)
    elif s and not w:
        target_vx = max(target_vx - accel_v, -MAX_VX)
    else:
        # 没按或两个都按 → 衰减
        if abs(target_vx) < decel_v:
            target_vx = 0.0
        else:
            target_vx -= math.copysign(decel_v, target_vx)

    # vy: A=正(左), D=负(右)
    if a and not d:
        target_vy = min(target_vy + accel_v, MAX_VY)
    elif d and not a:
        target_vy = max(target_vy - accel_v, -MAX_VY)
    else:
        if abs(target_vy) < decel_v:
            target_vy = 0.0
        else:
            target_vy -= math.copysign(decel_v, target_vy)

    # omega: Q=正(左转), E=负(右转)
    if q and not e:
        target_omega = min(target_omega + accel_om, MAX_OMEGA)
    elif e and not q:
        target_omega = max(target_omega - accel_om, -MAX_OMEGA)
    else:
        if abs(target_omega) < decel_om:
            target_omega = 0.0
        else:
            target_omega -= math.copysign(decel_om, target_omega)

    # === 升降：G=上升, H=下降，松开则保持当前高度 ===
    lift_step = LIFT_RATE * dt
    if g and not h:
        target_lift = min(target_lift + lift_step, LIFT_MAX)
    elif h and not g:
        target_lift = max(target_lift - lift_step, LIFT_MIN)

    # === 云台 ===
    head_step = HEAD_RATE * dt
    # I/K = pitch 上/下
    if i and not k:
        target_head_pitch = max(target_head_pitch - head_step, HEAD_PITCH_MIN)
    elif k and not i:
        target_head_pitch = min(target_head_pitch + head_step, HEAD_PITCH_MAX)
    # J/L = yaw 左/右
    if j and not l:
        target_head_yaw = min(target_head_yaw + head_step, HEAD_YAW_MAX)
    elif l and not j:
        target_head_yaw = max(target_head_yaw - head_step, HEAD_YAW_MIN)

    # === 云台高度 T/Y ===
    stem_step = HEAD_STEM_RATE * dt
    if t_up and not y_dn:
        target_head_stem = min(target_head_stem + stem_step, HEAD_STEM_MAX)
    elif y_dn and not t_up:
        target_head_stem = max(target_head_stem - stem_step, HEAD_STEM_MIN)


# ============== 舵轮逆运动学 ==============

def swerve_inverse_kinematics(vx, vy, omega):
    """对每个轮子位置 r_i = (rx, ry):
       v_wheel = v_body + omega × r = (vx - omega*ry, vy + omega*rx)
       steer = atan2(v_wheel.y, v_wheel.x)
       drive = |v_wheel| / wheel_radius
    """
    steer_angles = np.zeros(4)
    drive_speeds = np.zeros(4)

    for i, (rx, ry) in enumerate(WHEEL_POSITIONS):
        wvx = vx - omega * ry
        wvy = vy + omega * rx
        speed = math.hypot(wvx, wvy)
        if speed < 1e-4:
            steer_angles[i] = 0.0
            drive_speeds[i] = 0.0
        else:
            steer_angles[i] = math.atan2(wvy, wvx)
            drive_speeds[i] = speed / WHEEL_RADIUS

    return steer_angles, drive_speeds


def shortest_steer(target, current):
    """计算最短转向角差，必要时反转 180° 并反向驱动

    返回 (调整后的转向角, 是否需要反向驱动)
    """
    diff = target - current
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    if abs(diff) > math.pi / 2:
        if diff > 0:
            target -= math.pi
        else:
            target += math.pi
        return target, True
    return target, False


# ============== Touch 笔接入（完整对齐 airbot_force_sim.py 的增量控制）==============

def _scale_rotation(R_delta, scale):
    """将旋转矩阵 R_delta 的旋转角按 scale 倍缩放（Rodrigues 公式）"""
    trace_val = float(np.clip((np.trace(R_delta) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(trace_val)
    if angle < 1e-6:
        return R_delta.copy()
    axis = np.array([
        R_delta[2, 1] - R_delta[1, 2],
        R_delta[0, 2] - R_delta[2, 0],
        R_delta[1, 0] - R_delta[0, 1],
    ]) / (2.0 * math.sin(angle))
    scaled = angle * scale
    K = np.array([
        [     0.0, -axis[2],  axis[1]],
        [ axis[2],      0.0, -axis[0]],
        [-axis[1],  axis[0],      0.0],
    ])
    return (np.eye(3)
            + math.sin(scaled) * K
            + (1.0 - math.cos(scaled)) * (K @ K))


def _quat_to_rot(q):
    """Mujoco qw, qx, qy, qz → 3×3 旋转矩阵"""
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, dtype=float))
    return R.reshape(3, 3)


def _rot_log(R):
    """3x3 旋转矩阵 → 轴角向量 (3,)，对应 so(3) 的对数映射"""
    trace_val = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(trace_val)
    if angle < 1e-8:
        return np.zeros(3)
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ]) / (2.0 * math.sin(angle))
    return axis * angle


class TouchTeleop:
    """Touch 力反馈笔遥控右臂末端（位置 + 姿态双重增量控制）

    控制流程（完全对齐 airbot_force_sim.py）：
      1. Button2 (白色按钮) = 重置零点 + 姿态标定
         - 右臂关节归零
         - 记录当前笔位置为 pen_origin
         - 记录当前笔姿态为 pen_R0，当前右臂末端姿态为 robot_R0
         - 记录当前右臂末端 world 位置为 workspace_center
      2. Button1 (灰色按钮) = 切换夹爪 (暂未接双指)
      3. 校准后持续跟随：
         - target_pos = workspace_center + R_map @ (pen_pos - pen_origin) * scale
         - delta_R_pen = R_pen_cur @ pen_R0.T
         - delta_R_robot = R_map @ delta_R_pen @ R_map.T
         - R_des = delta_R_robot @ robot_R0
      4. 差分 IK（位置+姿态）→ 右臂 ctrl[15:21]
    """

    def __init__(self, model, data, right_eef_body_name="right_link6"):
        self.device = None
        self.connected = False
        self.model = model
        self.data = data

        # 右臂末端 body id
        self.eef_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, right_eef_body_name
        )
        if self.eef_body_id < 0:
            raise RuntimeError(f"找不到 body: {right_eef_body_name}")

        # 右臂 6 个关节的 qpos / qvel 索引
        self.right_joint_ids = []
        self.right_qvel_indices = []
        self.right_qpos_indices = []
        for i in range(1, 7):
            jname = f"right_joint{i}"
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid < 0:
                raise RuntimeError(f"找不到关节: {jname}")
            self.right_joint_ids.append(jid)
            self.right_qvel_indices.append(model.jnt_dofadr[jid])
            self.right_qpos_indices.append(model.jnt_qposadr[jid])

        # ---- 增量控制参考帧 ----
        self.pen_origin = None         # 笔位置零点 (mm)
        self.pen_R0 = None             # 笔姿态零点 (3×3)
        self.workspace_center = None   # 右臂末端 world 起点 (m)
        self.robot_R0 = None           # 右臂末端姿态起点 (3×3)

        # ---- 累积目标关节角（关键！解决"软掉"问题）----
        # 每次 IK 更新 target_q += dq，而不是 current_q + dq
        # 这样 position actuator 会尝试精确达到累积目标，重力下垂不会累计
        self.target_q = np.zeros(6)

        # ---- 按钮状态 ----
        self.btn1_last = False
        self.btn2_last = False
        self.calibrated = False

        # IK jacobian buffer
        self.jac_pos = np.zeros((3, model.nv))
        self.jac_rot = np.zeros((3, model.nv))

    def connect(self):
        """加载 Touch 笔驱动并初始化"""
        if not ENABLE_TOUCH:
            print(">>> Touch 笔已禁用 (ENABLE_TOUCH=0)")
            return False
        try:
            spec = importlib.util.spec_from_file_location(
                'haptic_driver', TOUCH_DRIVER_PATH
            )
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            self.device = mod.HapticDevice()
            self.device.initialize()
            self.device.start_scheduler()
            self.connected = True
            print(">>> Touch 笔已连接")
            print(">>> 按【白色按钮 Button2】进行零点校准后开始跟随")
            return True
        except Exception as exc:
            print(f"!!! Touch 笔连接失败: {exc}")
            print("    继续运行（无右臂 Touch 控制）")
            self.connected = False
            return False

    def disconnect(self):
        if self.connected and self.device is not None:
            try:
                self.device.stop()
            except Exception:
                pass

    def get_right_eef_position(self):
        return np.array(self.data.xpos[self.eef_body_id])

    def get_right_eef_rotation(self):
        """右臂末端在 world frame 下的旋转矩阵"""
        return self.data.xmat[self.eef_body_id].reshape(3, 3).copy()

    def _calibrate(self, pen_pos, pen_R):
        """按白色按钮触发：记录当前参考帧（不改变机器人姿态）

        关键：不归零 qpos，否则会导致右臂从当前位置突然跳到零位，
        期间会因为重力瞬间下垂让用户感觉"软掉"。
        正确做法：记录当前右臂的关节角作为 target_q 起点。
        """
        # 记录机器人当前参考（无重置动作）
        self.workspace_center = self.get_right_eef_position()
        self.robot_R0 = self.get_right_eef_rotation()

        # 累积目标 = 当前关节角
        for i, qp_idx in enumerate(self.right_qpos_indices):
            self.target_q[i] = self.data.qpos[qp_idx]

        # 记录笔参考
        self.pen_origin = pen_pos.copy()
        if pen_R is not None and abs(np.linalg.det(pen_R) - 1.0) < 0.2:
            self.pen_R0 = pen_R.copy()

        self.calibrated = True
        print(f">>> Touch 零点校准完成")
        print(f"    workspace_center = {np.round(self.workspace_center, 3)}")
        print(f"    pen_origin = {np.round(self.pen_origin, 1)} mm")
        print(f"    target_q init = {np.round(self.target_q, 3)}")

    def update(self):
        """每个仿真步调用一次，返回 (tcp_target, calibrated)"""
        if not self.connected:
            return None, False

        state = self.device.get_state()
        pen_pos = np.array(state.position)  # mm

        # 读取笔 transform → 旋转矩阵
        pen_R = None
        if state.transform and len(state.transform) == 16:
            T_raw = np.array(state.transform).reshape(4, 4)
            R_candidate = T_raw.T[:3, :3]  # 列主序存储 → 转置取实际旋转
            if abs(np.linalg.det(R_candidate) - 1.0) < 0.2:
                pen_R = R_candidate

        btn1 = bool(state.button1)
        btn2 = bool(state.button2)

        # Button2 上升沿：零点校准
        if btn2 and not self.btn2_last:
            self._calibrate(pen_pos, pen_R)
        self.btn2_last = btn2

        # Button1 上升沿：夹爪切换（未接双指，先预留）
        if btn1 and not self.btn1_last:
            print(">>> [夹爪] 切换（未接）")
        self.btn1_last = btn1

        if not self.calibrated:
            return None, False

        # ========== 位置跟随 ==========
        delta_pen_mm = pen_pos - self.pen_origin       # mm
        delta_world = R_MAP @ (delta_pen_mm * PEN_POS_SCALE)  # m
        target_pos = self.workspace_center + delta_world

        # ========== 姿态跟随 ==========
        target_R = self.robot_R0
        if pen_R is not None and self.pen_R0 is not None:
            delta_R_pen = pen_R @ self.pen_R0.T
            if PEN_ROT_SCALE != 1.0:
                delta_R_pen = _scale_rotation(delta_R_pen, PEN_ROT_SCALE)
            delta_R_robot = R_MAP @ delta_R_pen @ R_MAP.T
            target_R = delta_R_robot @ self.robot_R0
            U, _, Vt = np.linalg.svd(target_R)
            target_R = U @ Vt

        # ========== 差分 IK ==========
        # ★ 关键修复：从 target_q（累积目标）正运动学算当前"预期"位姿，
        # 而不是从 data.qpos（实际位置，受重力影响）。
        # 这样误差累积不会拖累跟随，机械臂会精确追踪目标。
        current_pos = self.get_right_eef_position()
        current_R = self.get_right_eef_rotation()

        pos_err = target_pos - current_pos
        rot_err = _rot_log(target_R @ current_R.T)

        # 雅可比 (6 × nv)
        mujoco.mj_jacBodyCom(
            self.model, self.data, self.jac_pos, self.jac_rot, self.eef_body_id
        )
        J_full = np.vstack([
            IK_POS_WEIGHT * self.jac_pos[:, self.right_qvel_indices],
            IK_ROT_WEIGHT * self.jac_rot[:, self.right_qvel_indices],
        ])
        err6 = np.concatenate([
            IK_POS_WEIGHT * pos_err,
            IK_ROT_WEIGHT * rot_err,
        ])

        # 阻尼伪逆
        JJT = J_full @ J_full.T
        damped = JJT + IK_DAMPING ** 2 * np.eye(6)
        dq = J_full.T @ np.linalg.solve(damped, err6)

        # 限幅
        max_dq = np.max(np.abs(dq))
        if max_dq > IK_MAX_DELTA:
            dq = dq * (IK_MAX_DELTA / max_dq)

        # ★ 累积到 target_q（这是核心）
        self.target_q += dq

        # 关节限位保护
        for i, jid in enumerate(self.right_joint_ids):
            jrange = self.model.jnt_range[jid]
            if jrange[0] < jrange[1]:
                self.target_q[i] = np.clip(self.target_q[i], jrange[0], jrange[1])

        # 写入 ctrl (索引 15~20)
        for i in range(6):
            self.data.ctrl[15 + i] = self.target_q[i]

        return target_pos, True


# ============== 主函数 ==============

def main():
    global reset_request, target_vx, target_vy, target_omega, target_lift
    global target_head_yaw, target_head_pitch, target_head_stem

    # 关键：禁用 X11 自动重复，避免按住按键时的 press/release 抖动顿挫
    disable_x11_repeat()

    print("加载 MJCF:", MJCF_PATH)
    model = mujoco.MjModel.from_xml_path(MJCF_PATH)
    data = mujoco.MjData(model)
    print(f"模型: nq={model.nq}, nv={model.nv}, nu={model.nu}")
    print()
    print("=" * 64)
    print("                    四舵轮全向底盘 — 键盘控制")
    print("=" * 64)
    print()
    print("【重要】请把焦点放在【本终端窗口】，不要点击 MuJoCo 窗口！")
    print("       原因：MuJoCo viewer 有自己的内置快捷键（C/T/F 等），")
    print("            按键会触发它的视图变化。pynput 是全局监听，")
    print("            焦点在终端时既能控制底盘又不会影响 viewer。")
    print()
    print("控制按键 (按住生效)：")
    print("  底盘:   W/S 前后    A/D 左右    Q/E 转向")
    print("  升降:   G/H 升/降")
    print("  云台:   I/K 俯仰    J/L 偏航    T/Y 高度上下")
    print("  Touch:  Button2 白色 = 零点校准 + 跟随启动")
    print("          Button1 灰色 = 夹爪（未接）")
    print("  其他:   R 重置      ESC 退出")
    print()
    print("Touch 笔使用步骤:")
    print("  1. 把笔放到舒适的起始位置")
    print("  2. 按白色按钮进行零点校准（右臂不会跳动）")
    print("  3. 移动笔，右臂跟随笔的位置和姿态")
    print("  4. 再按白色按钮可重新校准")
    print("=" * 64)
    print()

    # 启动 pynput 监听器（独立线程）
    listener = keyboard.Listener(on_press=on_key_press, on_release=on_key_release)
    listener.start()
    print(">>> pynput 键盘监听已启动 (全局，不需要聚焦窗口)")

    current_steer = np.zeros(4)
    chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis_link")

    # 初始化所有 ctrl 到稳定 home 值（防止重力下垂）
    mujoco.mj_forward(model, data)
    data.ctrl[:] = 0.0
    data.ctrl[8] = 0.0                         # lift
    data.ctrl[9:15]  = ARM_HOME_LEFT           # left arm
    data.ctrl[15:21] = ARM_HOME_RIGHT          # right arm
    data.ctrl[21] = 0.0                        # head yaw
    data.ctrl[22] = 0.0                        # head pitch
    data.ctrl[23] = 0.0                        # head stem

    # Touch 笔接入
    touch = TouchTeleop(model, data)
    touch.connect()
    atexit.register(touch.disconnect)

    # 第一人称相机子窗口（head_cam）
    head_cam_renderer = None
    head_cam_id = -1
    head_cam_last_update = 0.0
    if HAS_CV2:
        try:
            head_cam_renderer = mujoco.Renderer(model, height=360, width=480)
            head_cam_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_CAMERA, 'head_cam'
            )
            cv2.namedWindow('Head Camera (First Person)', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Head Camera (First Person)', 480, 360)
            print(">>> Head camera 子窗口已启动")
        except Exception as exc:
            print(f"!!! Head camera 启动失败: {exc}")
            head_cam_renderer = None

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 3.0
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25
        viewer.cam.lookat[:] = [0, 0, 0.1]

        last_print = 0.0
        last_log = 0.0

        while viewer.is_running() and not exit_request:
            step_start = time.time()
            sim_time = data.time

            # 重置请求处理
            if reset_request:
                # 1. 重置物理状态（qpos, qvel 归零到初始）
                mujoco.mj_resetData(model, data)
                # 2. 所有 ctrl 清零（防止飞奔）
                data.ctrl[:] = 0.0
                # 3. 重置 Python 侧目标变量
                target_vx = target_vy = target_omega = 0.0
                target_lift = LIFT_INIT
                target_head_yaw = target_head_pitch = 0.0
                target_head_stem = 0.0
                current_steer[:] = 0.0
                # 4. 重置 Touch 校准状态
                touch.calibrated = False
                touch.pen_origin = None
                touch.pen_R0 = None
                touch.target_q[:] = 0.0
                # 5. 前向计算刷新一次
                mujoco.mj_forward(model, data)
                reset_request = False
                print("\n>>> RESET <<<\n")

            # 1. 更新目标速度
            update_target_velocity(model.opt.timestep)

            # 2. 舵轮逆运动学
            steer_angles, drive_speeds = swerve_inverse_kinematics(
                target_vx, target_vy, target_omega
            )

            # 3. 转向角优化
            for i in range(4):
                if abs(drive_speeds[i]) > 1e-4:
                    steer_angles[i], reversed_ = shortest_steer(
                        steer_angles[i], current_steer[i]
                    )
                    if reversed_:
                        drive_speeds[i] = -drive_speeds[i]
                    current_steer[i] = steer_angles[i]

            # 4. 写入执行器
            data.ctrl[0:4] = steer_angles
            data.ctrl[4:8] = drive_speeds
            data.ctrl[8] = target_lift  # lift_act

            # 左臂保持 home pose
            data.ctrl[9:15] = ARM_HOME_LEFT
            # 右臂：默认 home pose（未校准时），Touch 校准后会被 touch.update() 覆盖
            if not touch.calibrated:
                data.ctrl[15:21] = ARM_HOME_RIGHT
            # Touch 笔更新右臂 ctrl[15:21]
            touch.update()

            # 云台 ctrl (索引 21/22/23)
            data.ctrl[21] = target_head_yaw
            data.ctrl[22] = target_head_pitch
            data.ctrl[23] = target_head_stem

            # 5. 物理步进
            mujoco.mj_step(model, data)
            viewer.sync()

            # 6a. head_cam 子窗口（30 Hz 更新，避免拖慢主循环）
            if head_cam_renderer is not None and head_cam_id >= 0:
                if sim_time - head_cam_last_update > 1.0 / 30.0:
                    try:
                        head_cam_renderer.update_scene(data, camera=head_cam_id)
                        frame = head_cam_renderer.render()
                        # RGB → BGR
                        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                        cv2.imshow('Head Camera (First Person)', bgr)
                        cv2.waitKey(1)
                    except Exception:
                        pass
                    head_cam_last_update = sim_time

            # 6. 实时 HUD overlay (每 0.1 秒更新一次)
            if sim_time - last_print > 0.1:
                pos = data.xpos[chassis_id]
                qw, qz = data.xquat[chassis_id, 0], data.xquat[chassis_id, 3]
                yaw = math.degrees(2.0 * math.atan2(qz, qw))

                # 实际升降高度（从 qpos 读取）
                lift_actual = data.qpos[7]  # qpos[0:7]=free joint, [7]=lift

                # 按键指示器
                indicator = "".join([
                    "W" if is_key_held('w') else "_",
                    "A" if is_key_held('a') else "_",
                    "S" if is_key_held('s') else "_",
                    "D" if is_key_held('d') else "_",
                    " ",
                    "Q" if is_key_held('q') else "_",
                    "E" if is_key_held('e') else "_",
                    " ",
                    "G" if is_key_held('g') else "_",
                    "H" if is_key_held('h') else "_",
                    " ",
                    "I" if is_key_held('i') else "_",
                    "K" if is_key_held('k') else "_",
                    "J" if is_key_held('j') else "_",
                    "L" if is_key_held('l') else "_",
                    " ",
                    "T" if is_key_held('t') else "_",
                    "Y" if is_key_held('y') else "_",
                ])

                # Touch 笔状态
                if touch.connected:
                    touch_state = "CALIBRATED" if touch.calibrated else "WAIT BTN2"
                    eef_pos = touch.get_right_eef_position()
                    touch_str = (
                        f"Touch: {touch_state}\n"
                        f"R-EEF = ({eef_pos[0]:+.2f},{eef_pos[1]:+.2f},{eef_pos[2]:+.2f})"
                    )
                else:
                    touch_str = "Touch: NOT CONNECTED"

                # 屏幕左上角 HUD
                hud_left = (
                    f"Chassis  vx={target_vx:+.2f} vy={target_vy:+.2f} w={target_omega:+.2f}\n"
                    f"Lift     target={target_lift:.2f} act={lift_actual:.2f} m\n"
                    f"Head     yaw={math.degrees(target_head_yaw):+.0f}  "
                    f"pitch={math.degrees(target_head_pitch):+.0f}  "
                    f"stem={target_head_stem:+.2f}\n"
                    f"\n"
                    f"Keys WASD QE GH IKJL TY\n"
                    f"     {indicator}\n"
                    f"\n"
                    f"{touch_str}\n"
                    f"\n"
                    f"Chassis ({pos[0]:+.2f},{pos[1]:+.2f}) yaw={yaw:+.0f}"
                )
                hud_right = (
                    f"Keyboard:\n"
                    f"W/S  forward/back\n"
                    f"A/D  strafe L/R\n"
                    f"Q/E  rotate L/R\n"
                    f"G/H  lift up/down\n"
                    f"I/K  head pitch\n"
                    f"J/L  head yaw\n"
                    f"T/Y  head stem up/dn\n"
                    f"R    reset\n"
                    f"ESC  quit\n"
                    f"\n"
                    f"Touch Pen:\n"
                    f"Btn2 white = calibrate\n"
                    f"Btn1 grey  = gripper\n"
                    f"\n"
                    f">>> Focus terminal\n"
                )
                viewer.set_texts([
                    (mujoco.mjtFontScale.mjFONTSCALE_150,
                     mujoco.mjtGridPos.mjGRID_TOPLEFT,
                     hud_left, None),
                    (mujoco.mjtFontScale.mjFONTSCALE_150,
                     mujoco.mjtGridPos.mjGRID_TOPRIGHT,
                     hud_right, None),
                ])

                last_print = sim_time

                # 终端日志降到 0.5s 一次
                if sim_time - last_log > 0.5:
                    print(f"t={sim_time:6.2f}  vx={target_vx:+.2f} vy={target_vy:+.2f} ω={target_omega:+.2f}  "
                          f"pos=({pos[0]:+.2f},{pos[1]:+.2f}) yaw={yaw:+.0f}°  [{indicator}]",
                          flush=True)
                    last_log = sim_time

            # 7. 实时同步
            elapsed = time.time() - step_start
            if elapsed < model.opt.timestep:
                time.sleep(model.opt.timestep - elapsed)

        print("\n=== 退出 ===")
        listener.stop()
        if head_cam_renderer is not None:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass


if __name__ == "__main__":
    main()
