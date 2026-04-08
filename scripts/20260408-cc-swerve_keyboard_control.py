#!/usr/bin/env python3
"""
四舵轮全向底盘 — MuJoCo 键盘 + Touch 笔遥操作

改进记录 (2026-04-08 夜间修复):
  1. 碰撞系统全面修复 — 地板/桌面/机械臂链路都参与碰撞
  2. 右臂改为力矩控制 + 任务空间阻抗控制 (对齐 airbot_force_sim.py)
  3. 力反馈：cfrc_ext → 低通滤波 → 坐标变换 → Touch 笔输出
  4. 夹爪：Button1 灰色按钮 toggle 开/合
  5. Reset 用 settle 方式避免物体飞走
  6. 升降行程扩大到 0~0.8m，速度加快
  7. 键盘响应改为直接赋值（无渐变延迟）

控制按键 (按住生效):
  底盘:   W/S 前后    A/D 左右    Q/E 转向
  升降:   G/H 升/降
  云台:   I/K 俯仰    J/L 偏航    T/Y 高度上下
  Touch:  Button2 白色 = 零点校准 + 跟随启动
          Button1 灰色 = 夹爪开/合
  其他:   R 重置      ESC 退出

运行:
  conda activate disc
  python 20260408-cc-swerve_keyboard_control.py
  # 禁用 Touch 笔: ENABLE_TOUCH=0 python ...
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

WHEEL_RADIUS = 0.06  # m

# 轮子在底盘坐标系下的位置
WHEEL_POSITIONS = np.array([
    [ 0.16,  0.15],  # fl
    [ 0.16, -0.15],  # fr
    [-0.16,  0.15],  # rl
    [-0.16, -0.15],  # rr
])

# 底盘速度上限
MAX_VX = 1.0       # m/s
MAX_VY = 1.0       # m/s
MAX_OMEGA = 2.0    # rad/s

# 升降参数（扩大行程 + 加快速度）
LIFT_MIN = 0.0
LIFT_MAX = 0.8     # 从 0.5 扩大到 0.8
LIFT_INIT = 0.0
LIFT_RATE = 1.0    # m/s 升降速度（从 0.6 提升到 1.0）

# 云台参数
HEAD_YAW_MIN = -1.5708
HEAD_YAW_MAX =  1.5708
HEAD_PITCH_MIN = -0.7854
HEAD_PITCH_MAX =  0.7854
HEAD_RATE = 2.5    # rad/s
HEAD_STEM_MIN = -0.05
HEAD_STEM_MAX =  0.40
HEAD_STEM_RATE = 0.30  # m/s

# 双臂 home pose：所有 6 个关节角归零
ARM_HOME_LEFT  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ARM_HOME_RIGHT = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# ============== 阻抗控制参数（对齐 airbot_force_sim.py）==============
# 位置刚度/阻尼 (对角阵)
KP_POS = np.diag([600.0, 600.0, 600.0])   # N/m
KD_POS = np.diag([40.0, 40.0, 40.0])      # Ns/m

# 姿态刚度/阻尼
KP_ROT = np.diag([30.0, 30.0, 30.0])      # Nm/rad
KD_ROT = np.diag([3.0, 3.0, 3.0])         # Nms/rad

# 零空间阻尼（防止冗余自由度漂移）
NULL_SPACE_DAMPING = 5.0

# Touch 笔 → 右臂映射
PEN_POS_SCALE = 0.002   # m / mm
PEN_ROT_SCALE = 1.0     # 姿态 1:1

# R_map: 笔坐标系 → 机器人 world 坐标系
R_MAP = np.array([
    [ 0,  0, -1],   # Robot X ← Touch -Z (笔向前推 → 机器人前进)
    [-1,  0,  0],   # Robot Y ← Touch -X (笔向右 → 机器人左)
    [ 0,  1,  0],   # Robot Z ← Touch Y  (笔向上 → 机器人上)
], dtype=float)

# 力反馈参数
FORCE_SCALE = 0.02      # N/N (仿真力 → 笔力)
MAX_PEN_FORCE = 3.0     # N (Touch 笔安全上限)
FORCE_FILTER_ALPHA = 0.3  # 低通滤波系数 (0~1, 越小越平滑)

# 夹爪参数
GRIPPER_OPEN = 0.0366   # 全开位置
GRIPPER_CLOSE = 0.0     # 全闭位置

# 底盘速度（改为直接赋值，无渐变延迟）
CHASSIS_SPEED = 0.6     # m/s 按键时直接达到的速度
CHASSIS_OMEGA = 1.5     # rad/s
DECEL_RATE = 10.0       # 松开后衰减率


# ============== 全局按键状态 ==============
key_held = {
    'w': False, 's': False,
    'a': False, 'd': False,
    'q': False, 'e': False,
    'g': False, 'h': False,
    'i': False, 'k': False,
    'j': False, 'l': False,
    't': False, 'y': False,
}
reset_request = False
exit_request = False
state_lock = threading.Lock()


def is_key_held(ch):
    with state_lock:
        return key_held.get(ch, False)


def disable_x11_repeat():
    """禁用 X11 键盘自动重复"""
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


atexit.register(restore_x11_repeat)


# ============== pynput 回调 ==============
def on_key_press(key):
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
    try:
        ch = key.char.lower() if hasattr(key, 'char') and key.char else ''
    except AttributeError:
        ch = ''
    with state_lock:
        if ch in key_held:
            key_held[ch] = False


# ============== 速度更新（直接赋值，无渐变）==============
target_vx = 0.0
target_vy = 0.0
target_omega = 0.0
target_lift = LIFT_INIT
target_head_yaw = 0.0
target_head_pitch = 0.0
target_head_stem = 0.0


def update_target_velocity(dt):
    """按键状态 → 目标速度。按下立即达到目标速度，松开快速衰减。"""
    global target_vx, target_vy, target_omega, target_lift
    global target_head_yaw, target_head_pitch, target_head_stem

    # 底盘：按下直接赋值，松开衰减
    w, s = is_key_held('w'), is_key_held('s')
    a, d = is_key_held('a'), is_key_held('d')
    q, e = is_key_held('q'), is_key_held('e')

    if w and not s:
        target_vx = CHASSIS_SPEED
    elif s and not w:
        target_vx = -CHASSIS_SPEED
    else:
        target_vx = target_vx * max(0.0, 1.0 - DECEL_RATE * dt)
        if abs(target_vx) < 0.01:
            target_vx = 0.0

    if a and not d:
        target_vy = CHASSIS_SPEED
    elif d and not a:
        target_vy = -CHASSIS_SPEED
    else:
        target_vy = target_vy * max(0.0, 1.0 - DECEL_RATE * dt)
        if abs(target_vy) < 0.01:
            target_vy = 0.0

    if q and not e:
        target_omega = CHASSIS_OMEGA
    elif e and not q:
        target_omega = -CHASSIS_OMEGA
    else:
        target_omega = target_omega * max(0.0, 1.0 - DECEL_RATE * dt)
        if abs(target_omega) < 0.01:
            target_omega = 0.0

    # 升降
    g, h = is_key_held('g'), is_key_held('h')
    if g and not h:
        target_lift = min(target_lift + LIFT_RATE * dt, LIFT_MAX)
    elif h and not g:
        target_lift = max(target_lift - LIFT_RATE * dt, LIFT_MIN)

    # 云台
    head_step = HEAD_RATE * dt
    ii, kk = is_key_held('i'), is_key_held('k')
    jj, ll = is_key_held('j'), is_key_held('l')
    t_up, y_dn = is_key_held('t'), is_key_held('y')

    if ii and not kk:
        target_head_pitch = max(target_head_pitch - head_step, HEAD_PITCH_MIN)
    elif kk and not ii:
        target_head_pitch = min(target_head_pitch + head_step, HEAD_PITCH_MAX)

    if jj and not ll:
        target_head_yaw = min(target_head_yaw + head_step, HEAD_YAW_MAX)
    elif ll and not jj:
        target_head_yaw = max(target_head_yaw - head_step, HEAD_YAW_MIN)

    stem_step = HEAD_STEM_RATE * dt
    if t_up and not y_dn:
        target_head_stem = min(target_head_stem + stem_step, HEAD_STEM_MAX)
    elif y_dn and not t_up:
        target_head_stem = max(target_head_stem - stem_step, HEAD_STEM_MIN)


# ============== 舵轮逆运动学 ==============
def swerve_ik(vx, vy, omega):
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
    diff = target - current
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    if abs(diff) > math.pi / 2:
        target = target - math.pi if diff > 0 else target + math.pi
        return target, True
    return target, False


# ============== 低通滤波器 ==============
class LowPassFilter:
    """简单一阶低通滤波"""
    def __init__(self, alpha=0.3, dim=3):
        self.alpha = alpha
        self.value = np.zeros(dim)
        self.initialized = False

    def filter(self, x):
        if not self.initialized:
            self.value = x.copy()
            self.initialized = True
        else:
            self.value = self.alpha * x + (1.0 - self.alpha) * self.value
        return self.value.copy()

    def reset(self):
        self.value[:] = 0.0
        self.initialized = False


# ============== Touch 笔遥控（阻抗控制 — 完全对齐 airbot_force_sim.py）==============

def _scale_rotation(R_delta, scale):
    """Rodrigues 旋转缩放"""
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
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    return np.eye(3) + math.sin(scaled) * K + (1.0 - math.cos(scaled)) * (K @ K)


class TouchTeleop:
    """Touch 力反馈笔遥控右臂 — 任务空间阻抗控制

    控制流程（完全对齐 airbot_force_sim.py）：
      1. Button2 (白色) = 校准：记录笔/机器人参考帧，重置机器人到 home
      2. Button1 (灰色) = 夹爪开/合 toggle
      3. 校准后持续跟随：
         - 笔增量位移 → workspace_center + R_map * delta * scale
         - 笔增量旋转 → R_map * delta_R * R_map^T * robot_R0
      4. 阻抗控制器：tau = J^T * [Kp*(x_des-x) - Kd*v] + qfrc_bias
      5. 力反馈：cfrc_ext → 低通滤波 → 坐标变换 → 笔力输出
    """

    def __init__(self, model, data):
        self.device = None
        self.connected = False
        self.model = model
        self.data = data

        # 右臂末端 site（用于精确 IK 和力反馈）
        self.eef_site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, "right_eef_site"
        )
        if self.eef_site_id < 0:
            raise RuntimeError("找不到 site: right_eef_site")

        # 右臂 6 个关节的索引信息
        self.n_arm = 6
        self.joint_ids = []
        self.qpos_indices = []
        self.qvel_indices = []
        for i in range(1, 7):
            jname = f"right_joint{i}"
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid < 0:
                raise RuntimeError(f"找不到关节: {jname}")
            self.joint_ids.append(jid)
            self.qpos_indices.append(model.jnt_qposadr[jid])
            self.qvel_indices.append(model.jnt_dofadr[jid])

        # 右臂末端 body id（用于 cfrc_ext 力读取）
        # 读取 link6 和两个夹爪指的接触力
        self.contact_body_ids = []
        for bname in ["right_link6", "right_g2_left", "right_g2_right"]:
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, bname)
            if bid >= 0:
                self.contact_body_ids.append(bid)

        # 总 DOF 数
        self.nv = model.nv

        # ---- 增量控制参考帧 ----
        self.pen_origin = None
        self.pen_R0 = None
        self.workspace_center = None
        self.robot_R0 = None
        self.desired_pos = None
        self.desired_quat = None

        # ---- 夹爪 ----
        self.gripper_target = GRIPPER_OPEN  # 初始张开

        # ---- 按钮状态 ----
        self.btn1_last = False
        self.btn2_last = False
        self.calibrated = False

        # ---- 力反馈 ----
        self.force_filter = LowPassFilter(alpha=FORCE_FILTER_ALPHA, dim=3)

        # ---- Jacobian buffer ----
        self.jac_pos = np.zeros((3, self.nv))
        self.jac_rot = np.zeros((3, self.nv))

    def connect(self):
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
            self.connected = False
            return False

    def disconnect(self):
        if self.connected and self.device:
            try:
                self.device.set_force(0, 0, 0)
                self.device.stop()
            except Exception:
                pass

    def get_eef_pos(self):
        """末端位置 (world frame)"""
        return self.data.site_xpos[self.eef_site_id].copy()

    def get_eef_rot(self):
        """末端旋转矩阵 (world frame)"""
        return self.data.site_xmat[self.eef_site_id].reshape(3, 3).copy()

    def get_arm_qpos(self):
        """右臂 6 个关节角"""
        return np.array([self.data.qpos[idx] for idx in self.qpos_indices])

    def get_arm_qvel(self):
        """右臂 6 个关节角速度"""
        return np.array([self.data.qvel[idx] for idx in self.qvel_indices])

    def get_contact_forces(self):
        """从 cfrc_ext 读取末端接触力 (world frame, N)"""
        force = np.zeros(3)
        for bid in self.contact_body_ids:
            # cfrc_ext[bid] = [tx, ty, tz, fx, fy, fz]
            force += self.data.cfrc_ext[bid, 3:6]
        return force

    def robot_force_to_pen(self, robot_force):
        """机器人末端力 → Touch 笔力（坐标变换 + 缩放 + 限幅）"""
        pen_force = np.array([
            FORCE_SCALE * (-robot_force[1]),   # -Fy_robot → Fx_pen
            FORCE_SCALE * robot_force[2],      # Fz_robot  → Fy_pen
            FORCE_SCALE * (-robot_force[0]),   # -Fx_robot → Fz_pen
        ])
        mag = np.linalg.norm(pen_force)
        if mag > MAX_PEN_FORCE:
            pen_force *= MAX_PEN_FORCE / mag
        return pen_force

    def _calibrate(self, pen_pos, pen_R):
        """白色按钮校准：重置右臂到 home，记录参考帧"""
        # 1. 右臂关节归零（通过直接设置 qpos）
        for idx in self.qpos_indices:
            self.data.qpos[idx] = 0.0
        for idx in self.qvel_indices:
            self.data.qvel[idx] = 0.0
        mujoco.mj_forward(self.model, self.data)

        # 2. 记录 home 状态下的末端位置和姿态
        self.workspace_center = self.get_eef_pos()
        self.desired_pos = self.workspace_center.copy()
        self.robot_R0 = self.get_eef_rot()

        # 转换为 MuJoCo 四元数
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, self.robot_R0.flatten())
        self.desired_quat = quat.copy()

        # 3. 记录笔参考
        self.pen_origin = pen_pos.copy()
        if pen_R is not None and abs(np.linalg.det(pen_R) - 1.0) < 0.2:
            self.pen_R0 = pen_R.copy()

        # 4. 重置力反馈
        self.force_filter.reset()

        self.calibrated = True
        print(f">>> Touch 校准完成")
        print(f"    workspace_center = {np.round(self.workspace_center, 3)}")
        print(f"    pen_origin = {np.round(self.pen_origin, 1)} mm")

    def compute_impedance_control(self):
        """任务空间阻抗控制器 — 返回右臂 6 个关节的力矩

        算法（对齐 airbot_force_sim.py）：
          tau = Jp^T * F_pos + Jr^T * T_rot + qfrc_bias - null_space_damping * qvel
        """
        n = self.n_arm

        if self.desired_pos is None:
            # 未校准：只做重力补偿
            return np.array([self.data.qfrc_bias[idx] for idx in self.qvel_indices])

        # 1. 当前末端状态
        pos_cur = self.get_eef_pos()
        R_cur = self.get_eef_rot()
        qvel = self.get_arm_qvel()

        # 2. 雅可比矩阵 (3 × nv)
        self.jac_pos[:] = 0.0
        self.jac_rot[:] = 0.0
        mujoco.mj_jacSite(
            self.model, self.data,
            self.jac_pos, self.jac_rot, self.eef_site_id
        )
        # 提取右臂部分
        Jp = self.jac_pos[:, self.qvel_indices]  # 3×6
        Jr = self.jac_rot[:, self.qvel_indices]  # 3×6

        # 3. 位置误差力
        pos_err = self.desired_pos - pos_cur
        vel_cur = Jp @ qvel
        F_pos = KP_POS @ pos_err - KD_POS @ vel_cur

        # 4. 姿态误差力矩
        rot_err = np.zeros(3)
        if self.desired_quat is not None:
            quat_cur = np.zeros(4)
            mujoco.mju_mat2Quat(quat_cur, R_cur.flatten())
            mujoco.mju_subQuat(rot_err, self.desired_quat, quat_cur)
        omega_cur = Jr @ qvel
        T_rot = KP_ROT @ rot_err - KD_ROT @ omega_cur

        # 5. 关节力矩 = J^T * [F; T] + 重力补偿
        tau = Jp.T @ F_pos + Jr.T @ T_rot

        # 加上重力/科里奥利补偿
        for i, dof_idx in enumerate(self.qvel_indices):
            tau[i] += self.data.qfrc_bias[dof_idx]

        # 6. 零空间阻尼
        tau -= NULL_SPACE_DAMPING * qvel

        return tau

    def update(self):
        """每步调用：读取 Touch 笔 → 更新目标 → 计算力矩 → 写入 ctrl + 力反馈"""
        if not self.connected:
            # 未连接时用重力补偿保持姿态
            tau = self.compute_impedance_control()
            for i in range(6):
                self.data.ctrl[15 + i] = tau[i]
            return

        state = self.device.get_state()
        pen_pos = np.array(state.position)  # mm

        # 读取笔 transform → 旋转矩阵
        pen_R = None
        if state.transform and len(state.transform) == 16:
            T_raw = np.array(state.transform).reshape(4, 4)
            R_candidate = T_raw.T[:3, :3]
            if abs(np.linalg.det(R_candidate) - 1.0) < 0.2:
                pen_R = R_candidate

        btn1 = bool(state.button1)
        btn2 = bool(state.button2)

        # Button2 上升沿：零点校准
        if btn2 and not self.btn2_last:
            self._calibrate(pen_pos, pen_R)
        self.btn2_last = btn2

        # Button1 上升沿：夹爪 toggle
        if btn1 and not self.btn1_last:
            if self.gripper_target > 0.018:
                self.gripper_target = GRIPPER_CLOSE
                print(">>> [夹爪] 闭合")
            else:
                self.gripper_target = GRIPPER_OPEN
                print(">>> [夹爪] 张开")
        self.btn1_last = btn1

        # 夹爪执行器 (ctrl[24])
        self.data.ctrl[24] = self.gripper_target

        if not self.calibrated:
            tau = self.compute_impedance_control()
            for i in range(6):
                self.data.ctrl[15 + i] = tau[i]
            return

        # ========== 位置跟随 ==========
        delta_pen_mm = pen_pos - self.pen_origin
        delta_world = R_MAP @ (delta_pen_mm * PEN_POS_SCALE)
        self.desired_pos = self.workspace_center + delta_world

        # ========== 姿态跟随 ==========
        if pen_R is not None and self.pen_R0 is not None:
            delta_R_pen = pen_R @ self.pen_R0.T
            if PEN_ROT_SCALE != 1.0:
                delta_R_pen = _scale_rotation(delta_R_pen, PEN_ROT_SCALE)
            delta_R_robot = R_MAP @ delta_R_pen @ R_MAP.T
            R_robot_des = delta_R_robot @ self.robot_R0
            # SVD 正交化
            U, _, Vt = np.linalg.svd(R_robot_des)
            R_robot_des = U @ Vt
            quat_new = np.zeros(4)
            mujoco.mju_mat2Quat(quat_new, R_robot_des.flatten())
            self.desired_quat = quat_new

        # ========== 阻抗控制 → 关节力矩 ==========
        tau = self.compute_impedance_control()
        for i in range(6):
            self.data.ctrl[15 + i] = tau[i]

        # ========== 力反馈 → Touch 笔 ==========
        contact_force = self.get_contact_forces()
        smoothed = self.force_filter.filter(contact_force)
        pen_force = self.robot_force_to_pen(smoothed)
        try:
            self.device.set_force(pen_force[0], pen_force[1], pen_force[2])
        except Exception:
            pass

    def reset(self):
        """重置所有状态"""
        self.calibrated = False
        self.pen_origin = None
        self.pen_R0 = None
        self.desired_pos = None
        self.desired_quat = None
        self.workspace_center = None
        self.robot_R0 = None
        self.gripper_target = GRIPPER_OPEN
        self.force_filter.reset()
        if self.connected:
            try:
                self.device.set_force(0, 0, 0)
            except Exception:
                pass


# ============== 主函数 ==============

def main():
    global reset_request, target_vx, target_vy, target_omega, target_lift
    global target_head_yaw, target_head_pitch, target_head_stem

    disable_x11_repeat()

    print("加载 MJCF:", MJCF_PATH)
    model = mujoco.MjModel.from_xml_path(MJCF_PATH)
    data = mujoco.MjData(model)
    print(f"模型: nq={model.nq}, nv={model.nv}, nu={model.nu}")
    print()
    print("=" * 64)
    print("        四舵轮全向底盘 — 键盘 + Touch 笔遥操作")
    print("=" * 64)
    print()
    print("【重要】请把焦点放在【本终端窗口】")
    print()
    print("控制按键 (按住生效)：")
    print("  底盘:   W/S 前后    A/D 左右    Q/E 转向")
    print("  升降:   G/H 升/降 (0~0.8m)")
    print("  云台:   I/K 俯仰    J/L 偏航    T/Y 高度上下")
    print("  Touch:  Button2 白色 = 零点校准 + 跟随")
    print("          Button1 灰色 = 夹爪开/合")
    print("  其他:   R 重置      ESC 退出")
    print("=" * 64)
    print()

    # 启动键盘监听
    listener = keyboard.Listener(on_press=on_key_press, on_release=on_key_release)
    listener.start()
    print(">>> pynput 键盘监听已启动")

    current_steer = np.zeros(4)
    chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis_link")

    # 初始化 ctrl
    mujoco.mj_forward(model, data)
    data.ctrl[:] = 0.0
    data.ctrl[8] = LIFT_INIT              # lift
    data.ctrl[9:15] = ARM_HOME_LEFT       # left arm (position control)
    data.ctrl[24] = GRIPPER_OPEN          # gripper

    # Touch 笔
    touch = TouchTeleop(model, data)
    touch.connect()
    atexit.register(touch.disconnect)

    # 头部相机子窗口
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

    # 让物体先 settle（避免 reset 后飞走）
    print(">>> 物体 settling (0.5s)...")
    for _ in range(250):  # 0.5s @ 0.002s timestep
        data.ctrl[9:15] = ARM_HOME_LEFT
        # 右臂重力补偿
        tau_init = touch.compute_impedance_control()
        for i in range(6):
            data.ctrl[15 + i] = tau_init[i]
        data.ctrl[24] = GRIPPER_OPEN
        mujoco.mj_step(model, data)
    print(">>> Settling 完成")

    # 记录 settle 后的初始状态用于 reset
    init_qpos = data.qpos.copy()
    init_qvel = data.qvel.copy()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 3.0
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25
        viewer.cam.lookat[:] = [0, 0, 0.5]

        last_print = 0.0
        last_log = 0.0

        while viewer.is_running() and not exit_request:
            step_start = time.time()
            sim_time = data.time

            # ===== 重置 =====
            if reset_request:
                # 用 settle 后的初始状态恢复（物体不飞）
                data.qpos[:] = init_qpos
                data.qvel[:] = init_qvel
                data.ctrl[:] = 0.0
                data.ctrl[8] = LIFT_INIT
                data.ctrl[9:15] = ARM_HOME_LEFT
                data.ctrl[24] = GRIPPER_OPEN

                target_vx = target_vy = target_omega = 0.0
                target_lift = LIFT_INIT
                target_head_yaw = target_head_pitch = 0.0
                target_head_stem = 0.0
                current_steer[:] = 0.0
                touch.reset()

                mujoco.mj_forward(model, data)
                reset_request = False
                print("\n>>> RESET (settle state) <<<\n")

            # 1. 更新速度
            update_target_velocity(model.opt.timestep)

            # 2. 舵轮 IK
            steer_angles, drive_speeds = swerve_ik(
                target_vx, target_vy, target_omega
            )
            for i in range(4):
                if abs(drive_speeds[i]) > 1e-4:
                    steer_angles[i], rev = shortest_steer(
                        steer_angles[i], current_steer[i]
                    )
                    if rev:
                        drive_speeds[i] = -drive_speeds[i]
                    current_steer[i] = steer_angles[i]

            # 3. 写入执行器
            data.ctrl[0:4] = steer_angles
            data.ctrl[4:8] = drive_speeds
            data.ctrl[8] = target_lift

            # 左臂保持 home
            data.ctrl[9:15] = ARM_HOME_LEFT

            # 右臂 + 力反馈 + 夹爪 (由 touch.update() 写入 ctrl[15:21] 和 ctrl[24])
            touch.update()

            # 云台
            data.ctrl[21] = target_head_yaw
            data.ctrl[22] = target_head_pitch
            data.ctrl[23] = target_head_stem

            # 4. 物理步进
            mujoco.mj_step(model, data)
            viewer.sync()

            # 5. Head Camera 子窗口 (30 Hz)
            if head_cam_renderer is not None and head_cam_id >= 0:
                if sim_time - head_cam_last_update > 1.0 / 30.0:
                    try:
                        head_cam_renderer.update_scene(data, camera=head_cam_id)
                        frame = head_cam_renderer.render()
                        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                        cv2.imshow('Head Camera (First Person)', bgr)
                        cv2.waitKey(1)
                    except Exception:
                        pass
                    head_cam_last_update = sim_time

            # 6. HUD (0.1s 间隔)
            if sim_time - last_print > 0.1:
                pos = data.xpos[chassis_id]
                qw, qz = data.xquat[chassis_id, 0], data.xquat[chassis_id, 3]
                yaw = math.degrees(2.0 * math.atan2(qz, qw))

                lift_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "lift_joint")
                lift_actual = data.qpos[model.jnt_qposadr[lift_jid]]

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

                # Touch 状态
                if touch.connected:
                    t_state = "CALIBRATED" if touch.calibrated else "WAIT BTN2"
                    eef = touch.get_eef_pos()
                    grip = "OPEN" if touch.gripper_target > 0.018 else "CLOSED"
                    touch_str = (
                        f"Touch: {t_state}  Grip: {grip}\n"
                        f"R-EEF = ({eef[0]:+.2f},{eef[1]:+.2f},{eef[2]:+.2f})"
                    )
                else:
                    touch_str = "Touch: NOT CONNECTED"

                hud_left = (
                    f"Chassis  vx={target_vx:+.2f} vy={target_vy:+.2f} w={target_omega:+.2f}\n"
                    f"Lift     target={target_lift:.2f} act={lift_actual:.2f} m\n"
                    f"Head     yaw={math.degrees(target_head_yaw):+.0f}  "
                    f"pitch={math.degrees(target_head_pitch):+.0f}  "
                    f"stem={target_head_stem:+.2f}\n"
                    f"\n"
                    f"Keys [{indicator}]\n"
                    f"\n"
                    f"{touch_str}\n"
                    f"Pos ({pos[0]:+.2f},{pos[1]:+.2f}) yaw={yaw:+.0f}"
                )

                hud_right = (
                    f"W/S forward/back\n"
                    f"A/D strafe L/R\n"
                    f"Q/E rotate L/R\n"
                    f"G/H lift (0~0.8m)\n"
                    f"I/K head pitch\n"
                    f"J/L head yaw\n"
                    f"T/Y head stem\n"
                    f"R   reset\n"
                    f"ESC quit\n"
                    f"\n"
                    f"Touch Pen:\n"
                    f"Btn2 = calibrate\n"
                    f"Btn1 = gripper\n"
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
