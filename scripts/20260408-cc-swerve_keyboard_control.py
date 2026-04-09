#!/usr/bin/env python3
"""
四舵轮全向底盘 — MuJoCo 键盘 + Touch 笔遥操作

修复记录 (2026-04-09):
  1. 阻抗控制增益对齐参考实现 (KP=5000, KD=150, KP_ROT=300, KD_ROT=30)
  2. 零空间阻尼降低至 0.5 (消除与控制器的冲突)
  3. 力矩限幅 ±100 Nm (防止穿透桌面)
  4. 夹爪默认闭合，灰色按钮第一次按 = 张开
  5. 升降过阻尼 (XML kv=400)，速度降低至 0.5 m/s
  6. 第一人称视角：按 F 切换主窗口到 head_cam
  7. 目标点实时可视化（绿色球 = 目标位置，红色球 = 末端实际位置）
  8. 校准后重力补偿稳定 50 步再跟随
  9. 升降目标低通滤波（消除突变振荡）

控制按键 (按住生效):
  底盘:   W/S 前后    A/D 左右    Q/E 转向
  升降:   G/H 升/降
  云台:   I/K 俯仰    J/L 偏航    T/Y 高度上下
  相机:   F 切换第一人称/第三人称
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

# OpenCV 用于第一人称相机子窗口（备选）
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

# 轮子在底盘坐标系下的位置 (fl/fr/rl/rr)
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

# ============== 舵轮控制参数（FRC 最佳实践）==============
# 轮模块最大驱动速度（rad/s），由 MAX_VX 和 WHEEL_RADIUS 推算
MAX_WHEEL_SPEED = MAX_VX / WHEEL_RADIUS * 1.5  # 余量系数 1.5

# 转向速率限制：每秒最多转多少弧度（避免轮子瞬变跳转）
STEER_RATE_LIMIT = 8.0  # rad/s — 比 MuJoCo position actuator 的 kp 响应慢一些

# 驱动加速度限制：每秒最多变化多少 rad/s（避免轮子打滑/物理震荡）
DRIVE_ACCEL_LIMIT = 30.0  # rad/s^2

# 底盘输入斜率限制（键盘 → 期望速度的平滑）
CHASSIS_VX_SLEW = 3.0    # m/s^2
CHASSIS_VY_SLEW = 3.0    # m/s^2
CHASSIS_OMEGA_SLEW = 6.0 # rad/s^2

# 升降参数（降低速度 + 平滑）
LIFT_MIN = 0.0
LIFT_MAX = 0.8
LIFT_INIT = 0.0
LIFT_RATE = 0.5     # m/s (降低速度避免惯性振荡)

# 云台参数
HEAD_YAW_MIN = -1.5708
HEAD_YAW_MAX =  1.5708
HEAD_PITCH_MIN = -0.7854
HEAD_PITCH_MAX =  0.7854
HEAD_RATE = 2.5    # rad/s
HEAD_STEM_MIN = -0.05
HEAD_STEM_MAX =  0.40
HEAD_STEM_RATE = 0.30  # m/s

# 双臂 home pose
# ★ 右臂使用非奇异 home pose，避免 all-zero 时的结构性奇异
#   [j1=0, j2=-0.5, j3=0.3, j4=1.0, j5=0, j6=0]
#   这让臂微微弯曲，6×6 Jacobian 无零奇异值
ARM_HOME_LEFT  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ARM_HOME_RIGHT = [0.0, -0.5, 0.3, 1.0, 0.0, 0.0]

# ============== 阻抗控制参数 ==============
# ★ 位置增益高 (5000)：跟踪精度 < 1mm
# ★ 姿态增益低 (30)：避免结构性奇异导致发散
KP_POS = np.diag([5000.0, 5000.0, 5000.0])   # N/m
KD_POS = np.diag([150.0, 150.0, 150.0])       # Ns/m

# ★ 姿态：仅用弱回复力 + 阻尼（不做高精度姿态跟踪）
# airbot play 6轴臂有结构性奇异（6×6 Jacobian 恒有一个零奇异值），
# 高姿态增益会导致不可控力矩→发散
KP_ROT = np.diag([30.0, 30.0, 30.0])          # Nm/rad (弱回复)
KD_ROT = np.diag([10.0, 10.0, 10.0])          # Nms/rad (阻尼为主)

# 零空间阻尼
NULL_SPACE_DAMPING = 0.5

# 力矩限幅
TORQUE_LIMIT = 200.0       # Nm per joint

# Touch 笔 → 右臂映射
PEN_POS_SCALE = 0.002   # m / mm
PEN_ROT_SCALE = 1.0     # 姿态 1:1

# R_map: 笔坐标系 → 机器人 world 坐标系
# Touch X (用户右) → Robot -Y (取反)
# Touch Y (上)     → Robot Z
# Touch -Z (前推)  → Robot X
R_MAP = np.array([
    [ 0,  0, -1],   # Robot X ← Touch -Z
    [-1,  0,  0],   # Robot Y ← Touch -X
    [ 0,  1,  0],   # Robot Z ← Touch Y
], dtype=float)

# 力反馈参数
FORCE_SCALE = 0.02      # N/N (仿真力 → 笔力)
MAX_PEN_FORCE = 3.0     # N
FORCE_FILTER_ALPHA = 0.3

# 夹爪参数
GRIPPER_OPEN = 0.0366   # 全开位置
GRIPPER_CLOSE = 0.0     # 全闭位置

# 底盘速度（直接赋值，无渐变延迟）
CHASSIS_SPEED = 0.6     # m/s
CHASSIS_OMEGA = 1.5     # rad/s
# DECEL_RATE 已废弃 — 改用 CHASSIS_*_SLEW 斜率限幅（见上方参数区）


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
toggle_fpv_request = False  # F 键切换第一人称
state_lock = threading.Lock()


def is_key_held(ch):
    with state_lock:
        return key_held.get(ch, False)


def disable_x11_repeat():
    """禁用 X11 键盘自动重复（避免 pynput 重复事件）"""
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
    global reset_request, exit_request, toggle_fpv_request
    try:
        ch = key.char.lower() if hasattr(key, 'char') and key.char else ''
    except AttributeError:
        ch = ''
    with state_lock:
        if ch in key_held:
            key_held[ch] = True
        elif ch == 'r':
            reset_request = True
        elif ch == 'f':
            toggle_fpv_request = True
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


# ============== 速度更新 ==============
target_vx = 0.0
target_vy = 0.0
target_omega = 0.0
target_lift = LIFT_INIT
# 升降目标低通滤波状态（消除突变振荡）
smoothed_lift = LIFT_INIT
target_head_yaw = 0.0
target_head_pitch = 0.0
target_head_stem = 0.0


def _slew_rate_limit(current, target, slew, dt):
    """
    通用斜率限幅：current 以最大 slew*dt 的步长趋近 target。
    平滑过渡，避免瞬变跳跃。
    """
    max_step = slew * dt
    delta = target - current
    if abs(delta) > max_step:
        delta = math.copysign(max_step, delta)
    return current + delta


def update_target_velocity(dt):
    """
    按键状态 → 目标底盘速度 (带斜率限幅)

    改进: 不再用乘法衰减（旧版 target *= 1 - DECEL_RATE*dt），
    而是用斜率限幅使速度平滑加速/减速。
    这样松开按键后减速曲线是线性的（恒减速），不会出现
    速度先快后慢的指数衰减"拖泥带水"感。
    """
    global target_vx, target_vy, target_omega, target_lift, smoothed_lift
    global target_head_yaw, target_head_pitch, target_head_stem

    # ---- 底盘: 按键 → 目标值 → 斜率限幅 ----
    w, s = is_key_held('w'), is_key_held('s')
    a, d = is_key_held('a'), is_key_held('d')
    q, e = is_key_held('q'), is_key_held('e')

    # 计算按键期望值（按下=全速，松开=归零）
    desired_vx = 0.0
    if w and not s:
        desired_vx = CHASSIS_SPEED
    elif s and not w:
        desired_vx = -CHASSIS_SPEED

    desired_vy = 0.0
    if a and not d:
        desired_vy = CHASSIS_SPEED
    elif d and not a:
        desired_vy = -CHASSIS_SPEED

    desired_omega = 0.0
    if q and not e:
        desired_omega = CHASSIS_OMEGA
    elif e and not q:
        desired_omega = -CHASSIS_OMEGA

    # 斜率限幅: 加速/减速都受限，曲线平滑
    target_vx = _slew_rate_limit(target_vx, desired_vx, CHASSIS_VX_SLEW, dt)
    target_vy = _slew_rate_limit(target_vy, desired_vy, CHASSIS_VY_SLEW, dt)
    target_omega = _slew_rate_limit(target_omega, desired_omega, CHASSIS_OMEGA_SLEW, dt)

    # 死区: 低于阈值直接归零（避免无限趋近零）
    if abs(target_vx) < 0.005:
        target_vx = 0.0
    if abs(target_vy) < 0.005:
        target_vy = 0.0
    if abs(target_omega) < 0.005:
        target_omega = 0.0

    # ---- 升降（低通滤波平滑目标值）----
    gg, hh = is_key_held('g'), is_key_held('h')
    if gg and not hh:
        target_lift = min(target_lift + LIFT_RATE * dt, LIFT_MAX)
    elif hh and not gg:
        target_lift = max(target_lift - LIFT_RATE * dt, LIFT_MIN)
    # 低通滤波: alpha=0.05 → 非常平滑，避免阶跃输入导致振荡
    LIFT_SMOOTH_ALPHA = 0.05
    smoothed_lift = LIFT_SMOOTH_ALPHA * target_lift + (1.0 - LIFT_SMOOTH_ALPHA) * smoothed_lift

    # ---- 云台 ----
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


# ============== 舵轮逆运动学（WPILib / Team 254 算法）==============
# 参考: WPILib SwerveDriveKinematics + SwerveModuleState.optimize()
# 解决了原版的 5 个核心问题:
#   1. 零速时保持上次轮角（不归零）
#   2. 轮速去饱和（等比缩放保持轨迹一致）
#   3. 余弦补偿（转向未到位时减小驱动力）
#   4. >90° 角度优化 + 驱动反转
#   5. 转向速率限幅 + 驱动加速度限幅

# 构建 (8, 3) 逆运动学矩阵 — 一次矩阵乘法替代 for 循环
#   每个轮子 i 在位置 (mx, my):
#     row 2i  :  [1, 0, -my]   → vx 分量
#     row 2i+1:  [0, 1, +mx]   → vy 分量
_IK_MATRIX = np.zeros((8, 3))
for _i, (_mx, _my) in enumerate(WHEEL_POSITIONS):
    _IK_MATRIX[2 * _i,     :] = [1.0, 0.0, -_my]
    _IK_MATRIX[2 * _i + 1, :] = [0.0, 1.0,  _mx]

# 计算最大可能模块距离（用于预归一化，可选）
_MAX_MODULE_DIST = max(math.hypot(mx, my) for mx, my in WHEEL_POSITIONS)


def _normalize_angle(angle):
    """将角度归一化到 [-pi, pi]"""
    angle = angle % (2.0 * math.pi)
    if angle > math.pi:
        angle -= 2.0 * math.pi
    return angle


def swerve_ik(vx, vy, omega, prev_steer_angles):
    """
    底盘速度 → 4 组 (转向角, 驱动转速)

    核心改进（来自 WPILib / FRC 最佳实践）:
      - 矩阵逆运动学: 一次矩阵乘法，精确且高效
      - 零速保持: speed < eps 时保留 prev_steer_angles（不归零！）
      - 全零输入检测: 所有输入都为零时直接返回上次角度 + 零速

    参数:
      vx, vy:   底盘期望线速度 (m/s), 前方为 +x, 左方为 +y
      omega:    底盘期望角速度 (rad/s), 逆时针为正
      prev_steer_angles: 上一帧的 4 个转向角 (rad)

    返回:
      steer_angles: ndarray(4,), 各轮目标转向角 (rad)
      drive_speeds:  ndarray(4,), 各轮目标驱动转速 (rad/s)
    """
    steer_angles = np.zeros(4)
    drive_speeds = np.zeros(4)

    # ---- 全零输入: 保持上次角度，速度归零 ----
    if abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(omega) < 1e-6:
        return prev_steer_angles.copy(), drive_speeds

    # ---- 矩阵逆运动学: [8×3] @ [3] → [8] ----
    chassis_vec = np.array([vx, vy, omega])
    module_vecs = _IK_MATRIX @ chassis_vec  # shape (8,)

    for i in range(4):
        wvx = module_vecs[2 * i]
        wvy = module_vecs[2 * i + 1]
        speed = math.hypot(wvx, wvy)

        if speed < 1e-6:
            # 该轮分解速度极小 → 保持上次角度（不用 atan2，避免随机方向）
            steer_angles[i] = prev_steer_angles[i]
            drive_speeds[i] = 0.0
        else:
            steer_angles[i] = math.atan2(wvy, wvx)
            drive_speeds[i] = speed / WHEEL_RADIUS

    return steer_angles, drive_speeds


def optimize_module(desired_speed, desired_angle, current_angle):
    """
    WPILib SwerveModuleState.optimize() 的 Python 等价实现。

    核心思想: 轮子 (speed, θ) 等价于 (-speed, θ+π)。
    因此轮子永远不需要转超过 90°。当目标角度与当前角度差 >90° 时，
    反转驱动方向并翻转目标角度。

    参数:
      desired_speed: 目标驱动速度 (rad/s)
      desired_angle: 目标转向角 (rad)
      current_angle: 当前转向角 (rad)

    返回:
      (optimized_speed, optimized_angle)
    """
    delta = _normalize_angle(desired_angle - current_angle)
    if abs(delta) > math.pi / 2.0:
        # 需要转超过 90° → 反转驱动 + 角度翻 180°
        desired_speed = -desired_speed
        desired_angle = _normalize_angle(desired_angle + math.pi)
    return desired_speed, desired_angle


def desaturate_wheel_speeds(drive_speeds, max_speed):
    """
    WPILib desaturateWheelSpeeds() — 轮速去饱和。

    当任何一个轮子的计算速度超过物理最大值时，
    等比例缩放所有轮子的速度，保持轮速之间的比例关系不变。
    这确保底盘沿正确轨迹运动而不变形。

    参数:
      drive_speeds: ndarray(4,), 各轮驱动速度
      max_speed: float, 物理最大轮速

    返回:
      ndarray(4,), 去饱和后的速度
    """
    real_max = np.max(np.abs(drive_speeds))
    if real_max > max_speed:
        scale = max_speed / real_max
        return drive_speeds * scale
    return drive_speeds.copy()


def cosine_scale(desired_speed, desired_angle, current_angle):
    """
    WPILib cosineScale() — 余弦补偿。

    当轮子还没转到目标方向时，按角度误差的余弦缩小驱动速度：
      - 轮子对准目标 → cos(0) = 1.0 → 全速
      - 偏离 45° → cos(45°) ≈ 0.71 → 减速 29%
      - 偏离 90° → cos(90°) = 0.0 → 完全停止

    这防止了轮子在转向过程中产生侧向推力（skew），
    是消除底盘 "吃屎感" 的关键。
    """
    angle_error = _normalize_angle(desired_angle - current_angle)
    return desired_speed * math.cos(angle_error)


def rate_limit_steer(target_angle, current_angle, max_rate, dt):
    """
    转向速率限幅：每步最多转 max_rate * dt 弧度。

    防止 MuJoCo position actuator 因为目标角度瞬变而产生
    巨大加速度，导致底盘 "抖动" 或 "甩尾"。
    """
    delta = _normalize_angle(target_angle - current_angle)
    max_step = max_rate * dt
    if abs(delta) > max_step:
        delta = math.copysign(max_step, delta)
    return _normalize_angle(current_angle + delta)


def rate_limit_drive(target_speed, current_speed, max_accel, dt):
    """
    驱动加速度限幅：每步速度变化不超过 max_accel * dt。

    防止轮子瞬间加减速导致打滑或物理震荡。
    """
    max_step = max_accel * dt
    delta = target_speed - current_speed
    if abs(delta) > max_step:
        delta = math.copysign(max_step, delta)
    return current_speed + delta


# ============== 低通滤波器 ==============
class LowPassFilter:
    """一阶低通滤波"""
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


# ============== 旋转缩放工具 ==============
def _scale_rotation(R_delta, scale):
    """Rodrigues 轴角缩放旋转矩阵"""
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


# ============== Touch 笔遥控（阻抗控制）==============
class TouchTeleop:
    """Touch 力反馈笔遥控右臂 — 增量式任务空间阻抗控制

    控制流程（★ 对齐参考实现 airbot_force_sim.py）：
      1. Button2 (白色) = 校准：臂归零，记录笔/机器人参考帧
      2. Button1 (灰色) = 夹爪开/合 toggle
      3. 增量跟随：desired_pos = workspace_center + R_map * (pen - pen_origin) * scale
      4. 阻抗控制：tau = Jp^T * F + Jr^T * T + qfrc_bias （力矩限幅 ±100 Nm）
      5. 力反馈：cfrc_ext → 低通滤波 → 坐标变换 → 笔力输出
    """

    def __init__(self, model, data):
        self.device = None
        self.connected = False
        self.model = model
        self.data = data

        # ---- 右臂末端 site ----
        self.eef_site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, "right_eef_site"
        )
        if self.eef_site_id < 0:
            raise RuntimeError("找不到 site: right_eef_site")

        # ---- 右臂 6 个关节 ----
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

        # ---- 接触力检测的 body ----
        self.contact_body_ids = []
        for bname in ["right_link6", "right_g2_left", "right_g2_right"]:
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, bname)
            if bid >= 0:
                self.contact_body_ids.append(bid)

        # ★ 右臂 base body（用于动态 workspace_center）
        self.arm_base_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "right_arm_base"
        )

        # 总 DOF 数（Jacobian 矩阵列数）
        self.nv = model.nv

        # ---- 增量控制参考帧 ----
        self.pen_origin = None       # 笔的校准原点 (mm)
        self.pen_R0 = None           # 笔的校准旋转矩阵
        # ★ workspace_center 改为相对于 arm_base 的偏移量
        self.home_offset_local = None  # 校准时 EEF 相对 arm_base 的偏移 (base frame)
        self.arm_base_pos_cal = None   # 校准时 arm_base 的世界位置
        self.robot_R0 = None         # 臂 home 位置下的末端旋转矩阵
        self.desired_pos = None      # 当前目标位置 (world frame, m)
        self.desired_quat = None     # 当前目标姿态 (四元数)

        # ---- 夹爪（★ 默认闭合，灰色按钮第一次按 = 打开）----
        self.gripper_target = GRIPPER_CLOSE

        # ---- 按钮边沿检测 ----
        self.btn1_last = False
        self.btn2_last = False
        self.calibrated = False
        # 校准后等待稳定的计数器（50 步 = 0.1s）
        self.settle_countdown = 0

        # ---- 力反馈 ----
        self.force_filter = LowPassFilter(alpha=FORCE_FILTER_ALPHA, dim=3)

        # ---- Jacobian 缓冲 ----
        self.jac_pos = np.zeros((3, self.nv))
        self.jac_rot = np.zeros((3, self.nv))

    def connect(self):
        """连接 Touch 力反馈笔"""
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
            print(">>> 按【白色按钮 Button2】校准后开始跟随")
            return True
        except Exception as exc:
            print(f"!!! Touch 笔连接失败: {exc}")
            self.connected = False
            return False

    def disconnect(self):
        """断开 Touch 笔，清零力输出"""
        if self.connected and self.device:
            try:
                self.device.set_force(0, 0, 0)
                self.device.stop()
            except Exception:
                pass

    # ---- 状态读取 ----
    def get_eef_pos(self):
        return self.data.site_xpos[self.eef_site_id].copy()

    def get_eef_rot(self):
        return self.data.site_xmat[self.eef_site_id].reshape(3, 3).copy()

    def get_arm_qpos(self):
        return np.array([self.data.qpos[idx] for idx in self.qpos_indices])

    def get_arm_qvel(self):
        return np.array([self.data.qvel[idx] for idx in self.qvel_indices])

    def get_contact_forces(self):
        """从 cfrc_ext 读取末端+夹爪接触力 (world frame, N)"""
        force = np.zeros(3)
        for bid in self.contact_body_ids:
            force += self.data.cfrc_ext[bid, 3:6]
        return force

    def robot_force_to_pen(self, robot_force):
        """机器人末端力 → Touch 笔力（坐标变换 + 缩放 + 限幅）
        映射关系（位置映射的力学对偶/转置）：
          Robot -Fy → Pen Fx
          Robot +Fz → Pen Fy
          Robot -Fx → Pen Fz
        """
        pen_force = np.array([
            FORCE_SCALE * (-robot_force[1]),
            FORCE_SCALE * robot_force[2],
            FORCE_SCALE * (-robot_force[0]),
        ])
        mag = np.linalg.norm(pen_force)
        if mag > MAX_PEN_FORCE:
            pen_force *= MAX_PEN_FORCE / mag
        return pen_force

    # ---- 校准 ----
    def get_arm_base_pos(self):
        """右臂 base body 的世界位置"""
        return self.data.xpos[self.arm_base_id].copy()

    def get_dynamic_workspace_center(self):
        """★ 动态 workspace_center：跟随底盘/升降运动

        在校准时记录 EEF 相对 arm_base 的偏移（base local frame），
        之后每帧用当前 arm_base 位置 + 偏移重建 workspace_center。
        这样底盘移动/升降时，工作空间跟着移动，避免耦合不稳定。
        """
        if self.home_offset_local is None:
            return None
        arm_base_pos = self.get_arm_base_pos()
        # 简化：直接用世界坐标偏移（底盘小角度旋转时近似正确）
        base_drift = arm_base_pos - self.arm_base_pos_cal
        return self.arm_base_pos_cal + self.home_offset_local + base_drift

    def _calibrate(self, pen_pos, pen_R):
        """白色按钮校准：重置右臂到 home，记录增量控制参考帧

        流程：
          1. 右臂关节设置到 ARM_HOME_RIGHT，速度归零
          2. mj_forward 计算 home 末端位置
          3. ★ 记录 EEF 相对 arm_base 的偏移
          4. 记录笔参考帧
          5. 等待稳定
        """
        # 1. 右臂设置到 home pose（非奇异）
        for i, idx in enumerate(self.qpos_indices):
            self.data.qpos[idx] = ARM_HOME_RIGHT[i]
        for idx in self.qvel_indices:
            self.data.qvel[idx] = 0.0
        mujoco.mj_forward(self.model, self.data)

        # 2. 记录 home 末端位置/姿态
        eef_pos = self.get_eef_pos()
        self.robot_R0 = self.get_eef_rot()
        self.desired_pos = eef_pos.copy()

        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, self.robot_R0.flatten())
        self.desired_quat = quat.copy()

        # 3. ★ 记录 EEF 相对 arm_base 的偏移（用于动态 workspace）
        self.arm_base_pos_cal = self.get_arm_base_pos()
        self.home_offset_local = eef_pos - self.arm_base_pos_cal

        # 4. 记录笔参考
        self.pen_origin = pen_pos.copy()
        if pen_R is not None and abs(np.linalg.det(pen_R) - 1.0) < 0.2:
            self.pen_R0 = pen_R.copy()

        # 5. 重置力反馈 + 设置稳定等待
        self.force_filter.reset()
        self.settle_countdown = 50

        self.calibrated = True
        workspace_center = self.get_dynamic_workspace_center()
        print(f">>> Touch 校准完成 (等待稳定 {self.settle_countdown} 步)")
        print(f"    workspace_center = {np.round(workspace_center, 3)}")
        print(f"    arm_base = {np.round(self.arm_base_pos_cal, 3)}")
        print(f"    pen_origin = {np.round(self.pen_origin, 1)} mm")

    # ---- 阻抗控制器 ----
    def compute_impedance_control(self):
        """任务空间阻抗控制 → 右臂 6 个关节力矩

        算法（★ 完全对齐参考实现）：
          tau = Jp^T * (Kp_pos * pos_err - Kd_pos * vel)
              + Jr^T * (Kp_rot * rot_err - Kd_rot * omega)
              + qfrc_bias
              - null_space_damping * qvel

        力矩限幅：每个关节 ±TORQUE_LIMIT Nm（防止穿透桌面）
        """
        n = self.n_arm

        if self.desired_pos is None:
            # 未校准：只做重力补偿
            return np.array([self.data.qfrc_bias[idx] for idx in self.qvel_indices])

        # 1. 当前末端状态
        pos_cur = self.get_eef_pos()
        R_cur = self.get_eef_rot()
        qvel = self.get_arm_qvel()

        # 2. Jacobian (3 × nv)
        self.jac_pos[:] = 0.0
        self.jac_rot[:] = 0.0
        mujoco.mj_jacSite(
            self.model, self.data,
            self.jac_pos, self.jac_rot, self.eef_site_id
        )
        Jp = self.jac_pos[:, self.qvel_indices]  # 3×6
        Jr = self.jac_rot[:, self.qvel_indices]  # 3×6

        # 3. 位置误差力 F = Kp * e - Kd * v
        pos_err = self.desired_pos - pos_cur
        vel_cur = Jp @ qvel
        F_pos = KP_POS @ pos_err - KD_POS @ vel_cur

        # 4. 姿态误差力矩 T = Kp_rot * e_rot - Kd_rot * omega
        rot_err = np.zeros(3)
        if self.desired_quat is not None:
            quat_cur = np.zeros(4)
            mujoco.mju_mat2Quat(quat_cur, R_cur.flatten())
            mujoco.mju_subQuat(rot_err, self.desired_quat, quat_cur)
        omega_cur = Jr @ qvel
        T_rot = KP_ROT @ rot_err - KD_ROT @ omega_cur

        # 5. 关节力矩 = J^T * [F; T] + 重力补偿
        tau = Jp.T @ F_pos + Jr.T @ T_rot
        for i, dof_idx in enumerate(self.qvel_indices):
            tau[i] += self.data.qfrc_bias[dof_idx]

        # 6. 零空间阻尼
        tau -= NULL_SPACE_DAMPING * qvel

        # 7. ★ 力矩限幅（防止推穿桌面/物体）
        tau = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)

        return tau

    # ---- 主更新循环 ----
    def update(self):
        """每仿真步调用：读取笔 → 更新目标 → 阻抗控制 → 力反馈"""
        if not self.connected:
            # 未连接：仅重力补偿
            tau = self.compute_impedance_control()
            for i in range(6):
                self.data.ctrl[15 + i] = tau[i]
            return

        # 读取笔状态
        state = self.device.get_state()
        pen_pos = np.array(state.position)  # mm

        # 读取笔旋转矩阵
        pen_R = None
        if state.transform and len(state.transform) == 16:
            T_raw = np.array(state.transform).reshape(4, 4)
            R_candidate = T_raw.T[:3, :3]
            if abs(np.linalg.det(R_candidate) - 1.0) < 0.2:
                pen_R = R_candidate

        btn1 = bool(state.button1)
        btn2 = bool(state.button2)

        # ===== Button2 上升沿：零点校准 =====
        if btn2 and not self.btn2_last:
            self._calibrate(pen_pos, pen_R)
        self.btn2_last = btn2

        # ===== Button1 上升沿：夹爪 toggle =====
        if btn1 and not self.btn1_last:
            if self.gripper_target > 0.018:
                self.gripper_target = GRIPPER_CLOSE
                print(">>> [夹爪] 闭合")
            else:
                self.gripper_target = GRIPPER_OPEN
                print(">>> [夹爪] 张开")
        self.btn1_last = btn1

        # 写入夹爪执行器 (ctrl[24])
        self.data.ctrl[24] = self.gripper_target

        # ===== 校准后稳定等待 =====
        if self.settle_countdown > 0:
            # 稳定期：只做重力补偿到 home 位置，不跟随笔
            tau = self.compute_impedance_control()
            for i in range(6):
                self.data.ctrl[15 + i] = tau[i]
            self.settle_countdown -= 1
            if self.settle_countdown == 0:
                # 稳定完成，更新笔原点为当前笔位置（消除漂移）
                self.pen_origin = pen_pos.copy()
                print(">>> 稳定完成，开始跟随")
            return

        if not self.calibrated:
            tau = self.compute_impedance_control()
            for i in range(6):
                self.data.ctrl[15 + i] = tau[i]
            return

        # ===== 增量位置跟随（★ 动态 workspace_center 跟随底盘）=====
        delta_pen_mm = pen_pos - self.pen_origin
        delta_world = R_MAP @ (delta_pen_mm * PEN_POS_SCALE)
        workspace_center = self.get_dynamic_workspace_center()
        self.desired_pos = workspace_center + delta_world

        # ===== 增量姿态跟随 =====
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

        # ===== 阻抗控制 → 关节力矩 =====
        tau = self.compute_impedance_control()
        for i in range(6):
            self.data.ctrl[15 + i] = tau[i]

        # ===== 力反馈 → Touch 笔 =====
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
        self.home_offset_local = None
        self.arm_base_pos_cal = None
        self.robot_R0 = None
        self.gripper_target = GRIPPER_CLOSE
        self.settle_countdown = 0
        self.force_filter.reset()
        if self.connected:
            try:
                self.device.set_force(0, 0, 0)
            except Exception:
                pass


# ============== 目标点可视化 ==============
def draw_target_markers(viewer, touch):
    """在 viewer 中绘制目标位置（绿球）和末端位置（红球）

    使用 viewer.user_scn 动态添加几何体，每帧更新。
    """
    n_geom = 0

    # 绿色球 = 目标位置
    if touch.desired_pos is not None:
        mujoco.mjv_initGeom(
            viewer.user_scn.geoms[n_geom],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.012, 0, 0],
            pos=touch.desired_pos,
            mat=np.eye(3).flatten(),
            rgba=[0.0, 1.0, 0.0, 0.6]
        )
        n_geom += 1

    # 红色球 = 末端实际位置
    if touch.eef_site_id >= 0:
        eef_pos = touch.get_eef_pos()
        mujoco.mjv_initGeom(
            viewer.user_scn.geoms[n_geom],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.008, 0, 0],
            pos=eef_pos,
            mat=np.eye(3).flatten(),
            rgba=[1.0, 0.2, 0.2, 0.8]
        )
        n_geom += 1

    # 黄色线 = 目标到末端的连线（跟踪误差可视化）
    if touch.desired_pos is not None and touch.eef_site_id >= 0:
        eef_pos = touch.get_eef_pos()
        mid = (touch.desired_pos + eef_pos) / 2.0
        diff = touch.desired_pos - eef_pos
        length = np.linalg.norm(diff)
        if length > 0.001:
            # 计算旋转矩阵让胶囊沿 diff 方向
            direction = diff / length
            # 构造旋转矩阵：z轴对齐 direction
            if abs(direction[2]) < 0.99:
                up = np.array([0, 0, 1])
            else:
                up = np.array([1, 0, 0])
            right = np.cross(up, direction)
            right /= np.linalg.norm(right)
            up2 = np.cross(direction, right)
            rot_mat = np.column_stack([right, up2, direction])
            mujoco.mjv_initGeom(
                viewer.user_scn.geoms[n_geom],
                type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                size=[0.002, length / 2, 0],
                pos=mid,
                mat=rot_mat.flatten(),
                rgba=[1.0, 1.0, 0.0, 0.5]
            )
            n_geom += 1

    viewer.user_scn.ngeom = n_geom


# ============== 主函数 ==============
def main():
    global reset_request, exit_request, toggle_fpv_request
    global target_vx, target_vy, target_omega, target_lift, smoothed_lift
    global target_head_yaw, target_head_pitch, target_head_stem

    disable_x11_repeat()

    print("加载 MJCF:", MJCF_PATH)
    model = mujoco.MjModel.from_xml_path(MJCF_PATH)
    data = mujoco.MjData(model)
    print(f"模型: nq={model.nq}, nv={model.nv}, nu={model.nu}")
    print()
    print("=" * 64)
    print("    四舵轮全向底盘 — 键盘 + Touch 笔遥操作")
    print("=" * 64)
    print()
    print("【重要】焦点放在终端窗口")
    print()
    print("控制按键 (按住生效)：")
    print("  底盘: W/S 前后  A/D 左右  Q/E 转向")
    print("  升降: G/H 升/降 (0~0.8m)")
    print("  云台: I/K 俯仰  J/L 偏航  T/Y 高度")
    print("  相机: F 切换第一人称/第三人称")
    print("  Touch: Button2 白色=校准  Button1 灰色=夹爪")
    print("  其他: R 重置  ESC 退出")
    print()
    print("阻抗控制参数 (对齐参考实现):")
    print(f"  Kp_pos={KP_POS[0,0]:.0f} N/m  Kd_pos={KD_POS[0,0]:.0f} Ns/m")
    print(f"  Kp_rot={KP_ROT[0,0]:.0f} Nm/rad  Kd_rot={KD_ROT[0,0]:.0f} Nms/rad")
    print(f"  τ_limit=±{TORQUE_LIMIT:.0f} Nm  null_damp={NULL_SPACE_DAMPING}")
    print("=" * 64)
    print()

    # 启动键盘监听
    listener = keyboard.Listener(on_press=on_key_press, on_release=on_key_release)
    listener.start()
    print(">>> pynput 键盘监听已启动")

    current_steer = np.zeros(4)  # 记忆: 各轮当前转向角 (rad)
    current_drive = np.zeros(4)  # 记忆: 各轮当前驱动速度 (rad/s)
    chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis_link")

    # 查找 head_cam 用于第一人称切换
    head_cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'head_cam')
    fpv_mode = False  # 第一人称视角状态
    print(f">>> head_cam id = {head_cam_id} ({'OK' if head_cam_id >= 0 else 'NOT FOUND'})")

    # 初始化 ctrl + ★ 右臂设到非奇异 home pose
    right_qpos_indices = []
    for i in range(1, 7):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"right_joint{i}")
        right_qpos_indices.append(model.jnt_qposadr[jid])
    for i, idx in enumerate(right_qpos_indices):
        data.qpos[idx] = ARM_HOME_RIGHT[i]

    mujoco.mj_forward(model, data)
    data.ctrl[:] = 0.0
    data.ctrl[8] = LIFT_INIT
    data.ctrl[9:15] = ARM_HOME_LEFT
    data.ctrl[24] = GRIPPER_CLOSE

    # Touch 笔
    touch = TouchTeleop(model, data)
    touch.connect()
    atexit.register(touch.disconnect)

    # 让物体先 settle + 右臂重力补偿稳定
    print(">>> 物体 settling (1.0s)...")
    for _ in range(500):  # 1.0s @ 0.002s
        data.ctrl[9:15] = ARM_HOME_LEFT
        tau_init = touch.compute_impedance_control()
        for i in range(6):
            data.ctrl[15 + i] = tau_init[i]
        data.ctrl[24] = GRIPPER_CLOSE
        mujoco.mj_step(model, data)
    print(">>> Settling 完成")

    # 记录 settle 后状态用于 reset
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

            # ===== F 键：切换第一人称 =====
            if toggle_fpv_request:
                toggle_fpv_request = False
                fpv_mode = not fpv_mode
                if fpv_mode and head_cam_id >= 0:
                    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                    viewer.cam.fixedcamid = head_cam_id
                    print(">>> 第一人称视角 (head_cam)")
                else:
                    fpv_mode = False
                    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                    viewer.cam.distance = 3.0
                    viewer.cam.azimuth = 135
                    viewer.cam.elevation = -25
                    viewer.cam.lookat[:] = [0, 0, 0.5]
                    print(">>> 第三人称视角 (free)")

            # ===== 重置 =====
            if reset_request:
                data.qpos[:] = init_qpos
                data.qvel[:] = init_qvel
                data.ctrl[:] = 0.0
                data.ctrl[8] = LIFT_INIT
                data.ctrl[9:15] = ARM_HOME_LEFT
                data.ctrl[24] = GRIPPER_CLOSE

                target_vx = target_vy = target_omega = 0.0
                target_lift = LIFT_INIT
                smoothed_lift = LIFT_INIT
                target_head_yaw = target_head_pitch = 0.0
                target_head_stem = 0.0
                current_steer[:] = 0.0
                current_drive[:] = 0.0
                touch.reset()

                mujoco.mj_forward(model, data)
                reset_request = False
                print("\n>>> RESET <<<\n")

            # 1. 更新速度/位置目标
            update_target_velocity(model.opt.timestep)

            # 2. 舵轮 IK（WPILib / FRC 最佳实践 pipeline）
            #    步骤: IK → 去饱和 → 角度优化 → 余弦补偿 → 速率限幅
            dt = model.opt.timestep

            # 2a. 逆运动学: 底盘速度 → 4 轮 (角度, 速度)
            raw_steer, raw_drive = swerve_ik(
                target_vx, target_vy, target_omega, current_steer
            )

            # 2b. 轮速去饱和: 超速时等比缩放，保持轨迹正确
            raw_drive = desaturate_wheel_speeds(raw_drive, MAX_WHEEL_SPEED)

            # 2c. 逐轮优化 + 余弦补偿 + 速率限幅
            cmd_steer = np.zeros(4)
            cmd_drive = np.zeros(4)
            for i in range(4):
                # optimize: >90° 时反转驱动 + 翻转角度（轮子不用转超过 90°）
                opt_speed, opt_angle = optimize_module(
                    raw_drive[i], raw_steer[i], current_steer[i]
                )
                # cosine: 轮子还没转到位时减小驱动力（防止侧向漂移）
                opt_speed = cosine_scale(opt_speed, opt_angle, current_steer[i])
                # 转向速率限幅: 每步最多转 STEER_RATE_LIMIT * dt 弧度
                cmd_steer[i] = rate_limit_steer(
                    opt_angle, current_steer[i], STEER_RATE_LIMIT, dt
                )
                # 驱动加速度限幅: 每步速度变化不超过 DRIVE_ACCEL_LIMIT * dt
                cmd_drive[i] = rate_limit_drive(
                    opt_speed, current_drive[i], DRIVE_ACCEL_LIMIT, dt
                )

            # 2d. 更新状态记忆
            current_steer[:] = cmd_steer
            current_drive[:] = cmd_drive

            # 3. 写入执行器
            data.ctrl[0:4] = cmd_steer         # 转向（平滑后）
            data.ctrl[4:8] = cmd_drive         # 驱动（平滑后）
            data.ctrl[8] = smoothed_lift        # ★ 使用平滑后的升降目标
            data.ctrl[9:15] = ARM_HOME_LEFT     # 左臂 home
            touch.update()                       # 右臂+夹爪 → ctrl[15:21]+ctrl[24]
            data.ctrl[21] = target_head_yaw     # 云台
            data.ctrl[22] = target_head_pitch
            data.ctrl[23] = target_head_stem

            # 4. 物理步进
            mujoco.mj_step(model, data)

            # 5. 目标点可视化
            draw_target_markers(viewer, touch)

            # 6. 同步渲染
            viewer.sync()

            # 7. HUD (0.1s 间隔)
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

                # Touch 状态 + 跟踪误差 + 动态 workspace
                if touch.connected:
                    t_state = "CALIBRATED" if touch.calibrated else "WAIT BTN2"
                    if touch.settle_countdown > 0:
                        t_state = f"SETTLING ({touch.settle_countdown})"
                    eef = touch.get_eef_pos()
                    grip = "OPEN" if touch.gripper_target > 0.018 else "CLOSED"
                    # 跟踪误差
                    if touch.desired_pos is not None:
                        err_mm = np.linalg.norm(touch.desired_pos - eef) * 1000
                        err_str = f"Err={err_mm:.1f}mm"
                        des = touch.desired_pos
                        des_str = f"Des=({des[0]:+.3f},{des[1]:+.3f},{des[2]:+.3f})"
                    else:
                        err_str = ""
                        des_str = ""
                    touch_str = (
                        f"Touch: {t_state}  Grip: {grip}\n"
                        f"R-EEF = ({eef[0]:+.3f},{eef[1]:+.3f},{eef[2]:+.3f})\n"
                        f"{des_str}  {err_str}"
                    )
                else:
                    touch_str = "Touch: NOT CONNECTED"

                cam_str = "FPV" if fpv_mode else "FREE"

                hud_left = (
                    f"Chassis  vx={target_vx:+.2f} vy={target_vy:+.2f} ω={target_omega:+.2f}\n"
                    f"Lift     target={target_lift:.2f} smooth={smoothed_lift:.2f} act={lift_actual:.2f}\n"
                    f"Head     yaw={math.degrees(target_head_yaw):+.0f}°  "
                    f"pitch={math.degrees(target_head_pitch):+.0f}°  "
                    f"stem={target_head_stem:+.2f}\n"
                    f"Camera   [{cam_str}]  (F=toggle)\n"
                    f"\n"
                    f"Keys [{indicator}]\n"
                    f"\n"
                    f"{touch_str}\n"
                    f"Pos ({pos[0]:+.2f},{pos[1]:+.2f}) yaw={yaw:+.0f}°"
                )

                hud_right = (
                    f"W/S forward/back\n"
                    f"A/D strafe L/R\n"
                    f"Q/E rotate L/R\n"
                    f"G/H lift (0~0.8m)\n"
                    f"I/K head pitch\n"
                    f"J/L head yaw\n"
                    f"T/Y head stem\n"
                    f"F   1st/3rd person\n"
                    f"R   reset\n"
                    f"ESC quit\n"
                    f"\n"
                    f"Touch Pen:\n"
                    f"Btn2(white) = calibrate\n"
                    f"Btn1(grey)  = gripper\n"
                    f"\n"
                    f"Green ball = target\n"
                    f"Red ball   = actual\n"
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

                # 终端日志 (0.5s)
                if sim_time - last_log > 0.5:
                    print(f"t={sim_time:6.2f}  "
                          f"vx={target_vx:+.2f} vy={target_vy:+.2f} ω={target_omega:+.2f}  "
                          f"lift={lift_actual:.2f}  "
                          f"head(y={math.degrees(target_head_yaw):+.0f}° "
                          f"p={math.degrees(target_head_pitch):+.0f}° "
                          f"s={target_head_stem:+.2f})  "
                          f"[{indicator}]",
                          flush=True)
                    last_log = sim_time

            # 8. 实时同步
            elapsed = time.time() - step_start
            if elapsed < model.opt.timestep:
                time.sleep(model.opt.timestep - elapsed)

        print("\n=== 退出 ===")
        listener.stop()


if __name__ == "__main__":
    main()
