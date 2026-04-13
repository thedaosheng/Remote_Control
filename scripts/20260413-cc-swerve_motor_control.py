#!/usr/bin/env python3
"""
==============================================================
  舵轮底盘 MuJoCo 仿真 + 达妙 DM4310 真实电机联动
==============================================================

功能：
  1. 打开 MuJoCo 仿真模型，渲染底盘 + 舵轮
  2. WASD 键盘控制底盘全向移动
  3. 舵轮逆运动学 (WPILib 算法) 计算各轮目标
  4. 同步发送指令到真实 DM4310 电机:
       - 转向电机 (ID 1~4): POS_VEL 模式（位置+速度限制）
       - 驱动电机 (ID 5~8): VEL 模式（速度控制）
  5. 实时显示电机反馈（位置/速度/力矩）

硬件前提：
  - 达妙 DM4310 电机 ×8（4 转向 + 4 驱动）
  - DISCOVER Robotics USB-CAN 适配器 ×1 或 ×2
  - 电机已上电，CAN ID 按 MC02 惯例配置

控制按键 (按住生效)：
  底盘:   W/S 前后    A/D 左右    Q/E 转向
  升降:   G/H 升/降
  云台:   I/K 俯仰    J/L 偏航
  其他:   R 重置      ESC 退出

用法：
  # 仿真 + 真实电机
  conda activate disc
  python3 /home/rhz/teleop/scripts/20260413-cc-swerve_motor_control.py

  # 仅仿真（无电机）
  python3 /home/rhz/teleop/scripts/20260413-cc-swerve_motor_control.py --sim-only

  # 指定 USB-CAN 适配器
  python3 /home/rhz/teleop/scripts/20260413-cc-swerve_motor_control.py --port /dev/ttyACM0

  # 指定两个适配器（转向/驱动分开）
  python3 /home/rhz/teleop/scripts/20260413-cc-swerve_motor_control.py --port /dev/ttyACM0 --port2 /dev/ttyACM1
"""

import os
import sys
import math
import time
import threading
import subprocess
import atexit
import argparse
import numpy as np

# Touch 力反馈笔的兼容库路径
_LIB_COMPAT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '..', 'lib')
if os.path.isdir(_LIB_COMPAT) and _LIB_COMPAT not in os.environ.get('LD_LIBRARY_PATH', ''):
    os.environ['LD_LIBRARY_PATH'] = (_LIB_COMPAT + ':'
                                      + os.environ.get('LD_LIBRARY_PATH', ''))
    os.execv(sys.executable, [sys.executable] + sys.argv)

import mujoco
import mujoco.viewer
from pynput import keyboard

# 达妙 CAN 库
sys.path.insert(0, '/home/rhz/teleop/DM_Control_Python')
from DM_CAN import (Motor, MotorControl, DM_Motor_Type, DM_variable,
                     Control_Type)


# =====================================================================
#  配置参数
# =====================================================================

# MuJoCo 模型路径
MJCF_PATH = '/home/rhz/teleop/RemoteControl/swerve_chassis/20260408-cc-swerve_chassis.xml'

# USB-CAN 适配器串口
DEFAULT_PORT = '/dev/ttyACM0'
BAUD = 921600

# ---- 电机 CAN ID 配置 ----
# SlaveID: 1~4 转向, 5~8 驱动
# MasterID = SlaveID + 0x10
STEER_IDS = [1, 2, 3, 4]       # FL, FR, RL, RR
DRIVE_IDS = [5, 6, 7, 8]       # FL, FR, RL, RR
MASTER_OFFSET = 0x10

# ---- 底盘几何参数 ----
WHEEL_RADIUS = 0.06  # m
WHEEL_POSITIONS = np.array([
    [ 0.16,  0.15],  # FL
    [ 0.16, -0.15],  # FR
    [-0.16,  0.15],  # RL
    [-0.16, -0.15],  # RR
])

# ---- 底盘速度上限 ----
MAX_VX = 1.0       # m/s
MAX_VY = 1.0       # m/s
MAX_OMEGA = 2.0    # rad/s
CHASSIS_SPEED = 0.6     # m/s（键盘输入时的目标速度）
CHASSIS_OMEGA_SPEED = 1.5     # rad/s

# ---- 舵轮控制参数 ----
MAX_WHEEL_SPEED = MAX_VX / WHEEL_RADIUS * 1.5  # rad/s, 余量系数
STEER_RATE_LIMIT = 8.0    # rad/s — 转向速率限幅
DRIVE_ACCEL_LIMIT = 30.0  # rad/s^2 — 驱动加速度限幅

# ---- 斜率限幅（键盘→底盘速度平滑过渡）----
CHASSIS_VX_SLEW = 3.0    # m/s^2
CHASSIS_VY_SLEW = 3.0    # m/s^2
CHASSIS_OMEGA_SLEW = 6.0 # rad/s^2

# ---- 真实电机控制参数 ----
# 转向电机 (POS_VEL 模式): 目标位置 + 速度限制
STEER_VEL_LIMIT = 10.0   # rad/s — 转向电机的速度限制

# 驱动电机 (VEL 模式): 目标速度
DRIVE_VEL_SCALE = 1.0    # 仿真速度 → 电机速度的缩放因子（先 1:1）

# 安全限幅
STEER_POS_LIMIT = 6.28   # rad — 转向位置限幅（±2π，无限旋转）
DRIVE_VEL_LIMIT = 25.0   # rad/s — 驱动速度限幅

# ---- MuJoCo 执行器索引 (与 XML 对应) ----
CTRL_CHASSIS_VX    = 0
CTRL_CHASSIS_VY    = 1
CTRL_CHASSIS_OMEGA = 2
CTRL_STEER_START   = 3   # 3,4,5,6
CTRL_DRIVE_START   = 7   # 7,8,9,10
CTRL_LIFT          = 11
CTRL_LEFT_ARM      = 12  # 12~17
CTRL_RIGHT_ARM     = 18  # 18~23
CTRL_HEAD_YAW      = 24
CTRL_HEAD_PITCH    = 25
CTRL_HEAD_STEM     = 26
CTRL_GRIPPER       = 27

# ---- 升降参数 ----
LIFT_MIN = 0.0
LIFT_MAX = 0.8
LIFT_RATE = 0.5   # m/s

# ---- 云台参数 ----
HEAD_YAW_MIN = -1.5708
HEAD_YAW_MAX =  1.5708
HEAD_PITCH_MIN = -0.7854
HEAD_PITCH_MAX =  0.7854
HEAD_RATE = 2.5   # rad/s

# ---- MuJoCo 混合驱动（底盘推力直接施加到 body）----
CHASSIS_FORCE_KP = 500.0
CHASSIS_FORCE_KD = 50.0
CHASSIS_TORQUE_KP = 200.0
CHASSIS_TORQUE_KD = 20.0
CHASSIS_MAX_FORCE = 800.0
CHASSIS_MAX_TORQUE = 300.0

# ---- 双臂 home pose ----
ARM_HOME_LEFT  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ARM_HOME_RIGHT = [0.0, -0.5, 0.3, 1.0, 0.0, 0.0]


# =====================================================================
#  舵轮逆运动学 (WPILib / Team 254 算法)
# =====================================================================

# 构建 (8,3) 逆运动学矩阵
_IK_MATRIX = np.zeros((8, 3))
for _i, (_mx, _my) in enumerate(WHEEL_POSITIONS):
    _IK_MATRIX[2 * _i,     :] = [1.0, 0.0, -_my]
    _IK_MATRIX[2 * _i + 1, :] = [0.0, 1.0,  _mx]


def _normalize_angle(angle):
    """将角度归一化到 [-π, π]"""
    angle = angle % (2.0 * math.pi)
    if angle > math.pi:
        angle -= 2.0 * math.pi
    return angle


def swerve_ik(vx, vy, omega, prev_steer_angles):
    """
    底盘速度 → 4 组 (转向角, 驱动转速)

    使用 8×3 矩阵逆运动学，零速时保持上次轮角。

    参数:
      vx, vy: 底盘线速度 (m/s), +x=前, +y=左
      omega:  底盘角速度 (rad/s), 逆时针为正
      prev_steer_angles: 上一帧的 4 个转向角

    返回:
      steer_angles: ndarray(4,)  各轮目标转向角 (rad)
      drive_speeds:  ndarray(4,)  各轮目标驱动转速 (rad/s)
    """
    steer_angles = np.zeros(4)
    drive_speeds = np.zeros(4)

    # 全零输入: 保持上次角度，速度归零
    if abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(omega) < 1e-6:
        return prev_steer_angles.copy(), drive_speeds

    # 矩阵逆运动学
    chassis_vec = np.array([vx, vy, omega])
    module_vecs = _IK_MATRIX @ chassis_vec

    for i in range(4):
        wvx = module_vecs[2 * i]
        wvy = module_vecs[2 * i + 1]
        speed = math.hypot(wvx, wvy)

        if speed < 1e-6:
            steer_angles[i] = prev_steer_angles[i]
            drive_speeds[i] = 0.0
        else:
            steer_angles[i] = math.atan2(wvy, wvx)
            drive_speeds[i] = speed / WHEEL_RADIUS

    return steer_angles, drive_speeds


def optimize_module(desired_speed, desired_angle, current_angle):
    """
    WPILib SwerveModuleState.optimize():
    轮子 (speed, θ) 等价于 (-speed, θ+π)，
    因此永远不需要转超过 90°。
    """
    delta = _normalize_angle(desired_angle - current_angle)
    if abs(delta) > math.pi / 2.0:
        desired_speed = -desired_speed
        desired_angle = _normalize_angle(desired_angle + math.pi)
    return desired_speed, desired_angle


def desaturate_wheel_speeds(drive_speeds, max_speed):
    """等比例缩放轮速，使最大值不超过物理极限"""
    real_max = np.max(np.abs(drive_speeds))
    if real_max > max_speed:
        return drive_speeds * (max_speed / real_max)
    return drive_speeds.copy()


def cosine_scale(desired_speed, desired_angle, current_angle):
    """余弦补偿: 转向未到位时减小驱动力"""
    angle_error = _normalize_angle(desired_angle - current_angle)
    return desired_speed * math.cos(angle_error)


def rate_limit_steer(target_angle, current_angle, max_rate, dt):
    """转向速率限幅"""
    delta = _normalize_angle(target_angle - current_angle)
    max_step = max_rate * dt
    if abs(delta) > max_step:
        delta = math.copysign(max_step, delta)
    return _normalize_angle(current_angle + delta)


def rate_limit_drive(target_speed, current_speed, max_accel, dt):
    """驱动加速度限幅"""
    max_step = max_accel * dt
    delta = target_speed - current_speed
    if abs(delta) > max_step:
        delta = math.copysign(max_step, delta)
    return current_speed + delta


def slew_rate_limit(current, target, slew, dt):
    """通用斜率限幅"""
    max_step = slew * dt
    delta = target - current
    if abs(delta) > max_step:
        delta = math.copysign(max_step, delta)
    return current + delta


# =====================================================================
#  键盘输入 (pynput)
# =====================================================================

key_held = {
    'w': False, 's': False,
    'a': False, 'd': False,
    'q': False, 'e': False,
    'g': False, 'h': False,
    'i': False, 'k': False,
    'j': False, 'l': False,
}
reset_request = False
exit_request = False
state_lock = threading.Lock()


def is_key_held(ch):
    with state_lock:
        return key_held.get(ch, False)


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


def disable_x11_repeat():
    """禁用 X11 键盘自动重复（避免 pynput 重复事件）"""
    try:
        subprocess.run(['xset', '-r'], check=False, timeout=2,
                       env={**os.environ, 'DISPLAY': ':0'})
        print(">>> X11 键盘自动重复已禁用")
    except Exception:
        pass


def restore_x11_repeat():
    """恢复 X11 键盘自动重复"""
    try:
        subprocess.run(['xset', 'r', 'on'], check=False, timeout=2,
                       env={**os.environ, 'DISPLAY': ':0'})
    except Exception:
        pass


atexit.register(restore_x11_repeat)


# =====================================================================
#  达妙电机管理器
# =====================================================================

class DM4310MotorManager:
    """
    管理 8 个 DM4310 电机（4 转向 + 4 驱动）。

    转向电机使用 POS_VEL 模式（位置+速度限制）：
      - 精确定位到目标角度
      - 速度限制保证平滑运动

    驱动电机使用 VEL 模式（速度控制）：
      - 直接设置轮子转速
      - 适合连续旋转
    """

    def __init__(self, port, port2=None):
        """
        初始化电机管理器。

        参数:
          port:  第一个 USB-CAN 适配器串口路径
          port2: 第二个适配器（可选，用于分开转向/驱动总线）
        """
        import serial as _serial

        self.steer_motors = []   # 4 个转向电机对象
        self.drive_motors = []   # 4 个驱动电机对象
        self.mc_steer = None     # 转向电机的 MotorControl
        self.mc_drive = None     # 驱动电机的 MotorControl
        self.enabled = False

        # ---- 打开串口 ----
        print(f">>> 打开 USB-CAN 适配器: {port}")
        ser1 = _serial.Serial(port, BAUD, timeout=0.5)
        self.mc_steer = MotorControl(ser1)

        if port2 and port2 != port:
            # 双总线：转向在 port, 驱动在 port2
            print(f">>> 打开第二个 USB-CAN 适配器: {port2}")
            ser2 = _serial.Serial(port2, BAUD, timeout=0.5)
            self.mc_drive = MotorControl(ser2)
        else:
            # 单总线：所有电机在同一适配器
            self.mc_drive = self.mc_steer

        # ---- 创建电机对象 ----
        for sid in STEER_IDS:
            motor = Motor(DM_Motor_Type.DM4310, sid, sid + MASTER_OFFSET)
            self.steer_motors.append(motor)
            self.mc_steer.addMotor(motor)

        for sid in DRIVE_IDS:
            motor = Motor(DM_Motor_Type.DM4310, sid, sid + MASTER_OFFSET)
            self.drive_motors.append(motor)
            self.mc_drive.addMotor(motor)

        print(f"  转向电机 ID: {STEER_IDS} (POS_VEL 模式)")
        print(f"  驱动电机 ID: {DRIVE_IDS} (VEL 模式)")

    def enable_all(self):
        """切换控制模式并使能所有电机"""
        print(">>> 切换控制模式...")

        # 转向电机 → POS_VEL 模式
        for i, motor in enumerate(self.steer_motors):
            ok = self.mc_steer.switchControlMode(motor, Control_Type.POS_VEL)
            status = "✓" if ok else "✗"
            print(f"  转向 ID={STEER_IDS[i]} → POS_VEL: {status}")
            time.sleep(0.05)

        # 驱动电机 → VEL 模式
        for i, motor in enumerate(self.drive_motors):
            ok = self.mc_drive.switchControlMode(motor, Control_Type.VEL)
            status = "✓" if ok else "✗"
            print(f"  驱动 ID={DRIVE_IDS[i]} → VEL: {status}")
            time.sleep(0.05)

        print(">>> 使能电机...")

        for i, motor in enumerate(self.steer_motors):
            self.mc_steer.enable(motor)
            print(f"  转向 ID={STEER_IDS[i]}: ✓")
            time.sleep(0.05)

        for i, motor in enumerate(self.drive_motors):
            self.mc_drive.enable(motor)
            print(f"  驱动 ID={DRIVE_IDS[i]}: ✓")
            time.sleep(0.05)

        self.enabled = True
        print(">>> 所有电机已使能")

    def disable_all(self):
        """失能所有电机"""
        for motor in self.steer_motors:
            try:
                self.mc_steer.disable(motor)
            except Exception:
                pass
            time.sleep(0.02)

        for motor in self.drive_motors:
            try:
                self.mc_drive.disable(motor)
            except Exception:
                pass
            time.sleep(0.02)

        self.enabled = False
        print(">>> 所有电机已失能")

    def send_commands(self, steer_angles, drive_speeds):
        """
        发送控制指令到真实电机。

        参数:
          steer_angles: ndarray(4,)  各轮目标转向角 (rad)
          drive_speeds: ndarray(4,)  各轮目标驱动速度 (rad/s)
        """
        if not self.enabled:
            return

        # 转向电机: POS_VEL 模式 → 发送(目标位置, 速度限制)
        for i, motor in enumerate(self.steer_motors):
            pos = float(np.clip(steer_angles[i], -STEER_POS_LIMIT, STEER_POS_LIMIT))
            try:
                self.mc_steer.control_Pos_Vel(motor, pos, STEER_VEL_LIMIT)
            except Exception:
                pass

        # 驱动电机: VEL 模式 → 发送目标速度
        for i, motor in enumerate(self.drive_motors):
            vel = float(np.clip(drive_speeds[i] * DRIVE_VEL_SCALE,
                                -DRIVE_VEL_LIMIT, DRIVE_VEL_LIMIT))
            try:
                self.mc_drive.control_Vel(motor, vel)
            except Exception:
                pass

    def stop_all(self):
        """紧急停止: 所有驱动速度归零，转向保持"""
        if not self.enabled:
            return
        for motor in self.drive_motors:
            try:
                self.mc_drive.control_Vel(motor, 0.0)
            except Exception:
                pass

    def get_feedback(self):
        """
        读取所有电机的反馈状态。

        返回:
          dict: {
            'steer_pos': [4], 'steer_vel': [4], 'steer_tau': [4],
            'drive_pos': [4], 'drive_vel': [4], 'drive_tau': [4],
          }
        """
        fb = {
            'steer_pos': [], 'steer_vel': [], 'steer_tau': [],
            'drive_pos': [], 'drive_vel': [], 'drive_tau': [],
        }
        for motor in self.steer_motors:
            fb['steer_pos'].append(motor.getPosition())
            fb['steer_vel'].append(motor.getVelocity())
            fb['steer_tau'].append(motor.getTorque())
        for motor in self.drive_motors:
            fb['drive_pos'].append(motor.getPosition())
            fb['drive_vel'].append(motor.getVelocity())
            fb['drive_tau'].append(motor.getTorque())
        return fb

    def close(self):
        """关闭串口"""
        self.disable_all()
        try:
            self.mc_steer.serial_.close()
        except Exception:
            pass
        if self.mc_drive is not self.mc_steer:
            try:
                self.mc_drive.serial_.close()
            except Exception:
                pass
        print(">>> 串口已关闭")


# =====================================================================
#  主循环
# =====================================================================

def main():
    parser = argparse.ArgumentParser(description='舵轮底盘 MuJoCo + DM4310 联动控制')
    parser.add_argument('--sim-only', action='store_true',
                        help='仅仿真，不连接真实电机')
    parser.add_argument('--port', type=str, default=DEFAULT_PORT,
                        help=f'USB-CAN 适配器串口 (默认 {DEFAULT_PORT})')
    parser.add_argument('--port2', type=str, default=None,
                        help='第二个适配器（可选，转向/驱动分总线）')
    args = parser.parse_args()

    # ---- 加载 MuJoCo 模型 ----
    print("=" * 60)
    print("  舵轮底盘 MuJoCo + DM4310 联动控制")
    print("=" * 60)
    print(f"  模型: {MJCF_PATH}")
    print(f"  模式: {'仅仿真' if args.sim_only else '仿真 + 真实电机'}")
    print()

    model = mujoco.MjModel.from_xml_path(MJCF_PATH)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    # ---- 初始化双臂 home pose ----
    # 找到左右臂关节并设置初始位置
    for i in range(1, 7):
        jname = f"left_joint{i}"
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = ARM_HOME_LEFT[i - 1]
    for i in range(1, 7):
        jname = f"right_joint{i}"
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = ARM_HOME_RIGHT[i - 1]
    mujoco.mj_forward(model, data)

    # ---- 初始化真实电机 ----
    motor_mgr = None
    if not args.sim_only:
        try:
            motor_mgr = DM4310MotorManager(args.port, args.port2)
            motor_mgr.enable_all()
        except Exception as e:
            print(f"\n!!! 电机初始化失败: {e}")
            print(">>> 降级为仅仿真模式\n")
            motor_mgr = None

    # ---- 键盘监听 ----
    disable_x11_repeat()
    listener = keyboard.Listener(on_press=on_key_press, on_release=on_key_release)
    listener.start()

    # ---- 状态变量 ----
    # 底盘速度（经斜率限幅后的当前值）
    cur_vx = 0.0
    cur_vy = 0.0
    cur_omega = 0.0

    # 舵轮当前状态
    cur_steer = np.zeros(4)   # 当前转向角
    cur_drive = np.zeros(4)   # 当前驱动速度

    # 升降/云台
    target_lift = 0.0
    smoothed_lift = 0.0
    target_head_yaw = 0.0
    target_head_pitch = 0.0

    # ---- 找到底盘 body ID（用于直接施加推力）----
    chassis_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis_link")

    # 显示 HUD 的计数器
    hud_counter = 0

    print("\n>>> 控制就绪！按键:")
    print("    W/S=前后  A/D=左右  Q/E=转向  G/H=升降  I/K/J/L=云台")
    print("    R=重置  ESC=退出")
    if motor_mgr:
        print("    真实电机: 已连接")
    else:
        print("    真实电机: 未连接 (仅仿真)")
    print()

    # ---- MuJoCo 仿真主循环 ----
    global reset_request, exit_request

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 设置相机
        viewer.cam.distance = 3.0
        viewer.cam.elevation = -25.0
        viewer.cam.azimuth = 135.0

        sim_dt = model.opt.timestep  # 0.002s = 500Hz
        ctrl_dt = 0.02               # 50Hz 控制循环
        steps_per_ctrl = int(ctrl_dt / sim_dt)

        last_time = time.time()

        while viewer.is_running() and not exit_request:
            loop_start = time.time()
            dt = ctrl_dt  # 使用固定 dt

            # ========= 1. 键盘输入 → 底盘目标速度 (斜率限幅) =========
            w, s = is_key_held('w'), is_key_held('s')
            a, d = is_key_held('a'), is_key_held('d')
            q, e = is_key_held('q'), is_key_held('e')

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
                desired_omega = CHASSIS_OMEGA_SPEED
            elif e and not q:
                desired_omega = -CHASSIS_OMEGA_SPEED

            cur_vx = slew_rate_limit(cur_vx, desired_vx, CHASSIS_VX_SLEW, dt)
            cur_vy = slew_rate_limit(cur_vy, desired_vy, CHASSIS_VY_SLEW, dt)
            cur_omega = slew_rate_limit(cur_omega, desired_omega, CHASSIS_OMEGA_SLEW, dt)

            # 死区
            if abs(cur_vx) < 0.005: cur_vx = 0.0
            if abs(cur_vy) < 0.005: cur_vy = 0.0
            if abs(cur_omega) < 0.005: cur_omega = 0.0

            # ========= 2. 升降/云台 =========
            gg, hh = is_key_held('g'), is_key_held('h')
            if gg and not hh:
                target_lift = min(target_lift + LIFT_RATE * dt, LIFT_MAX)
            elif hh and not gg:
                target_lift = max(target_lift - LIFT_RATE * dt, LIFT_MIN)
            smoothed_lift = 0.05 * target_lift + 0.95 * smoothed_lift

            ii, kk = is_key_held('i'), is_key_held('k')
            jj, ll = is_key_held('j'), is_key_held('l')
            head_step = HEAD_RATE * dt
            if ii and not kk:
                target_head_pitch = max(target_head_pitch - head_step, HEAD_PITCH_MIN)
            elif kk and not ii:
                target_head_pitch = min(target_head_pitch + head_step, HEAD_PITCH_MAX)
            if jj and not ll:
                target_head_yaw = min(target_head_yaw + head_step, HEAD_YAW_MAX)
            elif ll and not jj:
                target_head_yaw = max(target_head_yaw - head_step, HEAD_YAW_MIN)

            # ========= 3. 舵轮逆运动学 =========
            raw_steer, raw_drive = swerve_ik(cur_vx, cur_vy, cur_omega, cur_steer)

            # 去饱和
            raw_drive = desaturate_wheel_speeds(raw_drive, MAX_WHEEL_SPEED)

            # 优化 + 余弦补偿 + 速率限幅
            opt_steer = np.zeros(4)
            opt_drive = np.zeros(4)
            for idx in range(4):
                spd, ang = optimize_module(raw_drive[idx], raw_steer[idx], cur_steer[idx])
                spd = cosine_scale(spd, ang, cur_steer[idx])
                ang = rate_limit_steer(ang, cur_steer[idx], STEER_RATE_LIMIT, dt)
                spd = rate_limit_drive(spd, cur_drive[idx], DRIVE_ACCEL_LIMIT, dt)
                opt_steer[idx] = ang
                opt_drive[idx] = spd

            cur_steer = opt_steer
            cur_drive = opt_drive

            # ========= 4. MuJoCo 控制 =========
            # 底盘推力（混合驱动 — 直接施力到 body）
            # 读取当前底盘速度（body frame）
            if chassis_body_id >= 0:
                chassis_quat = data.xquat[chassis_body_id]
                # 四元数 → 旋转矩阵
                R_chassis = np.zeros(9)
                mujoco.mju_quat2Mat(R_chassis, chassis_quat)
                R_chassis = R_chassis.reshape(3, 3)

                # 当前 body 速度 (world frame → body frame)
                linvel = data.cvel[chassis_body_id][3:6]  # 线速度 (world)
                angvel = data.cvel[chassis_body_id][0:3]  # 角速度 (world)
                body_vx = R_chassis[0, :] @ linvel
                body_vy = R_chassis[1, :] @ linvel
                body_omega = R_chassis[2, :] @ angvel

                # PD 速度控制 → 推力 (body frame)
                fx = np.clip(CHASSIS_FORCE_KP * (cur_vx - body_vx), -CHASSIS_MAX_FORCE, CHASSIS_MAX_FORCE)
                fy = np.clip(CHASSIS_FORCE_KP * (cur_vy - body_vy), -CHASSIS_MAX_FORCE, CHASSIS_MAX_FORCE)
                tz = np.clip(CHASSIS_TORQUE_KP * (cur_omega - body_omega), -CHASSIS_MAX_TORQUE, CHASSIS_MAX_TORQUE)

                # body frame → world frame
                force_world = R_chassis.T @ np.array([fx, fy, 0.0])
                torque_world = R_chassis.T @ np.array([0.0, 0.0, tz])

                # 施加到 body
                data.xfrc_applied[chassis_body_id][:3] = torque_world
                data.xfrc_applied[chassis_body_id][3:] = force_world

            # 舵轮视觉同步（MuJoCo 中的轮子是视觉用途）
            for idx in range(4):
                data.ctrl[CTRL_STEER_START + idx] = cur_steer[idx]
                data.ctrl[CTRL_DRIVE_START + idx] = cur_drive[idx]

            # 升降 + 云台
            data.ctrl[CTRL_LIFT] = smoothed_lift
            data.ctrl[CTRL_HEAD_YAW] = target_head_yaw
            data.ctrl[CTRL_HEAD_PITCH] = target_head_pitch

            # 双臂 home pose 保持
            for idx in range(6):
                data.ctrl[CTRL_LEFT_ARM + idx] = ARM_HOME_LEFT[idx]
                data.ctrl[CTRL_RIGHT_ARM + idx] = ARM_HOME_RIGHT[idx]

            # ========= 5. 真实电机控制 =========
            if motor_mgr:
                motor_mgr.send_commands(cur_steer, cur_drive)

            # ========= 6. MuJoCo 步进 =========
            for _ in range(steps_per_ctrl):
                mujoco.mj_step(model, data)

            # ========= 7. 重置 =========
            if reset_request:
                mujoco.mj_resetData(model, data)
                # 重置双臂 home
                for i in range(1, 7):
                    jname = f"left_joint{i}"
                    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
                    if jid >= 0:
                        data.qpos[model.jnt_qposadr[jid]] = ARM_HOME_LEFT[i - 1]
                for i in range(1, 7):
                    jname = f"right_joint{i}"
                    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
                    if jid >= 0:
                        data.qpos[model.jnt_qposadr[jid]] = ARM_HOME_RIGHT[i - 1]
                mujoco.mj_forward(model, data)

                cur_vx = cur_vy = cur_omega = 0.0
                cur_steer[:] = 0.0
                cur_drive[:] = 0.0
                target_lift = smoothed_lift = 0.0
                target_head_yaw = target_head_pitch = 0.0

                if motor_mgr:
                    motor_mgr.stop_all()

                reset_request = False
                print(">>> 场景已重置")

            # ========= 8. HUD 显示 =========
            hud_counter += 1
            if hud_counter % 25 == 0:  # 每 0.5 秒打印一次
                status_parts = [
                    f"vx={cur_vx:+.2f}",
                    f"vy={cur_vy:+.2f}",
                    f"ω={cur_omega:+.2f}",
                ]
                steer_str = " ".join(f"{math.degrees(a):+6.1f}°" for a in cur_steer)
                drive_str = " ".join(f"{s:+6.1f}" for s in cur_drive)

                line = (f"  底盘: {' '.join(status_parts)}  |  "
                        f"转向: [{steer_str}]  |  "
                        f"驱动: [{drive_str}]")

                # 如果有电机反馈，也打印
                if motor_mgr:
                    fb = motor_mgr.get_feedback()
                    fb_steer = " ".join(f"{math.degrees(p):+6.1f}°"
                                        for p in fb['steer_pos'])
                    fb_drive = " ".join(f"{v:+6.1f}" for v in fb['drive_vel'])
                    line += f"\n  反馈: 转向=[{fb_steer}]  驱动=[{fb_drive}]"

                print(line)

            # ========= 9. 同步渲染 =========
            viewer.sync()

            # 控制循环节拍
            elapsed = time.time() - loop_start
            sleep_time = ctrl_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ---- 清理 ----
    print("\n>>> 退出中...")
    if motor_mgr:
        motor_mgr.stop_all()
        time.sleep(0.1)
        motor_mgr.close()
    restore_x11_repeat()
    print(">>> 已退出")


if __name__ == '__main__':
    main()
