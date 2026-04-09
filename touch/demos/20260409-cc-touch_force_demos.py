#!/usr/bin/env python3
"""
Touch 力反馈笔 — 8 种力渲染真机交互 Demo

按照手册（2026-03-29）的正确方式加载驱动：
1. 先 dlopen fakelibs/libncurses.so.5
2. 再 dlopen libHD.so（从 /tmp/patched_lib 优先）
3. 通过 ctypes 调用 HD API

运行方式：
    LD_LIBRARY_PATH=/tmp/patched_lib:/tmp/fakelibs:/usr/lib \
    python3 touch/demos/20260409-cc-touch_force_demos.py

操作方式：
    - 数字键 1-8 切换力渲染效果
    - 0 = 关闭力输出（自由移动）
    - q = 退出
    - 移动笔杆感受不同力效果

8 种力渲染效果：
    1. 刚度墙    — Y=80mm 处有一面硬墙，下压感受阻力
    2. 弹簧回中  — 弹簧把笔拉向工作空间中心
    3. 粘滞力场  — 像在蜂蜜里移动，速度越快阻力越大
    4. 表面摩擦  — Y=80mm 平面 + 水平方向摩擦力
    5. 磁吸效果  — 靠近中心点自动吸附
    6. 重力井    — 空间中的引力陷阱
    7. 振动纹理  — 平面上叠加正弦波振动
    8. 虚拟引导槽 — 只能沿 X 轴自由移动，Y/Z 偏离有恢复力

依赖: 系统已安装 libHD.so + 补丁库
"""

import ctypes
import ctypes.util
import os
import threading
import time
import sys
import select
import termios
import tty
import math
import numpy as np
from ctypes import (c_int, c_uint, c_double, c_char_p, c_void_p,
                    c_ulong, c_ushort, CFUNCTYPE, POINTER, byref)

# ctypes RTLD 标志（从 os 模块获取）
RTLD_NOW = os.RTLD_NOW if hasattr(os, 'RTLD_NOW') else 0x2
RTLD_GLOBAL = ctypes.RTLD_GLOBAL

# ============================================================
# HD API 常量（来自手册第 6 页，已验证）
# ============================================================
HD_CURRENT_BUTTONS      = 0x2000  # int, 位掩码
HD_CURRENT_POSITION     = 0x2050  # double[3], mm
HD_CURRENT_VELOCITY     = 0x2051  # double[3], mm/s
HD_CURRENT_TRANSFORM    = 0x2052  # double[16], 4x4 矩阵
HD_CURRENT_JOINT_ANGLES = 0x2100  # double[3], rad
HD_CURRENT_GIMBAL_ANGLES= 0x2150  # double[3], rad
HD_CURRENT_FORCE        = 0x2700  # double[3], N (写, 最大 3.3N)
HD_FORCE_OUTPUT         = 0x4000  # hdEnable() 参数
HD_CALLBACK_DONE        = 0       # 回调返回：结束
HD_CALLBACK_CONTINUE    = 1       # 回调返回：继续

# 按钮掩码
BTN_WHITE = 0x01  # 白色按钮（笔尖侧）
BTN_GREY  = 0x02  # 灰色按钮（笔尾侧）

# 回调函数类型
HD_CALLBACK_FUNC = CFUNCTYPE(c_uint, c_void_p)

# ============================================================
# 坐标系（来自手册第 3 页）
# 原点: 底座中心
# X: 向右 (mm), 工作范围约 ±100mm
# Y: 向上 (mm), 工作范围约 0~200mm
# Z: 向用户方向 (mm), 工作范围约 ±100mm
# ============================================================

# 安全力上限
MAX_FORCE = 2.5  # N — 留余量（设备额定 3.3N）


def clamp_force(force):
    """安全限幅：力向量大小不超过 MAX_FORCE"""
    mag = np.linalg.norm(force)
    if mag > MAX_FORCE:
        force = force / mag * MAX_FORCE
    return force


# ============================================================
# 8 种力渲染算法
# ============================================================

def force_off(pos, vel):
    """效果 0: 关闭力输出 — 自由移动"""
    return np.zeros(3)


def force_stiffness_wall(pos, vel):
    """
    效果 1: 刚度墙
    在 Y=80mm 处设置一面硬墙。
    当笔尖低于 80mm 时（下压），产生向上的恢复力。
    力 = k * penetration
    """
    wall_y = 80.0  # mm
    k = 0.5        # N/mm — 适中刚度，不会太硬
    force = np.zeros(3)
    if pos[1] < wall_y:
        penetration = wall_y - pos[1]
        force[1] = k * penetration  # 向上推
    return clamp_force(force)


def force_spring_center(pos, vel):
    """
    效果 2: 弹簧回中
    在工作空间中心 (0, 120, 0) 设置锚点，
    弹簧力把笔拉向中心。
    F = -k * displacement
    """
    center = np.array([0.0, 120.0, 0.0])  # mm
    k = 0.01   # N/mm — 软弹簧
    b = 0.001  # N/(mm/s) — 轻微阻尼防振荡
    displacement = pos - center
    force = -k * displacement - b * vel
    return clamp_force(force)


def force_viscosity(pos, vel):
    """
    效果 3: 粘滞力场
    与速度成正比的阻力，模拟在蜂蜜中移动。
    F = -η * velocity
    """
    eta = 0.008  # N/(mm/s) — 中等粘滞
    force = -eta * vel
    return clamp_force(force)


def force_surface_friction(pos, vel):
    """
    效果 4: 表面摩擦
    Y=80mm 处有一个表面：
    - 法向力: 弹簧（防止穿过）
    - 切向力: 库仑摩擦（水平滑动有阻力）
    """
    wall_y = 80.0
    k = 0.8      # N/mm — 法向刚度
    mu = 0.6     # 摩擦系数
    force = np.zeros(3)

    if pos[1] < wall_y:
        penetration = wall_y - pos[1]
        fn = k * penetration  # 法向力
        force[1] = fn

        # 切向摩擦力（XZ 平面）
        v_tangent = np.array([vel[0], 0.0, vel[2]])
        v_speed = np.linalg.norm(v_tangent)
        if v_speed > 1.0:  # mm/s — 死区防抖
            friction_mag = mu * fn
            force[0] -= friction_mag * vel[0] / v_speed
            force[2] -= friction_mag * vel[2] / v_speed

    return clamp_force(force)


def force_magnetic_snap(pos, vel):
    """
    效果 5: 磁吸效果
    在 (0, 120, 0) 设置磁铁，靠近时吸附。
    """
    magnet = np.array([0.0, 120.0, 0.0])
    snap_radius = 40.0  # mm — 吸附半径
    strength = 3000.0   # 力场强度参数

    r = magnet - pos
    dist = np.linalg.norm(r)
    if dist < 2.0:  # 极近处饱和
        dist = 2.0
    if dist > snap_radius:
        return np.zeros(3)

    # 衰减：边缘处力为零
    decay = max(0.0, 1.0 - (dist / snap_radius) ** 2)
    force_mag = strength / (dist ** 2) * decay
    force = force_mag * r / np.linalg.norm(r)
    return clamp_force(force)


def force_gravity_well(pos, vel):
    """
    效果 6: 重力井
    引力场，力随距离减小而增大（但在极近处饱和）。
    """
    well_center = np.array([0.0, 120.0, 0.0])
    well_radius = 60.0   # mm
    G = 2000.0            # 引力常数

    r = well_center - pos
    dist = np.linalg.norm(r)
    if dist < 3.0:
        dist = 3.0
    if dist > well_radius:
        return np.zeros(3)

    force_mag = G / (dist ** 2)
    force = force_mag * r / np.linalg.norm(r)
    return clamp_force(force)


def force_vibration_texture(pos, vel):
    """
    效果 7: 振动纹理
    Y=80mm 表面 + 沿 X 方向的正弦波力调制。
    水平滑动时能感受到"搓衣板"般的凹凸。
    """
    wall_y = 80.0
    k = 0.5       # N/mm
    period = 8.0  # mm — 凹凸周期
    amplitude = 0.4  # N — 振动幅值
    force = np.zeros(3)

    if pos[1] < wall_y:
        penetration = wall_y - pos[1]
        base_fn = k * penetration
        # 叠加位置相关的正弦调制
        texture = amplitude * math.sin(2 * math.pi * pos[0] / period)
        force[1] = base_fn + texture

    return clamp_force(force)


def force_guide_channel(pos, vel):
    """
    效果 8: 虚拟引导槽
    笔只能沿 X 轴自由移动。
    Y 偏离 120mm 或 Z 偏离 0mm 时有恢复力。
    """
    target_y = 120.0  # mm — 沟槽 Y 位置
    target_z = 0.0    # mm — 沟槽 Z 位置
    channel_width = 5.0  # mm — 沟槽半宽
    k = 0.3  # N/mm

    force = np.zeros(3)

    dy = pos[1] - target_y
    if abs(dy) > channel_width:
        if dy > 0:
            force[1] = -k * (dy - channel_width)
        else:
            force[1] = -k * (dy + channel_width)

    dz = pos[2] - target_z
    if abs(dz) > channel_width:
        if dz > 0:
            force[2] = -k * (dz - channel_width)
        else:
            force[2] = -k * (dz + channel_width)

    return clamp_force(force)


# 效果列表
EFFECTS = [
    ("关闭力输出（自由移动）",  force_off),
    ("刚度墙 — Y=80mm 下压感受硬面", force_stiffness_wall),
    ("弹簧回中 — 拉向中心点",       force_spring_center),
    ("粘滞力场 — 蜂蜜中移动",       force_viscosity),
    ("表面摩擦 — 平面+库仑摩擦",    force_surface_friction),
    ("磁吸效果 — 靠近中心被吸住",   force_magnetic_snap),
    ("重力井 — 引力陷阱",           force_gravity_well),
    ("振动纹理 — 搓衣板触感",       force_vibration_texture),
    ("虚拟引导槽 — 只能沿X轴走",    force_guide_channel),
]


# ============================================================
# Touch 设备驱动（按手册正确加载）
# ============================================================

class TouchDevice:
    def __init__(self):
        # 按手册步骤：先加载 fakelibs，再加载 libHD.so
        # 必须用 RTLD_GLOBAL 让符号全局可见
        self._ncurses = ctypes.CDLL(
            "/tmp/fakelibs/libncurses.so.5",
            mode=RTLD_NOW | RTLD_GLOBAL
        )
        self._hd = ctypes.CDLL(
            "/usr/lib/libHD.so",
            mode=RTLD_NOW | RTLD_GLOBAL
        )
        self._setup_api()

        # 共享状态
        self._position = np.zeros(3)
        self._velocity = np.zeros(3)
        self._buttons = 0
        self._force = np.zeros(3)
        self._lock = threading.Lock()
        self._running = False
        self._effect_func = force_off  # 当前力渲染函数

        # 初始化设备
        self._init_device()

    def _setup_api(self):
        """配置 HD API ctypes 签名"""
        hd = self._hd
        hd.hdInitDevice.argtypes = [c_char_p]
        hd.hdInitDevice.restype = c_uint
        hd.hdEnable.argtypes = [c_uint]
        hd.hdEnable.restype = None
        hd.hdDisable.argtypes = [c_uint]
        hd.hdDisable.restype = None
        hd.hdBeginFrame.argtypes = [c_uint]
        hd.hdBeginFrame.restype = None
        hd.hdEndFrame.argtypes = [c_uint]
        hd.hdEndFrame.restype = None
        hd.hdGetDoublev.argtypes = [c_uint, POINTER(c_double)]
        hd.hdGetDoublev.restype = None
        hd.hdSetDoublev.argtypes = [c_uint, POINTER(c_double)]
        hd.hdSetDoublev.restype = None
        hd.hdGetIntegerv.argtypes = [c_uint, POINTER(c_int)]
        hd.hdGetIntegerv.restype = None
        hd.hdScheduleAsynchronous.argtypes = [HD_CALLBACK_FUNC, c_void_p, c_ushort]
        hd.hdScheduleAsynchronous.restype = c_ulong
        hd.hdStartScheduler.argtypes = []
        hd.hdStartScheduler.restype = None
        hd.hdStopScheduler.argtypes = []
        hd.hdStopScheduler.restype = None
        hd.hdUnschedule.argtypes = [c_ulong]
        hd.hdUnschedule.restype = None
        hd.hdDisableDevice.argtypes = [c_uint]
        hd.hdDisableDevice.restype = None

    def _init_device(self):
        """初始化设备（按手册第 5-6 页）"""
        # 直接初始化（NULL = 使用 Default Device.config）
        self._handle = self._hd.hdInitDevice(None)
        print(f"[Touch] 设备初始化成功, handle={self._handle}")

        # 关键: 启用力输出
        self._hd.hdEnable(HD_FORCE_OUTPUT)
        print("[Touch] 力输出已启用 ✅")

    def _servo_callback(self, pUserData):
        """1kHz 伺服循环回调"""
        hd = self._hd
        h = self._handle

        hd.hdBeginFrame(h)

        # 读取位置和速度
        pos = (c_double * 3)()
        vel = (c_double * 3)()
        buttons = c_int(0)
        hd.hdGetDoublev(HD_CURRENT_POSITION, pos)
        hd.hdGetDoublev(HD_CURRENT_VELOCITY, vel)
        hd.hdGetIntegerv(HD_CURRENT_BUTTONS, byref(buttons))

        pos_np = np.array([pos[0], pos[1], pos[2]])
        vel_np = np.array([vel[0], vel[1], vel[2]])

        # 调用当前力渲染函数
        with self._lock:
            func = self._effect_func
        force = func(pos_np, vel_np)
        force = clamp_force(force)

        # 写入力
        f = (c_double * 3)(force[0], force[1], force[2])
        hd.hdSetDoublev(HD_CURRENT_FORCE, f)

        hd.hdEndFrame(h)

        # 更新共享状态
        with self._lock:
            self._position[:] = pos_np
            self._velocity[:] = vel_np
            self._buttons = buttons.value
            self._force[:] = force

        return HD_CALLBACK_CONTINUE if self._running else HD_CALLBACK_DONE

    def start(self):
        self._running = True
        self._cb_ref = HD_CALLBACK_FUNC(self._servo_callback)
        self._cb_handle = self._hd.hdScheduleAsynchronous(self._cb_ref, None, 0)
        self._hd.hdStartScheduler()  # 返回 0x103 可忽略（手册说明）
        print("[Touch] 伺服调度器已启动 (~1kHz)")

    def stop(self):
        self._running = False
        time.sleep(0.1)
        self._hd.hdStopScheduler()
        self._hd.hdDisable(HD_FORCE_OUTPUT)
        self._hd.hdDisableDevice(self._handle)
        print("[Touch] 设备已安全关闭")

    def set_effect(self, func):
        with self._lock:
            self._effect_func = func

    @property
    def position(self):
        with self._lock:
            return self._position.copy()

    @property
    def velocity(self):
        with self._lock:
            return self._velocity.copy()

    @property
    def buttons(self):
        with self._lock:
            return self._buttons

    @property
    def current_force(self):
        with self._lock:
            return self._force.copy()


# ============================================================
# 终端键盘输入（非阻塞）
# ============================================================

def get_key_nonblocking():
    """非阻塞读取一个按键，无输入返回 None"""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


# ============================================================
# 主程序
# ============================================================

def main():
    print("=" * 60)
    print("  Touch 力反馈笔 — 8 种力渲染真机交互 Demo")
    print("=" * 60)
    print()
    print("⚠️  请确保笔已从底座上取下，拿在手里！")
    print()

    # 打印效果列表
    for i, (name, _) in enumerate(EFFECTS):
        print(f"  [{i}] {name}")
    print()
    print("  [q] 退出")
    print()

    # 初始化设备
    dev = TouchDevice()
    dev.start()
    time.sleep(0.5)  # 等待伺服稳定

    current_effect = 0
    dev.set_effect(EFFECTS[current_effect][1])
    print(f"\n▶ 当前效果: [{current_effect}] {EFFECTS[current_effect][0]}\n")

    # 设置终端为原始模式（非阻塞读键盘）
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        while True:
            # 检查键盘输入
            key = get_key_nonblocking()
            if key == 'q':
                break
            elif key and key.isdigit():
                idx = int(key)
                if 0 <= idx < len(EFFECTS):
                    current_effect = idx
                    dev.set_effect(EFFECTS[idx][1])
                    print(f"\n▶ 切换到效果 [{idx}] {EFFECTS[idx][0]}\n")

            # 显示实时状态
            pos = dev.position
            vel = dev.velocity
            force = dev.current_force
            btn = dev.buttons
            f_mag = np.linalg.norm(force)

            btn_str = ""
            if btn & BTN_WHITE:
                btn_str += "⚪"
            if btn & BTN_GREY:
                btn_str += "🔘"

            print(f"\r  效果[{current_effect}] "
                  f"位置: X={pos[0]:6.1f} Y={pos[1]:6.1f} Z={pos[2]:6.1f} mm  "
                  f"力: {f_mag:.2f}N [{force[0]:+.2f} {force[1]:+.2f} {force[2]:+.2f}]  "
                  f"按钮:{btn_str or '--':4s}",
                  end="", flush=True)

            time.sleep(0.05)  # 20Hz 显示刷新

    except KeyboardInterrupt:
        pass
    finally:
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("\n\n正在安全关闭...")
        dev.set_effect(force_off)  # 先清零力
        time.sleep(0.1)
        dev.stop()
        print("再见！")


if __name__ == "__main__":
    main()
