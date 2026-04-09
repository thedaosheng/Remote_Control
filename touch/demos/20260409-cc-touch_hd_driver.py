#!/usr/bin/env python3
"""
Touch 设备 Python 驱动层 — 通过 ctypes 直接调用 libHD.so (OpenHaptics HDAPI)

功能：
- 初始化 Touch 设备
- 在 1kHz 伺服循环中读取位置、设置力
- 提供简洁的 Python 回调接口

依赖：系统已安装 libHD.so（3D Systems Touch Device Driver）
无需 OpenHaptics 头文件，纯 ctypes 动态绑定。

用法：
    from touch_hd_driver import TouchDevice

    dev = TouchDevice()
    dev.start()
    # dev.position → [x, y, z] mm
    # dev.set_force([fx, fy, fz])  → 设置力 (N)
    dev.stop()
"""

import ctypes
import ctypes.util
import threading
import time
import numpy as np
from ctypes import c_int, c_uint, c_double, c_char_p, c_void_p, c_ulong, CFUNCTYPE, POINTER, byref

# ============================================================
# OpenHaptics HD API 常量定义
# 来源: OpenHaptics Toolkit - HD API Reference
# ============================================================

# hdGet/hdSet 参数 ID
HD_CURRENT_POSITION = 0x2050          # 当前笔尖位置 [x,y,z] mm
HD_CURRENT_VELOCITY = 0x2051          # 当前速度 [vx,vy,vz] mm/s
HD_CURRENT_TRANSFORM = 0x2052         # 当前 4x4 变换矩阵
HD_CURRENT_ANGULAR_VELOCITY = 0x2053  # 角速度
HD_CURRENT_GIMBAL_ANGLES = 0x2150     # 云台角度 [θ1,θ2,θ3] rad
HD_CURRENT_JOINT_ANGLES = 0x2100      # 关节角度 rad
HD_CURRENT_FORCE = 0x2700             # 当前力 [Fx,Fy,Fz] N
HD_CURRENT_BUTTONS = 0x2000           # 按钮状态

# hdEnable 能力 ID
HD_FORCE_OUTPUT = 0x4000              # 启用力输出
HD_MAX_FORCE_CLAMPING = 0x4001        # 启用最大力限幅（安全）
HD_FORCE_RAMPING = 0x4002             # 启用力渐入（防突跳）

# 默认设备
HD_DEFAULT_DEVICE = None  # NULL = 默认设备

# 调度器回调返回值
HD_CALLBACK_DONE = 0      # 回调执行一次后停止
HD_CALLBACK_CONTINUE = 1  # 回调持续执行（每帧调用）

# 按钮掩码
HD_DEVICE_BUTTON_1 = 0x01  # 笔杆前按钮
HD_DEVICE_BUTTON_2 = 0x02  # 笔杆后按钮

# ============================================================
# HD API 回调函数类型
# ============================================================
# typedef HDCallbackCode (*HDSchedulerCallback)(void *pUserData);
HD_CALLBACK_FUNC = CFUNCTYPE(c_uint, c_void_p)


class TouchDevice:
    """
    Touch 力反馈设备 Python 封装。

    使用示例:
        dev = TouchDevice()
        dev.start()
        while True:
            pos = dev.position  # [x, y, z] in mm
            vel = dev.velocity  # [vx, vy, vz] in mm/s
            btn = dev.buttons   # bitmask
            dev.set_force([0, 0, -0.5])  # 向下 0.5N
        dev.stop()
    """

    def __init__(self, device_name=None):
        """
        初始化 Touch 设备。

        参数:
            device_name: 设备名称，None = 默认设备
        """
        # 加载 HD 运行时库
        self._hd = ctypes.CDLL("libHD.so", mode=ctypes.RTLD_GLOBAL)

        # 定义函数签名
        self._setup_api()

        # 设备状态（在伺服线程和主线程之间共享）
        self._position = np.zeros(3)     # mm
        self._velocity = np.zeros(3)     # mm/s
        self._gimbal = np.zeros(3)       # rad
        self._joint_angles = np.zeros(6) # rad
        self._buttons = 0
        self._force = np.zeros(3)        # N — 要发送的力
        self._lock = threading.Lock()
        self._running = False
        self._device_handle = None
        self._callback_handle = None

        # 安全限制（来自设备 config: NominalMaxForce=3.3, NominalMaxContinuousForce=0.88）
        self.max_force = 3.0             # N — 留 0.3N 安全余量
        self.max_continuous_force = 0.8  # N

        # 初始化设备
        self._init_device(device_name)

    def _setup_api(self):
        """配置 ctypes 函数签名"""
        hd = self._hd

        # HHD hdInitDevice(const char* device_name)
        hd.hdInitDevice.argtypes = [c_char_p]
        hd.hdInitDevice.restype = c_uint

        # void hdMakeCurrentDevice(HHD hHD)
        hd.hdMakeCurrentDevice.argtypes = [c_uint]
        hd.hdMakeCurrentDevice.restype = None

        # void hdEnable(HDuint cap)
        hd.hdEnable.argtypes = [c_uint]
        hd.hdEnable.restype = None

        # void hdDisable(HDuint cap)
        hd.hdDisable.argtypes = [c_uint]
        hd.hdDisable.restype = None

        # void hdBeginFrame(HHD hHD)
        hd.hdBeginFrame.argtypes = [c_uint]
        hd.hdBeginFrame.restype = None

        # void hdEndFrame(HHD hHD)
        hd.hdEndFrame.argtypes = [c_uint]
        hd.hdEndFrame.restype = None

        # void hdGetDoublev(HDuint pname, HDdouble *values)
        hd.hdGetDoublev.argtypes = [c_uint, POINTER(c_double)]
        hd.hdGetDoublev.restype = None

        # void hdSetDoublev(HDuint pname, HDdouble *values)
        hd.hdSetDoublev.argtypes = [c_uint, POINTER(c_double)]
        hd.hdSetDoublev.restype = None

        # void hdGetIntegerv(HDuint pname, int *values)
        hd.hdGetIntegerv.argtypes = [c_uint, POINTER(c_int)]
        hd.hdGetIntegerv.restype = None

        # HDSchedulerHandle hdScheduleAsynchronous(callback, userData, priority)
        hd.hdScheduleAsynchronous.argtypes = [HD_CALLBACK_FUNC, c_void_p, c_uint]
        hd.hdScheduleAsynchronous.restype = c_ulong

        # void hdStartScheduler()
        hd.hdStartScheduler.argtypes = []
        hd.hdStartScheduler.restype = None

        # void hdStopScheduler()
        hd.hdStopScheduler.argtypes = []
        hd.hdStopScheduler.restype = None

        # void hdUnschedule(HDSchedulerHandle handle)
        hd.hdUnschedule.argtypes = [c_ulong]
        hd.hdUnschedule.restype = None

        # void hdDisableDevice(HHD hHD)
        hd.hdDisableDevice.argtypes = [c_uint]
        hd.hdDisableDevice.restype = None

    def _init_device(self, device_name):
        """初始化 Touch 硬件设备"""
        name = device_name.encode() if device_name else HD_DEFAULT_DEVICE
        self._device_handle = self._hd.hdInitDevice(name)
        print(f"[TouchDevice] 设备初始化成功, handle={self._device_handle}")

        # 启用力输出 + 安全限幅
        self._hd.hdEnable(HD_FORCE_OUTPUT)
        self._hd.hdEnable(HD_MAX_FORCE_CLAMPING)
        self._hd.hdEnable(HD_FORCE_RAMPING)
        print(f"[TouchDevice] 力输出已启用 (最大 {self.max_force}N)")

    def _servo_callback(self, pUserData):
        """
        1kHz 伺服循环回调 — 这是力渲染的核心！
        在 OpenHaptics 调度器线程中执行，每毫秒调用一次。

        职责：
        1. 读取设备当前位置/速度/按钮状态
        2. 将主线程设置的力写入设备
        """
        hd = self._hd
        handle = self._device_handle

        hd.hdBeginFrame(handle)

        # --- 读取设备状态 ---
        pos = (c_double * 3)()
        vel = (c_double * 3)()
        gimbal = (c_double * 3)()
        buttons = c_int(0)

        hd.hdGetDoublev(HD_CURRENT_POSITION, pos)
        hd.hdGetDoublev(HD_CURRENT_VELOCITY, vel)
        hd.hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal)
        hd.hdGetIntegerv(HD_CURRENT_BUTTONS, byref(buttons))

        # --- 写入力 ---
        with self._lock:
            force = self._force.copy()

        # 安全限幅
        force_mag = np.linalg.norm(force)
        if force_mag > self.max_force:
            force = force / force_mag * self.max_force

        f = (c_double * 3)(force[0], force[1], force[2])
        hd.hdSetDoublev(HD_CURRENT_FORCE, f)

        hd.hdEndFrame(handle)

        # --- 更新共享状态 ---
        with self._lock:
            self._position[:] = [pos[0], pos[1], pos[2]]
            self._velocity[:] = [vel[0], vel[1], vel[2]]
            self._gimbal[:] = [gimbal[0], gimbal[1], gimbal[2]]
            self._buttons = buttons.value

        if self._running:
            return HD_CALLBACK_CONTINUE
        else:
            return HD_CALLBACK_DONE

    def start(self):
        """启动 1kHz 伺服调度器"""
        self._running = True

        # 必须保持对回调的引用，防止被 GC 回收
        self._callback_ref = HD_CALLBACK_FUNC(self._servo_callback)
        self._callback_handle = self._hd.hdScheduleAsynchronous(
            self._callback_ref, None, 0  # priority=0 (highest)
        )

        self._hd.hdStartScheduler()
        print("[TouchDevice] 伺服调度器已启动 (1kHz)")

    def stop(self):
        """停止伺服调度器并释放设备"""
        self._running = False
        time.sleep(0.1)  # 等待最后一个回调完成

        if self._callback_handle is not None:
            self._hd.hdUnschedule(self._callback_handle)
        self._hd.hdStopScheduler()

        # 关闭力输出
        self._hd.hdDisable(HD_FORCE_OUTPUT)
        self._hd.hdDisableDevice(self._device_handle)
        print("[TouchDevice] 设备已安全关闭")

    @property
    def position(self):
        """获取笔尖位置 [x, y, z] 单位 mm"""
        with self._lock:
            return self._position.copy()

    @property
    def velocity(self):
        """获取笔尖速度 [vx, vy, vz] 单位 mm/s"""
        with self._lock:
            return self._velocity.copy()

    @property
    def gimbal_angles(self):
        """获取云台角度 [θ1, θ2, θ3] 单位 rad"""
        with self._lock:
            return self._gimbal.copy()

    @property
    def buttons(self):
        """获取按钮状态 (bitmask: 1=前按钮, 2=后按钮)"""
        with self._lock:
            return self._buttons

    def set_force(self, force):
        """
        设置输出力 [Fx, Fy, Fz] 单位 N

        坐标系说明 (OpenHaptics 标准):
        - X: 向右为正
        - Y: 向上为正
        - Z: 朝向用户为正

        安全限制: 自动限幅到 max_force (默认 3.0N)
        """
        with self._lock:
            self._force[:] = force


# ============================================================
# 快速测试：读取位置并打印
# ============================================================
if __name__ == "__main__":
    print("=" * 50)
    print("Touch 设备连接测试")
    print("=" * 50)

    dev = TouchDevice()
    dev.start()

    try:
        print("\n移动 Touch 笔查看位置数据 (按 Ctrl+C 退出):\n")
        while True:
            pos = dev.position
            vel = dev.velocity
            btn = dev.buttons
            btn_str = ""
            if btn & HD_DEVICE_BUTTON_1:
                btn_str += "[前]"
            if btn & HD_DEVICE_BUTTON_2:
                btn_str += "[后]"
            print(f"\r  位置: X={pos[0]:7.1f} Y={pos[1]:7.1f} Z={pos[2]:7.1f} mm  "
                  f"速度: {np.linalg.norm(vel):6.1f} mm/s  "
                  f"按钮: {btn_str or '无':6s}", end="", flush=True)
            time.sleep(0.05)  # 20Hz 打印频率
    except KeyboardInterrupt:
        print("\n\n停止中...")
    finally:
        dev.stop()
