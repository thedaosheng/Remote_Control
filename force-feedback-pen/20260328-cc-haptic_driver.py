#!/usr/bin/env python3
"""
3D Systems Touch 力反馈笔 Python ctypes 驱动
============================================
通过直接调用 OpenHaptics HD API (libHD.so) 实现：
1. 设备初始化与校准
2. 读取笔尖位置 (X, Y, Z)、关节角度、万向节角度
3. 读取按钮状态
4. 发送力反馈输出 (Fx, Fy, Fz)

依赖：OpenHaptics SDK 3.4 + Touch Device Driver 2022
作者：Claude Code
日期：2026-03-28
"""

import ctypes
import ctypes.util
import os
import sys
import time
import signal
import threading
from dataclasses import dataclass, field
from typing import Optional, Tuple

# ============================================================================
# HD API 常量定义（来自 hdDefines.h）
# ============================================================================

# 错误码
HD_SUCCESS = 0x0000
HD_DEVICE_FAULT = 0x0300
HD_COMM_ERROR = 0x0302

# Get 参数 - 原始值
HD_CURRENT_BUTTONS = 0x2000
HD_CURRENT_SAFETY_SWITCH = 0x2001
HD_CURRENT_INKWELL_SWITCH = 0x2002
HD_CURRENT_ENCODER_VALUES = 0x2010

# Get 参数 - 笛卡尔空间值
HD_CURRENT_POSITION = 0x2050       # 当前笔尖位置 [x, y, z] (mm)
HD_CURRENT_VELOCITY = 0x2051       # 当前速度 [vx, vy, vz] (mm/s)
HD_CURRENT_TRANSFORM = 0x2052      # 当前变换矩阵 (4x4)
HD_CURRENT_ANGULAR_VELOCITY = 0x2053  # 当前角速度
HD_CURRENT_JACOBIAN = 0x2054       # 雅可比矩阵

# Get 参数 - 关节空间值
HD_CURRENT_JOINT_ANGLES = 0x2100   # 当前关节角度 [j1, j2, j3] (rad)
HD_CURRENT_GIMBAL_ANGLES = 0x2150  # 当前万向节角度 [g1, g2, g3] (rad)

# Get 参数 - 上一帧值
HD_LAST_BUTTONS = 0x2200
HD_LAST_POSITION = 0x2250

# 设备信息
HD_VERSION = 0x2500
HD_DEVICE_MODEL_TYPE = 0x2501
HD_DEVICE_DRIVER_VERSION = 0x2502
HD_DEVICE_VENDOR = 0x2503
HD_DEVICE_SERIAL_NUMBER = 0x2504

# 设备属性
HD_MAX_WORKSPACE_DIMENSIONS = 0x2550   # 最大工作空间 [minX,minY,minZ, maxX,maxY,maxZ]
HD_USABLE_WORKSPACE_DIMENSIONS = 0x2551
HD_INPUT_DOF = 0x2553                  # 输入自由度
HD_OUTPUT_DOF = 0x2554                 # 输出自由度
HD_UPDATE_RATE = 0x2600                # 伺服循环更新频率
HD_NOMINAL_MAX_FORCE = 0x2603          # 标称最大力 (N)
HD_NOMINAL_MAX_CONTINUOUS_FORCE = 0x2604  # 标称最大持续力 (N)
HD_NOMINAL_MAX_STIFFNESS = 0x2602      # 标称最大刚度 (N/mm)

# Set 参数 - 力反馈输出
HD_CURRENT_FORCE = 0x2700              # 当前力输出 [Fx, Fy, Fz] (N)
HD_CURRENT_TORQUE = 0x2701             # 当前扭矩输出

# 使能/禁用选项
HD_FORCE_OUTPUT = 0x4000               # 力输出使能
HD_MAX_FORCE_CLAMPING = 0x4001         # 最大力限幅
HD_FORCE_RAMPING = 0x4002              # 力渐变

# 回调返回码
HD_CALLBACK_DONE = 0
HD_CALLBACK_CONTINUE = 1

# 按钮掩码
HD_DEVICE_BUTTON_1 = (1 << 0)  # 灰色按钮
HD_DEVICE_BUTTON_2 = (1 << 1)  # 白色按钮

# 校准
HD_CALIBRATION_OK = 0x5000
HD_CALIBRATION_NEEDS_UPDATE = 0x5001
HD_CALIBRATION_ENCODER_RESET = (1 << 0)
HD_CALIBRATION_AUTO = (1 << 1)
HD_CALIBRATION_INKWELL = (1 << 2)

# 默认设备
HD_DEFAULT_DEVICE = None

# 无效句柄
HD_INVALID_HANDLE = 0xFFFFFFFF

# 调度器优先级
HD_DEFAULT_SCHEDULER_PRIORITY = 32767
HD_MAX_SCHEDULER_PRIORITY = 65535
HD_MIN_SCHEDULER_PRIORITY = 0


# ============================================================================
# 数据结构
# ============================================================================

@dataclass
class HapticState:
    """力反馈笔的实时状态数据"""
    # 位置 (mm)
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # 速度 (mm/s)
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # 关节角度 (rad)
    joint_angles: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # 万向节角度 (rad) - 对应笔尖的 roll/pitch/yaw
    gimbal_angles: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # 变换矩阵 (4x4, 行优先)
    transform: list = field(default_factory=lambda: [0.0] * 16)
    # 按钮状态
    button1: bool = False   # 灰色按钮（前方）
    button2: bool = False   # 白色按钮（后方）
    # 编码器原始值
    encoder_values: Tuple[int, int, int] = (0, 0, 0)
    # 当前输出力 (N)
    force: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # 时间戳
    timestamp: float = 0.0
    # 安全开关（笔是否在支架上）
    inkwell: bool = False


@dataclass
class DeviceInfo:
    """设备硬件信息"""
    model: str = ""
    vendor: str = ""
    serial: str = ""
    driver_version: str = ""
    firmware_version: str = ""
    input_dof: int = 0
    output_dof: int = 0
    max_force: float = 0.0
    max_continuous_force: float = 0.0
    max_stiffness: float = 0.0
    update_rate: float = 0.0
    workspace_min: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    workspace_max: Tuple[float, float, float] = (0.0, 0.0, 0.0)


# ============================================================================
# C 回调函数类型定义
# ============================================================================

# HDCallbackCode (*HDSchedulerCallback)(void *pUserData)
# 在 ctypes 中定义回调函数签名
HDCALLBACK = ctypes.CFUNCTYPE(ctypes.c_uint, ctypes.c_void_p)


# ============================================================================
# HDErrorInfo 结构体
# ============================================================================

class HDErrorInfo(ctypes.Structure):
    """对应 C 结构体 HDErrorInfo"""
    _fields_ = [
        ("errorCode", ctypes.c_uint),
        ("internalErrorCode", ctypes.c_int),
        ("hHD", ctypes.c_uint),
    ]


# ============================================================================
# HapticDevice 主类
# ============================================================================

class HapticDevice:
    """
    3D Systems Touch 力反馈笔驱动类

    用法：
        device = HapticDevice()
        device.initialize()        # 初始化设备
        device.start_scheduler()   # 启动伺服循环

        # 读取状态
        state = device.get_state()
        print(f"位置: {state.position}")

        # 发送力反馈
        device.set_force(0.0, 0.0, -0.5)  # 向下 0.5N

        device.stop()              # 停止并清理
    """

    def __init__(self, lib_path: Optional[str] = None):
        """
        初始化驱动，加载 libHD.so 动态库

        参数:
            lib_path: libHD.so 所在目录路径，默认使用本地安装路径
        """
        self._hd = None           # ctypes 库句柄
        self._device_handle = HD_INVALID_HANDLE  # 设备句柄
        self._scheduler_running = False
        self._state = HapticState()
        self._device_info = DeviceInfo()
        self._target_force = (ctypes.c_double * 3)(0.0, 0.0, 0.0)  # 目标力
        self._servo_callback_ref = None  # 保持回调引用防止 GC
        self._lock = threading.Lock()
        self._running = False

        # 确定库文件路径（libHD.so 所在目录）
        if lib_path is None:
            # 优先尝试系统路径 /usr/lib
            if os.path.exists('/usr/lib/libHD.so'):
                lib_path = '/usr/lib'
            else:
                lib_path = os.path.normpath(os.path.join(
                    os.path.dirname(os.path.abspath(__file__)),
                    '..', 'haptics_install', 'local', 'lib'
                ))

        # ★ 关键：必须将补丁库路径排在最前，否则加载系统未打补丁的版本
        # /tmp/patched_lib/ 中的 libPhantomIOLib42.so 包含三个必要的二进制补丁
        # /tmp/fakelibs/    中的 libncurses.so.5 是空壳库，满足链接依赖
        patched  = '/tmp/patched_lib'
        fakelibs = '/tmp/fakelibs'
        current_ld = os.environ.get('LD_LIBRARY_PATH', '')
        new_ld = f"{patched}:{fakelibs}:{lib_path}:{current_ld}".rstrip(':')
        os.environ['LD_LIBRARY_PATH'] = new_ld

        # 预加载依赖：必须用 RTLD_GLOBAL，使符号全局可见
        # 顺序：ncurses → 补丁驱动 → libHD（最后）
        # libHD.so 依赖 stdscr 等 ncurses 符号；必须先加载真实 ncurses.so.6
        for preload in [
            '/lib/x86_64-linux-gnu/libtinfo.so.6',            # ncurses 的底层依赖
            '/lib/x86_64-linux-gnu/libncurses.so.6',          # 提供 stdscr 等符号
            f'{patched}/libPhantomIOLib42.so',                 # 打过三个补丁的驱动
        ]:
            if os.path.exists(preload):
                ctypes.CDLL(preload, mode=ctypes.RTLD_GLOBAL)

        # 加载 libHD.so
        hd_path = os.path.join(lib_path, 'libHD.so')
        if not os.path.exists(hd_path):
            raise FileNotFoundError(f"找不到 libHD.so: {hd_path}")

        print(f"[HapticDevice] 加载库文件: {hd_path}")
        self._hd = ctypes.CDLL(hd_path)

        # 设置函数签名
        self._setup_function_signatures()

    def _setup_function_signatures(self):
        """设置所有 HD API 函数的参数和返回值类型"""
        hd = self._hd

        # hdInitDevice(const char* pConfigName) -> HHD (unsigned int)
        hd.hdInitDevice.argtypes = [ctypes.c_char_p]
        hd.hdInitDevice.restype = ctypes.c_uint

        # hdDisableDevice(HHD hHD) -> void
        hd.hdDisableDevice.argtypes = [ctypes.c_uint]
        hd.hdDisableDevice.restype = None

        # hdGetCurrentDevice() -> HHD
        hd.hdGetCurrentDevice.argtypes = []
        hd.hdGetCurrentDevice.restype = ctypes.c_uint

        # hdMakeCurrentDevice(HHD hHD) -> void
        hd.hdMakeCurrentDevice.argtypes = [ctypes.c_uint]
        hd.hdMakeCurrentDevice.restype = None

        # hdBeginFrame(HHD hHD) / hdEndFrame(HHD hHD)
        hd.hdBeginFrame.argtypes = [ctypes.c_uint]
        hd.hdBeginFrame.restype = None
        hd.hdEndFrame.argtypes = [ctypes.c_uint]
        hd.hdEndFrame.restype = None

        # hdGetError() -> HDErrorInfo
        hd.hdGetError.argtypes = []
        hd.hdGetError.restype = HDErrorInfo

        # hdGetErrorString(HDerror) -> const char*
        hd.hdGetErrorString.argtypes = [ctypes.c_uint]
        hd.hdGetErrorString.restype = ctypes.c_char_p

        # hdEnable / hdDisable / hdIsEnabled
        hd.hdEnable.argtypes = [ctypes.c_uint]
        hd.hdEnable.restype = None
        hd.hdDisable.argtypes = [ctypes.c_uint]
        hd.hdDisable.restype = None
        hd.hdIsEnabled.argtypes = [ctypes.c_uint]
        hd.hdIsEnabled.restype = ctypes.c_ubyte

        # hdGetDoublev(HDenum, HDdouble*)
        hd.hdGetDoublev.argtypes = [ctypes.c_uint, ctypes.POINTER(ctypes.c_double)]
        hd.hdGetDoublev.restype = None

        # hdGetIntegerv(HDenum, HDint*)
        hd.hdGetIntegerv.argtypes = [ctypes.c_uint, ctypes.POINTER(ctypes.c_int)]
        hd.hdGetIntegerv.restype = None

        # hdGetLongv(HDenum, HDlong*)
        hd.hdGetLongv.argtypes = [ctypes.c_uint, ctypes.POINTER(ctypes.c_long)]
        hd.hdGetLongv.restype = None

        # hdGetString(HDenum) -> const char*
        hd.hdGetString.argtypes = [ctypes.c_uint]
        hd.hdGetString.restype = ctypes.c_char_p

        # hdSetDoublev(HDenum, const HDdouble*)
        hd.hdSetDoublev.argtypes = [ctypes.c_uint, ctypes.POINTER(ctypes.c_double)]
        hd.hdSetDoublev.restype = None

        # hdStartScheduler / hdStopScheduler
        hd.hdStartScheduler.argtypes = []
        hd.hdStartScheduler.restype = None
        hd.hdStopScheduler.argtypes = []
        hd.hdStopScheduler.restype = None

        # hdSetSchedulerRate(HDulong)
        hd.hdSetSchedulerRate.argtypes = [ctypes.c_ulong]
        hd.hdSetSchedulerRate.restype = None

        # hdScheduleAsynchronous(callback, userData, priority) -> HDSchedulerHandle
        hd.hdScheduleAsynchronous.argtypes = [HDCALLBACK, ctypes.c_void_p, ctypes.c_ushort]
        hd.hdScheduleAsynchronous.restype = ctypes.c_ulong

        # hdScheduleSynchronous(callback, userData, priority)
        hd.hdScheduleSynchronous.argtypes = [HDCALLBACK, ctypes.c_void_p, ctypes.c_ushort]
        hd.hdScheduleSynchronous.restype = None

        # hdUnschedule(HDSchedulerHandle)
        hd.hdUnschedule.argtypes = [ctypes.c_ulong]
        hd.hdUnschedule.restype = None

        # hdCheckCalibration() -> HDenum
        hd.hdCheckCalibration.argtypes = []
        hd.hdCheckCalibration.restype = ctypes.c_uint

        # hdUpdateCalibration(HDenum)
        hd.hdUpdateCalibration.argtypes = [ctypes.c_uint]
        hd.hdUpdateCalibration.restype = None

    def _check_error(self, context: str = ""):
        """检查并报告 HD API 错误"""
        error = self._hd.hdGetError()
        if error.errorCode != HD_SUCCESS:
            err_str = self._hd.hdGetErrorString(error.errorCode)
            err_msg = err_str.decode('utf-8') if err_str else "未知错误"
            raise RuntimeError(
                f"[HD Error] {context}: {err_msg} "
                f"(code=0x{error.errorCode:04X}, internal={error.internalErrorCode})"
            )

    def initialize(self, device_name: Optional[str] = None):
        """
        初始化力反馈设备

        参数:
            device_name: 设备名称，None 表示使用默认设备
        """
        print("[HapticDevice] 正在初始化设备...")

        # 初始化设备
        name = device_name.encode('utf-8') if device_name else None
        self._device_handle = self._hd.hdInitDevice(name)
        self._check_error("hdInitDevice")

        if self._device_handle == HD_INVALID_HANDLE:
            raise RuntimeError("设备初始化失败：获取到无效句柄")

        print(f"[HapticDevice] 设备句柄: {self._device_handle}")

        # 读取设备信息
        self._read_device_info()

        # 校准设备
        self._calibrate()

        # 使能力输出和安全限幅
        self._hd.hdEnable(HD_FORCE_OUTPUT)
        self._check_error("hdEnable(HD_FORCE_OUTPUT)")
        self._hd.hdEnable(HD_MAX_FORCE_CLAMPING)
        self._check_error("hdEnable(HD_MAX_FORCE_CLAMPING)")

        self._running = True
        print("[HapticDevice] 设备初始化完成！")

    def _read_device_info(self):
        """读取设备硬件信息"""
        info = self._device_info

        # 字符串信息
        for attr, param in [
            ('model', HD_DEVICE_MODEL_TYPE),
            ('vendor', HD_DEVICE_VENDOR),
            ('serial', HD_DEVICE_SERIAL_NUMBER),
            ('driver_version', HD_DEVICE_DRIVER_VERSION),
        ]:
            val = self._hd.hdGetString(param)
            if val:
                setattr(info, attr, val.decode('utf-8', errors='replace'))

        # 整数信息
        int_val = ctypes.c_int(0)
        for attr, param in [
            ('input_dof', HD_INPUT_DOF),
            ('output_dof', HD_OUTPUT_DOF),
        ]:
            self._hd.hdGetIntegerv(param, ctypes.byref(int_val))
            setattr(info, attr, int_val.value)

        # 浮点信息
        dbl_val = ctypes.c_double(0.0)
        for attr, param in [
            ('max_force', HD_NOMINAL_MAX_FORCE),
            ('max_continuous_force', HD_NOMINAL_MAX_CONTINUOUS_FORCE),
            ('max_stiffness', HD_NOMINAL_MAX_STIFFNESS),
            ('update_rate', HD_UPDATE_RATE),
        ]:
            self._hd.hdGetDoublev(param, ctypes.byref(dbl_val))
            setattr(info, attr, dbl_val.value)

        # 工作空间尺寸 (6 个 double: minX, minY, minZ, maxX, maxY, maxZ)
        workspace = (ctypes.c_double * 6)()
        self._hd.hdGetDoublev(HD_USABLE_WORKSPACE_DIMENSIONS, workspace)
        info.workspace_min = (workspace[0], workspace[1], workspace[2])
        info.workspace_max = (workspace[3], workspace[4], workspace[5])

        # 打印设备信息
        print(f"\n{'='*60}")
        print(f"  3D Systems Touch 力反馈笔 - 设备信息")
        print(f"{'='*60}")
        print(f"  型号:           {info.model}")
        print(f"  厂商:           {info.vendor}")
        print(f"  序列号:         {info.serial}")
        print(f"  驱动版本:       {info.driver_version}")
        print(f"  输入自由度:     {info.input_dof} DOF")
        print(f"  输出自由度:     {info.output_dof} DOF")
        print(f"  最大力:         {info.max_force:.2f} N")
        print(f"  最大持续力:     {info.max_continuous_force:.2f} N")
        print(f"  最大刚度:       {info.max_stiffness:.3f} N/mm")
        print(f"  伺服频率:       {info.update_rate:.0f} Hz")
        print(f"  工作空间(mm):   ({info.workspace_min[0]:.1f}, {info.workspace_min[1]:.1f}, {info.workspace_min[2]:.1f})")
        print(f"             到   ({info.workspace_max[0]:.1f}, {info.workspace_max[1]:.1f}, {info.workspace_max[2]:.1f})")
        print(f"{'='*60}\n")

    def _calibrate(self):
        """检查并执行设备校准"""
        cal_status = self._hd.hdCheckCalibration()
        if cal_status == HD_CALIBRATION_OK:
            print("[HapticDevice] 校准状态: OK")
        elif cal_status == HD_CALIBRATION_NEEDS_UPDATE:
            print("[HapticDevice] 正在自动校准...")
            self._hd.hdUpdateCalibration(HD_CALIBRATION_AUTO)
            self._check_error("hdUpdateCalibration")
            print("[HapticDevice] 校准完成")
        else:
            print(f"[HapticDevice] 校准状态: 0x{cal_status:04X} (可能需要手动校准)")

    def start_scheduler(self):
        """
        启动 1kHz 伺服循环调度器
        注册异步回调函数，在每个伺服周期内读取状态并输出力
        """
        if self._scheduler_running:
            print("[HapticDevice] 调度器已在运行")
            return

        # 创建伺服回调函数
        # 注意：必须保持对回调的引用，否则会被 Python GC 回收
        @HDCALLBACK
        def servo_callback(user_data):
            """
            伺服循环回调 - 以 1kHz 频率被调用
            在此函数内读取传感器数据并输出力
            """
            try:
                hd = self._hd
                handle = self._device_handle

                # 开始帧 - 从设备读取最新数据
                hd.hdBeginFrame(handle)

                # --- 读取位置 (3 个 double) ---
                pos = (ctypes.c_double * 3)()
                hd.hdGetDoublev(HD_CURRENT_POSITION, pos)

                # --- 读取速度 (3 个 double) ---
                vel = (ctypes.c_double * 3)()
                hd.hdGetDoublev(HD_CURRENT_VELOCITY, vel)

                # --- 读取关节角度 (3 个 double) ---
                joints = (ctypes.c_double * 3)()
                hd.hdGetDoublev(HD_CURRENT_JOINT_ANGLES, joints)

                # --- 读取万向节角度 (3 个 double) ---
                gimbal = (ctypes.c_double * 3)()
                hd.hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal)

                # --- 读取变换矩阵 (16 个 double) ---
                transform = (ctypes.c_double * 16)()
                hd.hdGetDoublev(HD_CURRENT_TRANSFORM, transform)

                # --- 读取按钮状态 ---
                buttons = ctypes.c_int(0)
                hd.hdGetIntegerv(HD_CURRENT_BUTTONS, ctypes.byref(buttons))

                # --- 读取安全开关（墨水瓶开关，笔是否在支架上）---
                inkwell = ctypes.c_int(0)
                hd.hdGetIntegerv(HD_CURRENT_INKWELL_SWITCH, ctypes.byref(inkwell))

                # --- 输出力反馈 ---
                with self._lock:
                    hd.hdSetDoublev(HD_CURRENT_FORCE, self._target_force)

                # 结束帧 - 将力命令发送到设备
                hd.hdEndFrame(handle)

                # 更新状态（线程安全）
                with self._lock:
                    self._state.position = (pos[0], pos[1], pos[2])
                    self._state.velocity = (vel[0], vel[1], vel[2])
                    self._state.joint_angles = (joints[0], joints[1], joints[2])
                    self._state.gimbal_angles = (gimbal[0], gimbal[1], gimbal[2])
                    self._state.transform = list(transform)
                    self._state.button1 = bool(buttons.value & HD_DEVICE_BUTTON_1)
                    self._state.button2 = bool(buttons.value & HD_DEVICE_BUTTON_2)
                    self._state.force = (
                        self._target_force[0],
                        self._target_force[1],
                        self._target_force[2]
                    )
                    self._state.inkwell = bool(inkwell.value)
                    self._state.timestamp = time.time()

            except Exception as e:
                print(f"[伺服回调异常] {e}")

            # 返回 CONTINUE 表示下一周期继续调用
            return HD_CALLBACK_CONTINUE

        # 保存回调引用
        self._servo_callback_ref = servo_callback

        # 注册异步回调（以最高优先级运行）
        self._hd.hdScheduleAsynchronous(
            servo_callback, None, HD_MAX_SCHEDULER_PRIORITY
        )

        # 启动调度器
        # 注意：hdStartScheduler 会在错误队列中留下 0x103 (HD_INVALID_INPUT_TYPE)
        # 这是 libPhantomIOLib42.so 在普通 Linux 上尝试初始化 RTAI 硬件定时器失败的痕迹。
        # 已通过 Patch 4 (0x3b166: jne→nop) 绕过该问题，伺服线程实际正常运行。
        # 此处清空错误队列而不抛出异常。
        self._hd.hdStartScheduler()
        try:
            error = self._hd.hdGetError()
            while error.errorCode != HD_SUCCESS:
                # 0x103 (HD_INVALID_INPUT_TYPE) 是已知的假错误，直接忽略
                if error.errorCode != 0x0103:
                    err_str = self._hd.hdGetErrorString(error.errorCode)
                    err_msg = err_str.decode('utf-8') if err_str else "未知错误"
                    print(f"[HapticDevice] hdStartScheduler 警告 (已忽略): "
                          f"{err_msg} (0x{error.errorCode:04X})")
                error = self._hd.hdGetError()
        except Exception:
            pass
        self._scheduler_running = True

        print("[HapticDevice] 伺服调度器已启动 (1kHz)")

    def get_state(self) -> HapticState:
        """
        获取力反馈笔的最新状态（线程安全）

        返回:
            HapticState 数据对象，包含位置、速度、角度、按钮等信息
        """
        with self._lock:
            # 返回状态的浅拷贝
            return HapticState(
                position=self._state.position,
                velocity=self._state.velocity,
                joint_angles=self._state.joint_angles,
                gimbal_angles=self._state.gimbal_angles,
                transform=list(self._state.transform),
                button1=self._state.button1,
                button2=self._state.button2,
                encoder_values=self._state.encoder_values,
                force=self._state.force,
                timestamp=self._state.timestamp,
                inkwell=self._state.inkwell,
            )

    def set_force(self, fx: float, fy: float, fz: float):
        """
        设置力反馈输出 (N)

        参数:
            fx: X 方向力 (N)，正值向右
            fy: Y 方向力 (N)，正值向上
            fz: Z 方向力 (N)，正值向用户方向（前方）

        注意：
            - Touch 设备最大力约 3.3N，持续最大约 0.88N
            - 超过限制会被 SDK 自动限幅（已启用 HD_MAX_FORCE_CLAMPING）
        """
        with self._lock:
            self._target_force[0] = fx
            self._target_force[1] = fy
            self._target_force[2] = fz

    def set_spring_force(self, kp: float = 0.1, origin: Tuple[float, float, float] = (0.0, 0.0, 0.0)):
        """
        设置弹簧力效果：笔被弹簧拉向原点

        参数:
            kp: 弹簧刚度 (N/mm)，建议 0.01 ~ 0.3
            origin: 弹簧原点位置 (mm)
        """
        state = self.get_state()
        fx = kp * (origin[0] - state.position[0])
        fy = kp * (origin[1] - state.position[1])
        fz = kp * (origin[2] - state.position[2])
        self.set_force(fx, fy, fz)

    def stop(self):
        """停止设备和调度器，释放资源"""
        print("[HapticDevice] 正在停止...")

        # 先清零力输出
        self.set_force(0.0, 0.0, 0.0)
        time.sleep(0.05)  # 等待力清零生效

        if self._scheduler_running:
            self._hd.hdStopScheduler()
            self._scheduler_running = False
            print("[HapticDevice] 调度器已停止")

        if self._device_handle != HD_INVALID_HANDLE:
            self._hd.hdDisableDevice(self._device_handle)
            self._device_handle = HD_INVALID_HANDLE
            print("[HapticDevice] 设备已禁用")

        self._running = False
        print("[HapticDevice] 清理完毕")

    @property
    def info(self) -> DeviceInfo:
        """获取设备信息"""
        return self._device_info

    @property
    def is_running(self) -> bool:
        """设备是否正在运行"""
        return self._running


# ============================================================================
# 交互式演示程序
# ============================================================================

def demo_read_position(device: HapticDevice, duration: float = 10.0):
    """
    演示1：实时读取并显示笔尖位置

    移动力反馈笔，观察 XYZ 坐标变化
    按下按钮时会有提示
    """
    print(f"\n{'='*60}")
    print("  演示1: 实时位置读取 (持续 {:.0f} 秒)".format(duration))
    print("  移动力反馈笔，观察位置变化")
    print("  按下按钮查看按钮状态")
    print(f"{'='*60}\n")

    start = time.time()
    last_print = 0
    while time.time() - start < duration:
        state = device.get_state()
        now = time.time()

        # 每 100ms 打印一次（避免刷屏）
        if now - last_print > 0.1:
            pos = state.position
            vel = state.velocity
            joints = state.joint_angles
            gimbal = state.gimbal_angles
            btn_str = ""
            if state.button1:
                btn_str += " [按钮1-灰]"
            if state.button2:
                btn_str += " [按钮2-白]"
            if state.inkwell:
                btn_str += " [墨水瓶]"

            # 清行并打印
            sys.stdout.write(
                f"\r  位置(mm): X={pos[0]:+8.2f}  Y={pos[1]:+8.2f}  Z={pos[2]:+8.2f}"
                f"  | 关节(rad): {joints[0]:+5.2f} {joints[1]:+5.2f} {joints[2]:+5.2f}"
                f"  {btn_str}    "
            )
            sys.stdout.flush()
            last_print = now

        time.sleep(0.01)  # 10ms 间隔，100Hz 显示刷新率

    print("\n")


def demo_force_feedback(device: HapticDevice, duration: float = 15.0):
    """
    演示2：力反馈效果测试

    依次测试多种力反馈效果：
    1. 恒力（重力感）：向下施加恒定力
    2. 弹簧力：笔被拉向中心点
    3. 粘性阻尼：运动时感到阻力
    4. 脉冲振动：短促的力脉冲
    5. 正弦波振动：周期性振动
    """
    print(f"\n{'='*60}")
    print("  演示2: 力反馈效果测试 (持续 {:.0f} 秒)".format(duration))
    print(f"{'='*60}\n")

    effects = [
        ("恒力（模拟重力）", 3.0),
        ("弹簧回中力", 3.0),
        ("粘性阻尼", 3.0),
        ("脉冲振动", 3.0),
        ("正弦波振动", 3.0),
    ]

    start = time.time()
    effect_idx = 0
    effect_start = start

    while time.time() - start < duration and effect_idx < len(effects):
        elapsed_in_effect = time.time() - effect_start
        effect_name, effect_duration = effects[effect_idx]

        # 切换到下一个效果
        if elapsed_in_effect >= effect_duration:
            device.set_force(0.0, 0.0, 0.0)
            time.sleep(0.2)  # 效果切换间隔
            effect_idx += 1
            effect_start = time.time()
            if effect_idx < len(effects):
                print(f"\n  >> 切换到效果 {effect_idx + 1}: {effects[effect_idx][0]}")
            continue

        state = device.get_state()

        if effect_idx == 0:
            # ---- 效果1: 恒力（模拟重力）----
            # 向 Y 轴负方向（向下）施加 0.5N 恒力
            device.set_force(0.0, -0.5, 0.0)

        elif effect_idx == 1:
            # ---- 效果2: 弹簧回中力 ----
            # 将笔拉向原点，刚度 0.08 N/mm
            kp = 0.08
            pos = state.position
            fx = -kp * pos[0]
            fy = -kp * pos[1]
            fz = -kp * pos[2]
            device.set_force(fx, fy, fz)

        elif effect_idx == 2:
            # ---- 效果3: 粘性阻尼 ----
            # 与速度方向相反的力，模拟在粘稠液体中移动
            kd = 0.003  # 阻尼系数 (N·s/mm)
            vel = state.velocity
            fx = -kd * vel[0]
            fy = -kd * vel[1]
            fz = -kd * vel[2]
            device.set_force(fx, fy, fz)

        elif effect_idx == 3:
            # ---- 效果4: 脉冲振动 ----
            # 每 200ms 产生一个短促的力脉冲
            t = elapsed_in_effect
            pulse_period = 0.2   # 脉冲周期 (s)
            pulse_width = 0.05   # 脉冲宽度 (s)
            phase = t % pulse_period
            if phase < pulse_width:
                # 脉冲期间施加力
                device.set_force(0.0, 0.8, 0.0)
            else:
                device.set_force(0.0, 0.0, 0.0)

        elif effect_idx == 4:
            # ---- 效果5: 正弦波振动 ----
            # 在 Y 轴上施加正弦波力，频率 5Hz，振幅 0.6N
            import math
            t = elapsed_in_effect
            freq = 5.0   # 频率 (Hz)
            amp = 0.6    # 振幅 (N)
            force_y = amp * math.sin(2 * math.pi * freq * t)
            device.set_force(0.0, force_y, 0.0)

        # 显示状态
        sys.stdout.write(
            f"\r  [{effect_name}] "
            f"位置: ({state.position[0]:+6.1f}, {state.position[1]:+6.1f}, {state.position[2]:+6.1f}) mm  "
            f"力: ({state.force[0]:+5.2f}, {state.force[1]:+5.2f}, {state.force[2]:+5.2f}) N  "
            f"剩余: {effect_duration - elapsed_in_effect:.1f}s    "
        )
        sys.stdout.flush()

        time.sleep(0.001)  # 1kHz 力更新（与伺服循环匹配）

    # 结束时清零力
    device.set_force(0.0, 0.0, 0.0)
    print("\n\n  力反馈效果演示完成！")


def demo_wall_effect(device: HapticDevice, duration: float = 10.0):
    """
    演示3：虚拟墙壁效果

    在 Z=0 平面设置一面虚拟墙壁
    当笔尖穿过墙壁时，会感到阻力（弹簧力推回）
    """
    print(f"\n{'='*60}")
    print("  演示3: 虚拟墙壁效果 (持续 {:.0f} 秒)".format(duration))
    print("  在 Z=0 处有一面虚拟墙壁")
    print("  将笔向前推（Z 负方向），感受墙壁阻力")
    print(f"{'='*60}\n")

    wall_z = 0.0       # 墙壁位置 (mm)
    wall_stiffness = 0.5  # 墙壁刚度 (N/mm)
    wall_damping = 0.002  # 墙壁阻尼 (N·s/mm)

    start = time.time()
    while time.time() - start < duration:
        state = device.get_state()
        pos = state.position
        vel = state.velocity

        fz = 0.0
        touching = False

        # 如果笔尖穿过了墙壁（Z < wall_z）
        if pos[2] < wall_z:
            # 弹簧力 + 阻尼力 将笔推回墙壁外侧
            penetration = wall_z - pos[2]  # 穿透深度 (正值)
            fz = wall_stiffness * penetration - wall_damping * vel[2]
            touching = True

        device.set_force(0.0, 0.0, fz)

        wall_str = ">> 接触墙壁! <<" if touching else "   自由空间   "
        sys.stdout.write(
            f"\r  Z={pos[2]:+8.2f}mm  力Z={fz:+6.3f}N  {wall_str}    "
        )
        sys.stdout.flush()

        time.sleep(0.001)

    device.set_force(0.0, 0.0, 0.0)
    print("\n")


# ============================================================================
# 主程序入口
# ============================================================================

def main():
    """主程序：初始化设备并运行所有演示"""
    print("\n" + "=" * 60)
    print("  3D Systems Touch 力反馈笔 - 驱动与演示程序")
    print("  " + "=" * 56)
    print("  日期: 2026-03-28")
    print("  依赖: OpenHaptics SDK 3.4 + Touch Driver 2022")
    print("=" * 60 + "\n")

    device = HapticDevice()

    # 注册信号处理，确保 Ctrl+C 能正常退出
    def signal_handler(sig, frame):
        print("\n\n[中断] 收到 Ctrl+C，正在安全退出...")
        device.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        # 1. 初始化设备
        device.initialize()

        # 2. 启动伺服循环
        device.start_scheduler()

        # 等待一下让调度器稳定
        time.sleep(0.5)

        # 3. 演示1: 读取位置
        demo_read_position(device, duration=10.0)

        # 4. 演示2: 力反馈效果
        demo_force_feedback(device, duration=15.0)

        # 5. 演示3: 虚拟墙壁
        demo_wall_effect(device, duration=10.0)

        print("\n" + "=" * 60)
        print("  所有演示完成！")
        print("=" * 60 + "\n")

    except Exception as e:
        print(f"\n[错误] {e}")
        import traceback
        traceback.print_exc()

    finally:
        device.stop()


if __name__ == "__main__":
    main()
