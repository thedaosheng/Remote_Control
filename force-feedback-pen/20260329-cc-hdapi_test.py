#!/usr/bin/env python3
"""
3D Systems Touch - HD API 直接调用测试
========================================
通过 ctypes 调用 libHD.so，让 SDK 自己完成完整初始化。
SDK 可能在内部做了 strace 看不到的事情（如 USB 控制传输）。

前提条件：
- /dev/ttyACM300 符号链接已存在（指向 /dev/ttyACM0）
- ~/.3dsystems/config/Default Device.config 存在且 ModelType=OMNI
"""

import ctypes
import ctypes.util
import os
import sys
import time
import signal

# ============================================================================
# HD API 常量（从 hdDefines.h）
# ============================================================================

HD_SUCCESS = 0x0000
HD_DEFAULT_DEVICE = None  # NULL = 默认设备

# hdGet 参数
HD_CURRENT_POSITION = 0x2050
HD_CURRENT_VELOCITY = 0x2051
HD_CURRENT_TRANSFORM = 0x2052
HD_CURRENT_ANGULAR_VELOCITY = 0x2053
HD_CURRENT_JACOBIAN = 0x2054
HD_CURRENT_JOINT_ANGLES = 0x2100
HD_CURRENT_GIMBAL_ANGLES = 0x2150
HD_CURRENT_FORCE = 0x2700
HD_CURRENT_BUTTONS = 0x2000
HD_CURRENT_ENCODER_VALUES = 0x2010

# hdSet 参数
HD_CURRENT_FORCE_SET = 0x2700

# hdEnable/hdDisable 参数
HD_FORCE_OUTPUT = 0x4000
HD_MAX_FORCE_CLAMPING = 0x4001
HD_FORCE_RAMPING = 0x4002

# 回调返回值
HD_CALLBACK_DONE = 0
HD_CALLBACK_CONTINUE = 1

# 调度器优先级
HD_MAX_SCHEDULER_PRIORITY = 0xffff
HD_DEFAULT_SCHEDULER_PRIORITY = 0

# 错误码
HD_DEVICE_FAULT = 0x0300
HD_DEVICE_ERROR = 0x0301
HD_INVALID_HANDLE = 0x0302
HD_COMM_ERROR = 0x0400
HD_COMM_CONFIG_ERROR = 0x0401
HD_TIMER_ERROR = 0x0402
HD_INVALID_LICENSE = 0x0600

# 类型定义
HDCallbackCode = ctypes.c_uint
HHD = ctypes.c_ulong
HDSchedulerHandle = ctypes.c_ulong
HDErrorInfo_errorCode = ctypes.c_ushort
HDdouble = ctypes.c_double
HDint = ctypes.c_int
HDlong = ctypes.c_long

# 回调函数类型
HDCALLBACK = ctypes.CFUNCTYPE(HDCallbackCode, ctypes.c_void_p)

# HDErrorInfo 结构体
class HDErrorInfo(ctypes.Structure):
    _fields_ = [
        ("errorCode", ctypes.c_ushort),
        ("internalErrorCode", ctypes.c_int),
        ("hHD", HHD),
    ]


def error_code_name(code):
    """错误码转可读名称"""
    names = {
        0x0000: "HD_SUCCESS",
        0x0300: "HD_DEVICE_FAULT",
        0x0301: "HD_DEVICE_ERROR",
        0x0302: "HD_INVALID_HANDLE",
        0x0400: "HD_COMM_ERROR",
        0x0401: "HD_COMM_CONFIG_ERROR",
        0x0402: "HD_TIMER_ERROR",
        0x0600: "HD_INVALID_LICENSE",
    }
    return names.get(code, f"UNKNOWN(0x{code:04x})")


def main():
    print(f"\n{'='*60}")
    print(f"  3D Systems Touch - HD API 直接调用测试")
    print(f"{'='*60}\n")

    # ================================================================
    # 环境准备
    # ================================================================

    # 确认 ttyACM300 符号链接
    if not os.path.exists('/dev/ttyACM300'):
        print("[错误] /dev/ttyACM300 不存在！请运行：")
        print("  sudo ln -sf /dev/ttyACM0 /dev/ttyACM300")
        return

    # 确认配置文件
    config_path = os.path.expanduser("~/.3dsystems/config/Default Device.config")
    if not os.path.exists(config_path):
        print(f"[警告] 配置文件不存在: {config_path}")
        print("  Touch_Diagnostic 会自动创建")

    # 设置库路径
    lib_dirs = [
        "/usr/lib",
        "/home/rhz/haptics_install/local/lib",
    ]
    ld_path = os.environ.get("LD_LIBRARY_PATH", "")
    for d in lib_dirs:
        if d not in ld_path:
            ld_path = d + ":" + ld_path
    os.environ["LD_LIBRARY_PATH"] = ld_path

    print(f"[环境]")
    print(f"  ttyACM300: {os.path.realpath('/dev/ttyACM300')}")
    print(f"  LD_LIBRARY_PATH: {ld_path[:80]}...")
    print(f"  config: {config_path}")
    if os.path.exists(config_path):
        with open(config_path) as f:
            first_line = f.readline().strip()
            print(f"  ModelType: {first_line}")

    # ================================================================
    # 加载 libHD.so
    # ================================================================
    print(f"\n[1] 加载 libHD.so...")
    try:
        hd = ctypes.CDLL("/usr/lib/libHD.so", mode=ctypes.RTLD_GLOBAL)
        print(f"    ✓ libHD.so 已加载")
    except OSError as e:
        print(f"    ✗ 加载失败: {e}")
        # 尝试加载依赖库
        try:
            ctypes.CDLL("/usr/lib/libPhantomIOLib42.so", mode=ctypes.RTLD_GLOBAL)
            hd = ctypes.CDLL("/usr/lib/libHD.so", mode=ctypes.RTLD_GLOBAL)
            print(f"    ✓ libHD.so 已加载（通过手动加载依赖）")
        except OSError as e2:
            print(f"    ✗ 仍然失败: {e2}")
            return

    # 设置函数签名
    hd.hdInitDevice.restype = HHD
    hd.hdInitDevice.argtypes = [ctypes.c_char_p]

    hd.hdGetError.restype = HDErrorInfo
    hd.hdGetError.argtypes = []

    hd.hdGetErrorString.restype = ctypes.c_char_p
    hd.hdGetErrorString.argtypes = [ctypes.c_ushort]

    hd.hdBeginFrame.restype = None
    hd.hdBeginFrame.argtypes = [HHD]

    hd.hdEndFrame.restype = None
    hd.hdEndFrame.argtypes = [HHD]

    hd.hdGetDoublev.restype = None
    hd.hdGetDoublev.argtypes = [ctypes.c_uint, ctypes.POINTER(HDdouble)]

    hd.hdGetIntegerv.restype = None
    hd.hdGetIntegerv.argtypes = [ctypes.c_uint, ctypes.POINTER(HDint)]

    hd.hdGetLongv.restype = None
    hd.hdGetLongv.argtypes = [ctypes.c_uint, ctypes.POINTER(HDlong)]

    hd.hdEnable.restype = None
    hd.hdEnable.argtypes = [ctypes.c_uint]

    hd.hdDisable.restype = None
    hd.hdDisable.argtypes = [ctypes.c_uint]

    hd.hdStartScheduler.restype = None
    hd.hdStartScheduler.argtypes = []

    hd.hdStopScheduler.restype = None
    hd.hdStopScheduler.argtypes = []

    hd.hdScheduleAsynchronous.restype = HDSchedulerHandle
    hd.hdScheduleAsynchronous.argtypes = [HDCALLBACK, ctypes.c_void_p, ctypes.c_ushort]

    hd.hdScheduleSynchronous.restype = None
    hd.hdScheduleSynchronous.argtypes = [HDCALLBACK, ctypes.c_void_p, ctypes.c_ushort]

    hd.hdDisableDevice.restype = None
    hd.hdDisableDevice.argtypes = [HHD]

    hd.hdGetString.restype = ctypes.c_char_p
    hd.hdGetString.argtypes = [ctypes.c_uint]

    # ================================================================
    # 初始化设备
    # ================================================================
    print(f"\n[2] 初始化设备 (hdInitDevice)...")
    print(f"    这是 SDK 的完整初始化，包含 strace 看不到的操作...")

    handle = hd.hdInitDevice(HD_DEFAULT_DEVICE)

    # 检查错误
    error = hd.hdGetError()
    if error.errorCode != HD_SUCCESS:
        err_str = hd.hdGetErrorString(error.errorCode)
        err_name = error_code_name(error.errorCode)
        print(f"    ✗ 初始化失败!")
        print(f"    错误码: {err_name} (0x{error.errorCode:04x})")
        print(f"    内部错误: {error.internalErrorCode}")
        print(f"    错误描述: {err_str.decode() if err_str else 'N/A'}")
        print(f"    设备句柄: {handle}")

        # 即使失败也尝试获取一些信息
        print(f"\n    尝试获取设备信息（即使初始化失败）...")
        try:
            # 0x2200 = HD_DEVICE_MODEL_TYPE
            model = hd.hdGetString(0x2200)
            if model:
                print(f"    设备型号: {model.decode()}")
        except:
            pass

        return

    print(f"    ✓ 设备初始化成功! 句柄: {handle}")

    try:
        # ================================================================
        # 获取设备信息
        # ================================================================
        print(f"\n[3] 读取设备信息...")

        # HD_DEVICE_MODEL_TYPE = 0x2200
        model = hd.hdGetString(0x2200)
        print(f"    型号: {model.decode() if model else 'N/A'}")

        # HD_DEVICE_DRIVER_VERSION = 0x2201
        driver = hd.hdGetString(0x2201)
        print(f"    驱动版本: {driver.decode() if driver else 'N/A'}")

        # HD_DEVICE_VENDOR = 0x2202
        vendor = hd.hdGetString(0x2202)
        print(f"    厂商: {vendor.decode() if vendor else 'N/A'}")

        # HD_DEVICE_SERIAL_NUMBER = 0x2203
        sn = hd.hdGetString(0x2203)
        print(f"    序列号: {sn.decode() if sn else 'N/A'}")

        # ================================================================
        # 使能力输出
        # ================================================================
        print(f"\n[4] 使能力输出 (hdEnable(HD_FORCE_OUTPUT))...")
        hd.hdEnable(HD_FORCE_OUTPUT)
        error = hd.hdGetError()
        if error.errorCode != HD_SUCCESS:
            err_name = error_code_name(error.errorCode)
            print(f"    ✗ 失败: {err_name}")
        else:
            print(f"    ✓ 力输出已使能")

        # ================================================================
        # 启动调度器并读取位置
        # ================================================================
        print(f"\n[5] 启动调度器 (hdStartScheduler)...")
        hd.hdStartScheduler()
        error = hd.hdGetError()
        if error.errorCode != HD_SUCCESS:
            err_name = error_code_name(error.errorCode)
            print(f"    ✗ 失败: {err_name}")
        else:
            print(f"    ✓ 调度器已启动")

        # 读取位置数据
        print(f"\n[6] 读取位置数据 (5 秒)...")
        print(f"    请移动力反馈笔！\n")

        for i in range(50):  # 50 次 x 0.1秒 = 5 秒
            try:
                hd.hdBeginFrame(handle)

                # 读取位置
                pos = (HDdouble * 3)()
                hd.hdGetDoublev(HD_CURRENT_POSITION, pos)

                # 读取按钮
                buttons = HDint(0)
                hd.hdGetIntegerv(HD_CURRENT_BUTTONS, ctypes.byref(buttons))

                # 读取关节角
                joints = (HDdouble * 3)()
                hd.hdGetDoublev(HD_CURRENT_JOINT_ANGLES, joints)

                hd.hdEndFrame(handle)

                # 检查错误
                error = hd.hdGetError()
                if error.errorCode != HD_SUCCESS:
                    err_name = error_code_name(error.errorCode)
                    if i == 0:
                        print(f"    帧错误: {err_name}")
                    continue

                if i % 5 == 0:  # 每 0.5 秒显示一次
                    print(f"    [{i*100:4d}ms] "
                          f"pos=({pos[0]:8.2f}, {pos[1]:8.2f}, {pos[2]:8.2f}) "
                          f"btn={buttons.value:02x} "
                          f"joints=({joints[0]:6.3f}, {joints[1]:6.3f}, {joints[2]:6.3f})")

            except Exception as e:
                print(f"    [{i*100:4d}ms] 异常: {e}")

            time.sleep(0.1)

        # ================================================================
        # 测试力反馈
        # ================================================================
        print(f"\n[7] 测试力反馈 (轻微向上推力 0.3N，2 秒)...")
        print(f"    你应该能感受到笔尖被轻轻推开")

        force = (HDdouble * 3)(0.0, 0.3, 0.0)  # Y 轴向上 0.3N

        for i in range(20):
            try:
                hd.hdBeginFrame(handle)
                hd.hdSetDoublev(HD_CURRENT_FORCE_SET, force)

                pos = (HDdouble * 3)()
                hd.hdGetDoublev(HD_CURRENT_POSITION, pos)

                hd.hdEndFrame(handle)

                if i % 5 == 0:
                    print(f"    [{i*100:4d}ms] 施力中... pos=({pos[0]:8.2f}, {pos[1]:8.2f}, {pos[2]:8.2f})")
            except Exception as e:
                print(f"    异常: {e}")
                break

            time.sleep(0.1)

        # 停止力输出
        force_zero = (HDdouble * 3)(0.0, 0.0, 0.0)
        hd.hdBeginFrame(handle)
        hd.hdSetDoublev(HD_CURRENT_FORCE_SET, force_zero)
        hd.hdEndFrame(handle)
        print(f"    力输出已停止")

    except Exception as e:
        print(f"\n[错误] {e}")
        import traceback
        traceback.print_exc()

    finally:
        # 清理
        print(f"\n[8] 清理...")
        try:
            hd.hdStopScheduler()
            print(f"    调度器已停止")
        except:
            pass
        try:
            hd.hdDisableDevice(handle)
            print(f"    设备已禁用")
        except:
            pass

    print(f"\n{'='*60}")
    print(f"  测试完成")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
