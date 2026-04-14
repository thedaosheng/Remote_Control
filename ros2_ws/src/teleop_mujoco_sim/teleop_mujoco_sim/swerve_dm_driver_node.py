#!/usr/bin/env python3
"""
==============================================================
  舵轮底盘电机驱动节点 (DM4310 转向 + ZLAC8015D 驱动)
==============================================================

ROS2 节点：订阅 /mujoco/swerve_cmd → 驱动真实电机

数据流:
  keyboard_teleop_node → /cmd_vel (Twist)
  mujoco_sim_node      → /mujoco/swerve_cmd (Float64MultiArray[8])
  本节点               → 达妙 USB-CAN → DM4310 × 4 (转向)
                                       → ZLAC8015D × 1 (驱动)

swerve_cmd 格式 (8 个 double):
  [0..3] steer: FL, FR, RL, RR 转向角 (rad)   → DM4310 MIT 位置控制
  [4..7] drive: FL, FR, RL, RR 驱动速度 (rad/s) → ZLAC8015D CANopen 速度控制

硬件架构:
  所有电机共用一条 CAN 总线，通过达妙 HDSC CDC USB-CAN 适配器。
  - DM4310: 私有协议，CAN ID = SlaveID (0x03/0x04/0x05/0x06)
  - ZLAC8015D: CANopen CiA402，CAN ID = 0x600+NodeID (SDO)
  两种协议的 CAN ID 不冲突，可以共存。

DM4310 电机布局:
  FL = SlaveID 0x03  左前
  FR = SlaveID 0x06  右前
  RL = SlaveID 0x04  左后
  RR = SlaveID 0x05  右后

ZLAC8015D:
  双通道轮毂伺服，当前只接 1 个轮
  默认 CAN Node ID = 1 (注意: RS485 默认是 4，CAN 默认是 1!)
  速度模式 (CiA402 Profile Velocity, mode=3)

运行:
  ros2 run teleop_mujoco_sim swerve_dm_driver_node \\
    --ros-args --params-file config/swerve_dm_motor.yaml
"""

import sys
import os
import time
import math
import threading
import glob as glob_module
import struct

# 达妙 CAN 库
sys.path.insert(0, '/home/rhz/teleop/DM_Control_Python')
from DM_CAN import (Motor, MotorControl, DM_Motor_Type, DM_variable,
                     Control_Type)

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


# ============================================================
#  常量定义
# ============================================================

# rad/s → RPM 转换系数
RAD_S_TO_RPM = 60.0 / (2.0 * math.pi)  # ≈ 9.5493

# ---- CANopen CiA402 对象字典 ----
OBJ_CONTROLWORD      = 0x6040   # 控制字 (UINT16, RW)
OBJ_STATUSWORD       = 0x6041   # 状态字 (UINT16, RO)
OBJ_MODES_OF_OP      = 0x6060   # 运行模式 (INT8, RW)
OBJ_MODES_OF_OP_DISP = 0x6061   # 运行模式显示 (INT8, RO)
OBJ_VEL_ACTUAL       = 0x606C   # 实际速度 (INT32, RO)
OBJ_TARGET_VEL       = 0x60FF   # 目标速度 (INT32, RW)
OBJ_PROFILE_ACCEL    = 0x6083   # 轮廓加速度 (UINT32, RW)
OBJ_PROFILE_DECEL    = 0x6084   # 轮廓减速度 (UINT32, RW)

# CiA402 控制字命令
CW_SHUTDOWN     = 0x0006   # → Ready to Switch On
CW_SWITCH_ON    = 0x0007   # → Switched On
CW_ENABLE_OP    = 0x000F   # → Operation Enabled
CW_DISABLE_VOLT = 0x0000   # → 断电
CW_FAULT_RESET  = 0x0080   # → 故障复位

# 运行模式
MODE_PROFILE_VEL = 3  # 轮廓速度模式

# NMT 命令 (COB-ID = 0x000)
NMT_START      = 0x01  # 启动远程节点
NMT_RESET_COMM = 0x82  # 复位通讯

# SDO 命令字节 (expedited transfer)
SDO_WRITE_1B = 0x2F
SDO_WRITE_2B = 0x2B
SDO_WRITE_4B = 0x23
SDO_READ_REQ = 0x40

# ---- 达妙 USB2CAN 串口帧模板 ----
# 30 字节固定格式:
#   [0:1]   = 0x55 0xAA (帧头)
#   [13:14] = CAN ID (little-endian)
#   [21:28] = 8 字节 CAN 数据
DAMIAO_TX_TEMPLATE = np.array(
    [0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00,
     0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0x00, 0x08, 0x00,
     0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00], np.uint8)


# ============================================================
#  工具函数
# ============================================================

def find_hdsc_cdc_port():
    """
    自动扫描系统中的 HDSC CDC 设备（达妙 USB-CAN 适配器）。

    遍历 /dev/ttyACM*，通过 udevadm 读取 ID_VENDOR，
    匹配 "HDSC" 即为达妙适配器。

    返回:
      str: 设备路径 (如 '/dev/ttyACM2')，未找到返回 None
    """
    import subprocess
    candidates = sorted(glob_module.glob('/dev/ttyACM*'))
    for dev in candidates:
        try:
            result = subprocess.run(
                ['udevadm', 'info', '-q', 'property', dev],
                capture_output=True, text=True, timeout=2)
            for line in result.stdout.split('\n'):
                if line.startswith('ID_VENDOR=') and 'HDSC' in line:
                    return dev
        except Exception:
            continue
    return None


def clear_motor_errors(mc, motor):
    """
    清除 DM4310 电机错误状态。

    达妙电机错误清除命令: CAN 帧 data = FF FF FF FF FF FF FF FB
    流程: 发送 0xFB 清除 → 失能 → 切 MIT → 使能 → 检查 ERR
    最多重试 3 次。

    参数:
      mc: MotorControl 对象
      motor: Motor 对象

    返回:
      int: ERR 码 (1=正常, 其他=仍有错误)
    """
    for attempt in range(3):
        # 发送 0xFB 清除错误命令
        clear_data = np.array([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB],
                              np.uint8)
        frame = mc.send_data_frame.copy()
        frame[13] = motor.SlaveID & 0xFF
        frame[14] = (motor.SlaveID >> 8) & 0xFF
        frame[21:29] = clear_data
        mc.serial_.write(bytes(frame))
        time.sleep(0.3)
        mc.serial_.read_all()

        # 失能 → 切 MIT → 使能
        mc.disable(motor)
        time.sleep(0.1)
        mc.switchControlMode(motor, Control_Type.MIT)
        time.sleep(0.1)
        mc.enable(motor)
        time.sleep(0.1)

        # 发零力矩触发反馈帧
        mc.serial_.reset_input_buffer()
        mc.controlMIT(motor, 0.0, 0.5, 0.0, 0.0, 0.0)
        time.sleep(0.05)
        raw = mc.serial_.read_all()

        # 解析 ERR 字段
        err_code = -1
        i = 0
        while i <= len(raw) - 16:
            if raw[i] == 0xAA and raw[i + 15] == 0x55:
                err_code = (raw[i + 7] >> 4) & 0x0F
                break
            i += 1

        if err_code == 1:
            return err_code

        mc.disable(motor)
        time.sleep(0.3)

    return err_code


# ============================================================
#  ZLAC8015D CANopen 驱动 (通过达妙 USB2CAN 串口)
# ============================================================

class ZlacCANopen:
    """
    ZLAC8015D CANopen 底层驱动。

    不依赖 SocketCAN / python-can，直接通过达妙 USB2CAN 的串口协议
    发送/接收 CANopen CAN 帧。与 DM4310 共享同一个串口。

    发送帧格式 (达妙): 30 字节
      [13:14] = CAN ID (little-endian)
      [21:28] = 8 字节 CAN 数据

    接收帧格式 (达妙): 16 字节
      [0] = 0xAA (帧头)
      [3:6] = CAN ID (little-endian, 4 字节)
      [7:14] = 8 字节 CAN 数据
      [15] = 0x55 (帧尾)
    """

    def __init__(self, serial_dev, node_id, logger=None):
        """
        参数:
          serial_dev: pyserial Serial 对象 (与 MotorControl 共享)
          node_id: ZLAC 的 CANopen Node ID (默认 1)
          logger: ROS2 logger
        """
        self.ser = serial_dev
        self.node_id = node_id
        self.log = logger
        # SDO COB-ID
        self.sdo_tx_cobid = 0x600 + node_id  # 我们发给 ZLAC
        self.sdo_rx_cobid = 0x580 + node_id  # ZLAC 回给我们

    def _log_info(self, msg):
        if self.log:
            self.log.info(msg)

    def _log_warn(self, msg):
        if self.log:
            self.log.warn(msg)

    # ---- 底层 CAN 帧发送/接收 ----

    def _send_can(self, can_id, data_8):
        """
        通过达妙 USB2CAN 发送一帧 CAN 数据。

        参数:
          can_id: 标准 CAN ID (11 bit, 0x000-0x7FF)
          data_8: 8 字节数据 (list/array)
        """
        frame = DAMIAO_TX_TEMPLATE.copy()
        frame[13] = can_id & 0xFF
        frame[14] = (can_id >> 8) & 0xFF
        frame[21:29] = data_8
        self.ser.write(bytes(frame))

    def _recv_can_filtered(self, target_cobid, timeout=0.5):
        """
        接收并过滤指定 CAN ID 的响应帧。

        遍历串口缓冲区中的达妙 16 字节帧，
        匹配 target_cobid，返回 8 字节数据。
        不匹配的帧 (如 DM4310 反馈帧) 会被跳过。

        参数:
          target_cobid: 期望的 CAN ID (如 0x581)
          timeout: 超时时间 (秒)

        返回:
          bytes (8字节) 或 None
        """
        deadline = time.time() + timeout
        buf = b''
        while time.time() < deadline:
            chunk = self.ser.read_all()
            if chunk:
                buf += chunk
            # 逐帧扫描
            i = 0
            while i <= len(buf) - 16:
                if buf[i] == 0xAA and buf[i + 15] == 0x55:
                    # 解析 CAN ID (4 字节 little-endian)
                    cid = (buf[i+6] << 24) | (buf[i+5] << 16) | \
                          (buf[i+4] << 8) | buf[i+3]
                    if cid == target_cobid:
                        return buf[i+7:i+15]
                    i += 16
                else:
                    i += 1
            # 保留未解析尾部
            buf = buf[max(0, len(buf) - 15):]
            if not chunk:
                time.sleep(0.005)
        return None

    # ---- NMT ----

    def nmt_start(self):
        """NMT: 启动远程节点 → Operational"""
        self._send_can(0x000, [NMT_START, self.node_id, 0, 0, 0, 0, 0, 0])
        self._log_info(f'  ZLAC Node {self.node_id}: NMT Start')

    def nmt_reset_comm(self):
        """NMT: 复位通讯"""
        self._send_can(0x000, [NMT_RESET_COMM, self.node_id, 0, 0, 0, 0, 0, 0])

    # ---- SDO 读写 ----

    def sdo_write(self, index, subindex, value, size):
        """
        CANopen SDO 写入 (expedited transfer)。

        帧格式: COB-ID = 0x600 + NodeID
          Byte 0: 命令 (0x2F=1B, 0x2B=2B, 0x23=4B)
          Byte 1-2: 索引 (little-endian)
          Byte 3: 子索引
          Byte 4-7: 数据 (little-endian, 有符号用补码)

        参数:
          index: 对象字典索引 (如 0x6040)
          subindex: 子索引 (通常 0)
          value: 写入值 (int, 可负数)
          size: 字节数 (1/2/4)

        返回:
          bool: 是否收到 0x60 确认
        """
        cmd_map = {1: SDO_WRITE_1B, 2: SDO_WRITE_2B, 4: SDO_WRITE_4B}
        cmd = cmd_map.get(size, SDO_WRITE_4B)

        # 处理负数补码
        if value < 0:
            val_uint = value & 0xFFFFFFFF
        else:
            val_uint = value

        data = [
            cmd,
            index & 0xFF, (index >> 8) & 0xFF,   # 索引 little-endian
            subindex,
            val_uint & 0xFF,                       # 数据 little-endian
            (val_uint >> 8) & 0xFF,
            (val_uint >> 16) & 0xFF,
            (val_uint >> 24) & 0xFF,
        ]

        # 清空接收缓冲，避免旧数据干扰
        self.ser.read_all()
        self._send_can(self.sdo_tx_cobid, data)

        # 等待 SDO 应答
        resp = self._recv_can_filtered(self.sdo_rx_cobid, timeout=0.3)
        if resp is None:
            self._log_warn(f'  ZLAC SDO 写超时 0x{index:04X}:{subindex}')
            return False

        if resp[0] == 0x60:
            return True  # 写入确认
        elif resp[0] == 0x80:
            # SDO 中止
            abort = struct.unpack_from('<I', resp, 4)[0]
            self._log_warn(f'  ZLAC SDO 写失败 0x{index:04X} abort=0x{abort:08X}')
            return False

        return False

    def sdo_read(self, index, subindex):
        """
        CANopen SDO 读取 (expedited transfer)。

        发送 0x40 读请求，等待 0x4x 响应。

        返回:
          int (unsigned 32-bit) 或 None
        """
        data = [
            SDO_READ_REQ,
            index & 0xFF, (index >> 8) & 0xFF,
            subindex,
            0, 0, 0, 0
        ]

        self.ser.read_all()
        self._send_can(self.sdo_tx_cobid, data)

        resp = self._recv_can_filtered(self.sdo_rx_cobid, timeout=0.3)
        if resp is None:
            self._log_warn(f'  ZLAC SDO 读超时 0x{index:04X}:{subindex}')
            return None

        cmd = resp[0]
        if cmd in (0x4F, 0x4B, 0x43, 0x42):
            return struct.unpack_from('<I', resp, 4)[0]
        elif cmd == 0x80:
            abort = struct.unpack_from('<I', resp, 4)[0]
            self._log_warn(f'  ZLAC SDO 读失败 0x{index:04X} abort=0x{abort:08X}')
        return None

    # ---- CiA402 状态机 ----

    def read_statusword(self):
        """读取状态字 (0x6041)"""
        val = self.sdo_read(OBJ_STATUSWORD, 0)
        return (val & 0xFFFF) if val is not None else None

    def enable_motor(self):
        """
        ZLAC8015D 完整使能流程 (参考 byeongkyu/zltech_canopen_test):

          1. 故障复位 (上升沿 bit7: 0x00 → 0x80 → 0x00)
          2. 设异步模式 (0x200F = 0) ← 关键! 否则速度指令不生效
          3. 设速度模式 (0x6060 = 3)
          4. 设加减速时间 (0x6083/0x6084, sub 1/2 分左右通道)
          5. 初始速度 = 0 (0x60FF sub 1/2)
          6. CiA402 状态机: Shutdown → SwitchOn → EnableOp → EnableOp(重复)

        返回: bool 使能是否成功
        """
        nid = self.node_id

        # ---- 1. 故障复位 (上升沿触发 bit7) ----
        self.sdo_write(OBJ_CONTROLWORD, 0, 0x0000, 2)
        time.sleep(0.05)
        self.sdo_write(OBJ_CONTROLWORD, 0, CW_FAULT_RESET, 2)
        time.sleep(0.3)
        self.sdo_write(OBJ_CONTROLWORD, 0, 0x0000, 2)
        time.sleep(0.3)

        # ---- 2. 设异步模式 (0x200F = 0) ----
        # 默认 0x200F=1 (同步)，速度指令会被缓存不执行!
        if self.sdo_write(0x200F, 0, 0, 2):
            self._log_info(f'  ZLAC Node {nid}: 异步模式已设置 (0x200F=0)')
        else:
            self._log_warn(f'  ZLAC Node {nid}: 0x200F 写入失败，继续尝试')

        # ---- 3. 速度模式 ----
        self.sdo_write(OBJ_MODES_OF_OP, 0, MODE_PROFILE_VEL, 1)
        time.sleep(0.05)

        # ---- 4. 加减速 (sub 1=左, sub 2=右) ----
        # sub 0 在某些固件上只读，用 sub 1/2 分别设置
        accel = 500  # ms
        decel = 500
        self.sdo_write(OBJ_PROFILE_ACCEL, 1, accel, 4)
        self.sdo_write(OBJ_PROFILE_ACCEL, 2, accel, 4)
        self.sdo_write(OBJ_PROFILE_DECEL, 1, decel, 4)
        self.sdo_write(OBJ_PROFILE_DECEL, 2, decel, 4)

        # ---- 5. 初始速度 = 0 (sub 1/2, 不能用 sub 0!) ----
        self.sdo_write(OBJ_TARGET_VEL, 1, 0, 4)
        self.sdo_write(OBJ_TARGET_VEL, 2, 0, 4)

        # ---- 6. CiA402 状态机 ----
        for cw_val, desc in [(CW_SHUTDOWN, 'Shutdown'),
                              (CW_SWITCH_ON, 'SwitchOn'),
                              (CW_ENABLE_OP, 'EnableOp'),
                              (CW_ENABLE_OP, 'EnableOp(再次)')]:
            if not self.sdo_write(OBJ_CONTROLWORD, 0, cw_val, 2):
                self._log_warn(f'  ZLAC Node {nid}: {desc} 失败')
                return False
            time.sleep(0.1)

        # 验证: 读 statusword (高16位=右, 低16位=左)
        time.sleep(0.1)
        sw_raw = self.sdo_read(OBJ_STATUSWORD, 0)
        if sw_raw is not None:
            sw_left = sw_raw & 0xFFFF
            sw_right = (sw_raw >> 16) & 0xFFFF
            l_ok = (sw_left & 0x6F) == 0x27
            r_ok = (sw_right & 0x6F) == 0x27
            if l_ok:
                self._log_info(
                    f'  ZLAC Node {nid}: ✓ 使能成功 '
                    f'(L=0x{sw_left:04X} R=0x{sw_right:04X})')
            else:
                self._log_warn(
                    f'  ZLAC Node {nid}: 使能异常 '
                    f'(L=0x{sw_left:04X} R=0x{sw_right:04X})')
            return l_ok
        return False

    def disable_motor(self):
        """断电"""
        self.sdo_write(OBJ_CONTROLWORD, 0, CW_DISABLE_VOLT, 2)

    def set_velocity_mode(self):
        """设置速度模式 (mode=3)"""
        ok = self.sdo_write(OBJ_MODES_OF_OP, 0, MODE_PROFILE_VEL, 1)
        if ok:
            time.sleep(0.05)
            mode = self.sdo_read(OBJ_MODES_OF_OP_DISP, 0)
            if mode is not None and (mode & 0xFF) == MODE_PROFILE_VEL:
                self._log_info(f'  ZLAC Node {self.node_id}: 速度模式已设置')
                return True
            self._log_warn(f'  ZLAC Node {self.node_id}: 模式回读不匹配')
        return False

    def set_target_velocity_raw(self, rpm_int, subindex=1):
        """
        发送目标速度 (RPM)，不等待确认 (fire-and-forget)。

        ZLAC8015D 双通道对象的 sub-index:
          sub 0 = 只读 (公共值)
          sub 1 = 左电机
          sub 2 = 右电机
          sub 3 = 组合 (高16位=右, 低16位=左)

        参数:
          rpm_int: 目标转速 (int, 有符号, 单位 RPM)
          subindex: 1=左, 2=右 (默认左)
        """
        if rpm_int < 0:
            val = rpm_int & 0xFFFFFFFF
        else:
            val = rpm_int

        data = [
            SDO_WRITE_4B,
            OBJ_TARGET_VEL & 0xFF, (OBJ_TARGET_VEL >> 8) & 0xFF,
            subindex,  # sub 1=左, 2=右
            val & 0xFF, (val >> 8) & 0xFF,
            (val >> 16) & 0xFF, (val >> 24) & 0xFF,
        ]
        self._send_can(self.sdo_tx_cobid, data)


# ============================================================
#  ROS2 节点
# ============================================================

class SwerveDMDriverNode(Node):
    """
    舵轮底盘电机驱动节点。

    集成 DM4310 转向控制 + ZLAC8015D 驱动控制，
    共用达妙 USB-CAN 适配器。
    """

    def __init__(self):
        super().__init__('swerve_dm_driver_node')

        # ============== DM4310 参数 ==============
        self.declare_parameter('serial_port', 'auto')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('motor_ids', [3, 6, 4, 5])
        self.declare_parameter('master_id_offset', 0x10)
        self.declare_parameter('steer_kp', 15.0)
        self.declare_parameter('steer_kd', 1.5)
        self.declare_parameter('steer_pos_limit', 3.14)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('control_hz', 200)
        self.declare_parameter('feedback_hz', 20)

        # ============== ZLAC8015D 参数 ==============
        self.declare_parameter('zlac_enabled', True)
        self.declare_parameter('zlac_node_id', 1)         # CAN 默认 Node ID = 1
        self.declare_parameter('zlac_max_rpm', 200)
        self.declare_parameter('zlac_profile_accel', 500)  # RPM/s
        self.declare_parameter('zlac_profile_decel', 500)
        self.declare_parameter('zlac_gear_ratio', 1.0)
        # 驱动轮方向修正: swerve_cmd 的 4 个驱动速度分别乘以这个系数
        # 左右轮安装方向相反时需要反转
        self.declare_parameter('zlac_direction', [1, -1, 1, -1])
        # 驱动轮映射: swerve_cmd[4+i] 对应 ZLAC 的哪个通道
        # 当前只有 1 个轮，只处理第一个非零速度
        self.declare_parameter('zlac_wheel_index', 0)

        # ---- 读取 DM 参数 ----
        port_param = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        motor_ids = list(self.get_parameter('motor_ids').value)
        master_offset = self.get_parameter('master_id_offset').value
        self.steer_kp = self.get_parameter('steer_kp').value
        self.steer_kd = self.get_parameter('steer_kd').value
        self.steer_pos_limit = self.get_parameter('steer_pos_limit').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        control_hz = self.get_parameter('control_hz').value
        feedback_hz = self.get_parameter('feedback_hz').value

        # ---- 读取 ZLAC 参数 ----
        self.zlac_enabled = self.get_parameter('zlac_enabled').value
        zlac_node_id = self.get_parameter('zlac_node_id').value
        self.zlac_max_rpm = self.get_parameter('zlac_max_rpm').value
        zlac_accel = self.get_parameter('zlac_profile_accel').value
        zlac_decel = self.get_parameter('zlac_profile_decel').value
        self.zlac_gear_ratio = self.get_parameter('zlac_gear_ratio').value
        self.zlac_direction = list(self.get_parameter('zlac_direction').value)
        self.zlac_wheel_index = self.get_parameter('zlac_wheel_index').value
        while len(self.zlac_direction) < 4:
            self.zlac_direction.append(1)

        labels = ['FL', 'FR', 'RL', 'RR']

        # ---- 自动扫描或使用指定端口 ----
        if port_param == 'auto':
            self.get_logger().info('自动扫描 HDSC CDC (达妙 USB-CAN) ...')
            port = find_hdsc_cdc_port()
            if port is None:
                self.get_logger().error('未找到 HDSC CDC 设备!')
                raise RuntimeError('No HDSC CDC device found')
            self.get_logger().info('  找到: %s' % port)
        else:
            port = port_param

        self.get_logger().info('=' * 55)
        self.get_logger().info('舵轮底盘电机驱动节点 (DM4310 + ZLAC8015D)')
        self.get_logger().info('  串口: %s @ %d' % (port, baudrate))
        self.get_logger().info('  DM4310 ID: %s (%s)' % (
            ['0x%02X' % m for m in motor_ids], '/'.join(labels)))
        self.get_logger().info('  Kp=%.1f  Kd=%.1f  控制=%dHz' % (
            self.steer_kp, self.steer_kd, control_hz))
        if self.zlac_enabled:
            self.get_logger().info('  ZLAC Node ID: %d  最大RPM: %d' % (
                zlac_node_id, self.zlac_max_rpm))
        self.get_logger().info('=' * 55)

        # ---- 打开串口 ----
        import serial
        self.ser = serial.Serial(port, baudrate, timeout=0.5)
        self.mc = MotorControl(self.ser)

        # ============================================
        #  DM4310 初始化 (转向电机)
        # ============================================
        self.motors = []
        self.motor_online = []
        self.motor_ids = motor_ids
        self.labels = labels

        for i, sid in enumerate(motor_ids):
            mid = sid + master_offset
            motor = Motor(DM_Motor_Type.DM4310, sid, mid)
            self.mc.addMotor(motor)

            pmax = self.mc.read_motor_param(motor, DM_variable.PMAX)
            if pmax is not None:
                self.motors.append(motor)
                self.motor_online.append(True)
                self.get_logger().info('  DM ✓ %s (0x%02X): 在线' % (labels[i], sid))
            else:
                self.motors.append(None)
                self.motor_online.append(False)
                self.get_logger().warn('  DM ✗ %s (0x%02X): 离线' % (labels[i], sid))

        dm_online = sum(self.motor_online)

        # ---- DM: 清除错误 + 设零 + 使能 ----
        if dm_online > 0:
            self.get_logger().info('DM4310 清除错误并初始化...')
            for i, motor in enumerate(self.motors):
                if motor is None:
                    continue

                err = clear_motor_errors(self.mc, motor)
                if err == 1:
                    self.get_logger().info('  %s: ERR 正常' % labels[i])
                else:
                    self.get_logger().warn('  %s: ERR=0x%X (尝试继续)' % (labels[i], err))

                self.mc.disable(motor)
                time.sleep(0.1)
                self.mc.set_zero_position(motor)
                time.sleep(0.15)
                self.mc.switchControlMode(motor, Control_Type.MIT)
                time.sleep(0.05)
                self.mc.enable(motor)
                time.sleep(0.05)
                self.get_logger().info('  %s: 已使能' % labels[i])

        # ============================================
        #  ZLAC8015D 初始化 (驱动轮)
        # ============================================
        self.zlac = None
        self.zlac_online = False

        if self.zlac_enabled:
            self.get_logger().info('ZLAC8015D 初始化...')

            # 自动扫描 Node ID: 先试配置值，再试常见 ID
            scan_ids = [zlac_node_id] + [x for x in [1, 2, 3, 4, 5] if x != zlac_node_id]
            found_id = None

            # 先切达妙适配器到 500Kbps CAN (ZLAC 默认 500K)
            self.get_logger().info('  达妙适配器 → CAN 500Kbps')
            self.ser.write(bytes([0x55, 0x05, 0x03, 0xAA, 0x55]))
            time.sleep(0.5)
            self.ser.read_all()

            for try_id in scan_ids:
                self.get_logger().info(f'  尝试 Node ID={try_id}...')
                zlac = ZlacCANopen(self.ser, try_id, self.get_logger())

                # NMT Reset → Start (完整序列)
                zlac.nmt_reset_comm()
                time.sleep(0.5)
                zlac.nmt_start()
                time.sleep(0.3)

                # 读状态字确认在线
                sw = zlac.read_statusword()
                if sw is not None:
                    self.get_logger().info(
                        f'  ✓ ZLAC 在线! Node ID={try_id} (SW=0x{sw:04X})')
                    found_id = try_id
                    self.zlac = zlac
                    break
                else:
                    self.get_logger().info(f'    Node ID={try_id} 无响应')

            if found_id is None:
                self.get_logger().warn(
                    '  ZLAC 全部无响应 (扫描了 ID %s)! 可能原因:\n'
                    '    1. CAN 线未连接 (J4: Pin1=CANH, Pin3=CANL)\n'
                    '    2. ZLAC 未上电 (绿灯应亮)\n'
                    '    3. 两通道都需接电机 (否则 HALL 故障阻塞状态机)\n'
                    '  驱动轮功能已禁用' % str(scan_ids))
            else:
                # enable_motor 包含完整初始化:
                # 故障复位 → 异步模式 → 速度模式 → 加减速 → CiA402 使能
                if self.zlac.enable_motor():
                    self.zlac_online = True
                else:
                    self.get_logger().warn('  ZLAC 使能失败')

        # ---- 至少有一个电机可用就继续 ----
        if dm_online == 0 and not self.zlac_online:
            self.get_logger().warn(
                '没有任何在线电机! 节点仍启动，等待手动重连...')

        # ---- 状态变量 ----
        self.lock = threading.Lock()
        self.target_steer = [0.0] * 4
        self.target_drive = [0.0] * 4
        self.last_cmd_time = 0.0
        self.running = True

        # ---- ROS2 订阅/发布 ----
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.sub_cmd = self.create_subscription(
            Float64MultiArray, '/mujoco/swerve_cmd',
            self.cmd_callback, qos)
        self.pub_feedback = self.create_publisher(
            JointState, '/swerve/motor_feedback', qos)

        # ---- 定时器 ----
        self.ctrl_timer = self.create_timer(1.0 / control_hz, self.control_loop)
        self.fb_timer = self.create_timer(1.0 / feedback_hz, self.publish_feedback)

        status = []
        if dm_online > 0:
            status.append(f'DM {dm_online}/4')
        if self.zlac_online:
            status.append('ZLAC ✓')
        self.get_logger().info(
            f'✓ 驱动就绪 ({", ".join(status)}), 等待 /mujoco/swerve_cmd ...')

    def cmd_callback(self, msg):
        """
        接收 swerve_cmd 回调。

        msg.data[0:4] = 转向角 (rad) → DM4310
        msg.data[4:8] = 驱动速度 (rad/s) → ZLAC8015D
        """
        if len(msg.data) < 8:
            return
        with self.lock:
            for i in range(4):
                self.target_steer[i] = msg.data[i]
                self.target_drive[i] = msg.data[4 + i]
            self.last_cmd_time = time.time()

    def control_loop(self):
        """
        200Hz 控制循环。

        先发 DM4310 转向指令 (MIT 位置控制)，
        再发 ZLAC8015D 驱动指令 (CANopen 速度控制)。
        """
        if not self.running:
            return

        with self.lock:
            steer = list(self.target_steer)
            drive = list(self.target_drive)
            dt = time.time() - self.last_cmd_time if self.last_cmd_time > 0 else 999

        # 超时保护
        if dt > self.cmd_timeout:
            drive = [0.0] * 4

        # ---- DM4310 转向 ----
        for i, motor in enumerate(self.motors):
            if motor is None:
                continue
            q = max(-self.steer_pos_limit, min(self.steer_pos_limit, steer[i]))
            try:
                self.mc.controlMIT(motor, self.steer_kp, self.steer_kd,
                                   q, 0.0, 0.0)
            except Exception:
                pass

        # ---- ZLAC8015D 驱动 (双通道: sub1=左, sub2=右) ----
        if self.zlac_online:
            # swerve_cmd[4] = FL 驱动速度 → ZLAC 左通道 (sub 1)
            # swerve_cmd[5] = FR 驱动速度 → ZLAC 右通道 (sub 2)
            # 当前只有一个 ZLAC (双通道)，映射前两个驱动轮
            for ch, sub in [(0, 1), (1, 2)]:  # ch=swerve索引, sub=ZLAC子索引
                speed_rad_s = drive[ch] if ch < len(drive) else 0.0

                # rad/s → RPM (考虑减速比和方向)
                rpm = speed_rad_s * RAD_S_TO_RPM * self.zlac_gear_ratio
                rpm *= self.zlac_direction[ch]

                # 限幅
                rpm = max(-self.zlac_max_rpm, min(self.zlac_max_rpm, rpm))

                try:
                    self.zlac.set_target_velocity_raw(int(rpm), subindex=sub)
                except Exception:
                    pass

    def publish_feedback(self):
        """发布电机反馈 (DM4310 转向 + ZLAC 驱动状态)"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # DM4310 反馈
        steer_labels = ['fl_steer', 'fr_steer', 'rl_steer', 'rr_steer']
        for i, motor in enumerate(self.motors):
            msg.name.append(steer_labels[i])
            if motor is None:
                msg.position.append(0.0)
                msg.velocity.append(0.0)
                msg.effort.append(0.0)
            else:
                msg.position.append(float(motor.getPosition()))
                msg.velocity.append(float(motor.getVelocity()))
                msg.effort.append(float(motor.getTorque()))

        # ZLAC 驱动反馈 (仅标记在线状态，不做 SDO 读取避免阻塞)
        if self.zlac_online:
            msg.name.append('drive_zlac')
            msg.position.append(0.0)
            msg.velocity.append(0.0)  # TODO: 低频 SDO 读取实际速度
            msg.effort.append(0.0)

        self.pub_feedback.publish(msg)

    def destroy_node(self):
        """安全关闭: 先停 ZLAC → 再停 DM → 关串口"""
        self.running = False
        self.get_logger().info('关闭中...')

        # ZLAC: 左右通道速度归零 → 断电
        if self.zlac_online:
            try:
                self.zlac.set_target_velocity_raw(0, subindex=1)
                self.zlac.set_target_velocity_raw(0, subindex=2)
                time.sleep(0.05)
                self.zlac.disable_motor()
                self.get_logger().info('  ZLAC 已停止')
            except Exception:
                pass

        # DM4310: 零力矩 → 失能
        for motor in self.motors:
            if motor is None:
                continue
            try:
                self.mc.controlMIT(motor, 0.0, 1.0, 0.0, 0.0, 0.0)
            except Exception:
                pass
        time.sleep(0.05)
        for motor in self.motors:
            if motor is None:
                continue
            try:
                self.mc.disable(motor)
            except Exception:
                pass
            time.sleep(0.02)

        # 关串口
        try:
            self.ser.close()
        except Exception:
            pass

        self.get_logger().info('已关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SwerveDMDriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
