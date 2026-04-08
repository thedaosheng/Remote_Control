#!/usr/bin/env python3
"""
20260328-cc-airbot_force_sim.py
================================
Airbot Play 六轴机械臂 MuJoCo 力控仿真 + 力反馈笔遥操作系统

核心功能：
1. MuJoCo 物理仿真 + 可视化渲染
2. 任务空间阻抗控制（impedance control）— 力控模式
3. 直接集成 3D Systems Touch 力反馈笔驱动
4. 碰撞接触力检测 → 力反馈笔力输出
5. 可调延迟缓冲区（模拟通信延迟）
6. 夹爪控制（笔上按钮1=闭合/抓取, 按钮2=张开/释放）
7. 可选 UDP 模式（与外部笔进程通信）

运行方式：
  # 直接连接力反馈笔（默认）
  python3 20260328-cc-airbot_force_sim.py

  # 使用 UDP 模式（与外部进程通信）
  python3 20260328-cc-airbot_force_sim.py --mode udp

  # 调节延迟
  python3 20260328-cc-airbot_force_sim.py --cmd-delay 50 --fb-delay 50

  # 无输入模式（仅查看仿真环境）
  python3 20260328-cc-airbot_force_sim.py --mode none
"""

import os
import sys
import time
import math
import signal
import socket
import json
import argparse
import threading
import importlib.util
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple

# 自动设置 DISPLAY 环境变量（支持 VNC/X11 远程显示）
if "DISPLAY" not in os.environ:
    os.environ["DISPLAY"] = ":0"

import numpy as np
import mujoco
import mujoco.viewer
import matplotlib
matplotlib.use("TkAgg")          # VNC 环境下用 TkAgg；如报错可改 "Agg"（不弹窗，只存图）
import matplotlib.pyplot as plt


# ============================================================================
# 力反馈事件记录器
# ============================================================================

class ForceLogger:
    """
    记录每次接触事件中笔收到的三轴力数据，仿真结束后绘图。

    逻辑：
      - 当 |pen_force| > threshold 时视为接触开始，记录为一个事件
      - 当 |pen_force| ≤ threshold 时视为接触结束
      - 最多保留最近 max_events 次事件（先进先出淘汰旧的）
    """

    def __init__(self, threshold: float = 0.05, max_events: int = 9):
        """
        参数:
            threshold: 笔力幅值阈值 (N)，超过则认为有接触
            max_events: 最多保留的事件数，超出时丢弃最早的
        """
        self.threshold = threshold
        self.max_events = max_events

        # 已完成的事件列表，每个元素是 dict:
        #   {"t": [float,...], "fx": [...], "fy": [...], "fz": [...]}
        self._events: list = []

        # 当前正在记录的事件（None 表示不在接触中）
        self._current: Optional[dict] = None
        self._sim_time_ref: float = 0.0   # 当前事件的起始仿真时间

    def update(self, sim_time: float, fx: float, fy: float, fz: float):
        """
        每帧调用一次，传入当前仿真时间和笔实际输出的三轴力。

        参数:
            sim_time: 当前仿真时间 (s)
            fx, fy, fz: 发给笔的力 (N)，已经过坐标变换和缩放
        """
        mag = (fx**2 + fy**2 + fz**2) ** 0.5

        if mag > self.threshold:
            # 接触中
            if self._current is None:
                # 新接触事件开始
                self._current = {"t": [], "fx": [], "fy": [], "fz": []}
                self._sim_time_ref = sim_time
            self._current["t"].append(sim_time - self._sim_time_ref)
            self._current["fx"].append(fx)
            self._current["fy"].append(fy)
            self._current["fz"].append(fz)
        else:
            # 接触结束
            if self._current is not None and len(self._current["t"]) > 0:
                self._events.append(self._current)
                # 超出上限则丢弃最早的事件
                if len(self._events) > self.max_events:
                    self._events.pop(0)
            self._current = None

    def finalize(self):
        """仿真结束时调用，把正在记录的事件也保存进来。"""
        if self._current is not None and len(self._current["t"]) > 0:
            self._events.append(self._current)
            if len(self._events) > self.max_events:
                self._events.pop(0)
            self._current = None

    def plot(self):
        """
        绘制最近 N 次接触事件的三轴力时间曲线。

        每个事件一列子图（最多 max_events 列），行固定为 3（Fx / Fy / Fz）。
        若没有任何记录则跳过。
        """
        if not self._events:
            print("[ForceLogger] 本次仿真未检测到任何接触事件，无图可画。")
            return

        n = len(self._events)
        print(f"\n[ForceLogger] 共记录 {n} 次接触事件，正在绘图...")

        fig, axes = plt.subplots(3, n, figsize=(max(6, 3 * n), 7), squeeze=False)
        fig.suptitle(f"力反馈笔三轴受力（最近 {n} 次接触）", fontsize=13)

        colors = {"fx": "#e74c3c", "fy": "#2ecc71", "fz": "#3498db"}
        labels = {"fx": "Fx (N)", "fy": "Fy (N)", "fz": "Fz (N)"}

        for col, ev in enumerate(self._events):
            t = ev["t"]
            for row, axis in enumerate(["fx", "fy", "fz"]):
                ax = axes[row][col]
                ax.plot(t, ev[axis], color=colors[axis], linewidth=1.2)
                ax.axhline(0, color="gray", linewidth=0.5, linestyle="--")
                ax.set_ylabel(labels[axis] if col == 0 else "")
                ax.set_xlabel("时间 (s)" if row == 2 else "")
                if row == 0:
                    ax.set_title(f"事件 {col + 1}")
                ax.grid(True, alpha=0.3)
                # 标注峰值
                peak = max(abs(v) for v in ev[axis]) if ev[axis] else 0
                ax.set_title(
                    (f"事件 {col + 1}\n峰值 {peak:.2f}N" if row == 0
                     else ax.get_title())
                )

        plt.tight_layout()

        # 同时保存到文件（方便截图）
        save_path = f"/tmp/force_log_{int(time.time())}.png"
        plt.savefig(save_path, dpi=120)
        print(f"[ForceLogger] 图已保存到 {save_path}")

        plt.show()


# ============================================================================
# 延迟缓冲区
# ============================================================================

class DelayBuffer:
    """
    环形延迟缓冲区，用于模拟通信延迟

    工作原理：数据进入缓冲区后，需等待 delay_ms 毫秒才能被取出。
    当 delay_ms = 0 时，数据直接透传（无延迟）。
    """

    def __init__(self, delay_ms: float = 0.0):
        """
        初始化延迟缓冲区

        参数:
            delay_ms: 延迟时间（毫秒），0 表示无延迟
        """
        self.delay_ms = delay_ms
        self._buffer = deque()

    def push(self, data):
        """将数据推入缓冲区，附带当前时间戳"""
        self._buffer.append((time.time(), data))

    def pop(self):
        """
        取出已超过延迟时间的最新数据

        返回:
            最新可用数据，若无可用数据返回 None
        """
        # 无延迟模式：直接返回最新数据
        if self.delay_ms <= 0:
            if self._buffer:
                # 取最新的，丢弃中间的
                result = None
                while self._buffer:
                    _, data = self._buffer.popleft()
                    result = data
                return result
            return None

        # 有延迟模式：只返回已经"成熟"的数据
        now = time.time()
        delay_s = self.delay_ms / 1000.0
        result = None
        while self._buffer and (now - self._buffer[0][0]) >= delay_s:
            _, data = self._buffer.popleft()
            result = data
        return result

    def set_delay(self, delay_ms: float):
        """动态调整延迟时间"""
        self.delay_ms = max(0.0, delay_ms)


# ============================================================================
# 低通滤波器
# ============================================================================

class LowPassFilter:
    """
    一阶指数移动平均低通滤波器

    用于平滑力反馈信号，避免高频震颤
    """

    def __init__(self, alpha: float = 0.3, dim: int = 3):
        """
        参数:
            alpha: 滤波系数 (0~1)，越大越接近原始信号，越小越平滑
            dim: 信号维度
        """
        self.alpha = alpha
        self.value = np.zeros(dim)
        self._initialized = False

    def filter(self, raw: np.ndarray) -> np.ndarray:
        """
        对输入信号进行滤波

        参数:
            raw: 原始信号向量

        返回:
            滤波后的信号
        """
        if not self._initialized:
            self.value = raw.copy()
            self._initialized = True
        else:
            self.value = self.alpha * raw + (1.0 - self.alpha) * self.value
        return self.value.copy()

    def reset(self):
        """重置滤波器状态"""
        self._initialized = False
        self.value[:] = 0.0


# ============================================================================
# 力反馈笔接口抽象
# ============================================================================

@dataclass
class PenState:
    """力反馈笔状态数据"""
    # 笔尖位置 (mm)，在 Touch 设备坐标系下
    position: np.ndarray = None
    # 笔尖速度 (mm/s)
    velocity: np.ndarray = None
    # 4x4 变换矩阵（姿态信息）
    transform: Optional[np.ndarray] = None
    # 按钮状态
    button1: bool = False  # 灰色按钮 → 闭合夹爪
    button2: bool = False  # 白色按钮 → 张开夹爪
    # 时间戳
    timestamp: float = 0.0
    # 有效标志
    valid: bool = False

    def __post_init__(self):
        if self.position is None:
            self.position = np.zeros(3)
        if self.velocity is None:
            self.velocity = np.zeros(3)


class HapticInterface:
    """力反馈笔接口基类"""

    def get_state(self) -> PenState:
        """获取笔的最新状态"""
        raise NotImplementedError

    def set_force(self, fx: float, fy: float, fz: float):
        """设置力反馈输出 (N)"""
        raise NotImplementedError

    def is_connected(self) -> bool:
        """检查设备是否已连接"""
        return False

    def close(self):
        """关闭设备"""
        pass


class DirectHapticInterface(HapticInterface):
    """
    直接连接 3D Systems Touch 力反馈笔

    使用同目录下的 haptic_driver.py 驱动，
    通过 OpenHaptics HD API 直接控制设备。
    """

    def __init__(self):
        """加载并初始化力反馈笔驱动"""
        # 动态导入 haptic_driver 模块（文件名含日期前缀）
        driver_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "20260328-cc-haptic_driver.py"
        )
        if not os.path.exists(driver_path):
            raise FileNotFoundError(f"找不到力反馈笔驱动: {driver_path}")

        # 使用 importlib 加载带日期前缀的模块
        spec = importlib.util.spec_from_file_location("haptic_driver", driver_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        # 创建设备实例
        self._device = module.HapticDevice()
        self._device.initialize()
        self._device.start_scheduler()
        self._connected = True
        print("[DirectHaptic] 力反馈笔已连接并启动")

    def get_state(self) -> PenState:
        """从驱动读取笔的最新状态"""
        state = self._device.get_state()
        pen = PenState()
        pen.position = np.array(state.position)
        pen.velocity = np.array(state.velocity)
        pen.button1 = state.button1
        pen.button2 = state.button2
        pen.timestamp = state.timestamp
        pen.valid = True
        # 构造变换矩阵 (4x4)
        if state.transform and len(state.transform) == 16:
            pen.transform = np.array(state.transform).reshape(4, 4)
        return pen

    def set_force(self, fx: float, fy: float, fz: float):
        """设置力反馈输出"""
        self._device.set_force(fx, fy, fz)

    def is_connected(self) -> bool:
        return self._connected

    def close(self):
        """停止设备"""
        if self._connected:
            self._device.stop()
            self._connected = False
            print("[DirectHaptic] 力反馈笔已断开")


class UDPHapticInterface(HapticInterface):
    """
    通过 UDP 与外部力反馈笔进程通信

    协议格式 (JSON):
      接收 (笔→仿真): {"pos": [x,y,z], "vel": [vx,vy,vz], "buttons": [b1,b2]}
      发送 (仿真→笔): {"force": [fx,fy,fz], "endpoint": [x,y,z], "contact": bool}
    """

    def __init__(self, recv_port: int = 12345, send_port: int = 12346,
                 send_ip: str = "127.0.0.1"):
        """
        参数:
            recv_port: 接收笔数据的本地端口
            send_port: 发送力反馈的目标端口
            send_ip: 目标 IP 地址
        """
        # 接收 socket（非阻塞）
        self._recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_sock.bind(("0.0.0.0", recv_port))
        self._recv_sock.setblocking(False)

        # 发送 socket
        self._send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._send_addr = (send_ip, send_port)

        self._last_state = PenState()
        self._connected = True
        print(f"[UDP] 监听端口 {recv_port}，发送到 {send_ip}:{send_port}")

    def get_state(self) -> PenState:
        """从 UDP 接收笔状态"""
        try:
            # 读取所有可用数据，保留最新的
            latest_data = None
            while True:
                try:
                    data, addr = self._recv_sock.recvfrom(4096)
                    latest_data = data
                except BlockingIOError:
                    break

            if latest_data:
                msg = json.loads(latest_data.decode("utf-8"))
                pen = PenState()
                pen.position = np.array(msg.get("pos", [0, 0, 0]), dtype=float)
                pen.velocity = np.array(msg.get("vel", [0, 0, 0]), dtype=float)
                buttons = msg.get("buttons", [0, 0])
                pen.button1 = bool(buttons[0])
                pen.button2 = bool(buttons[1])
                pen.timestamp = time.time()
                pen.valid = True
                self._last_state = pen
                return pen
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            print(f"[UDP] 数据解析错误: {e}")

        return self._last_state

    def set_force(self, fx: float, fy: float, fz: float):
        """通过 UDP 发送力反馈"""
        msg = json.dumps({
            "force": [round(fx, 4), round(fy, 4), round(fz, 4)]
        })
        try:
            self._send_sock.sendto(msg.encode("utf-8"), self._send_addr)
        except OSError:
            pass

    def is_connected(self) -> bool:
        return self._connected

    def close(self):
        self._recv_sock.close()
        self._send_sock.close()
        self._connected = False


class DummyHapticInterface(HapticInterface):
    """空接口 — 无输入设备，仅用于查看仿真环境"""

    def get_state(self) -> PenState:
        return PenState()

    def set_force(self, fx, fy, fz):
        pass

    def is_connected(self) -> bool:
        return True


# ============================================================================
# 主仿真类
# ============================================================================

class AirbotForceSim:
    """
    Airbot Play MuJoCo 力控仿真器

    核心控制算法：
      - 任务空间阻抗控制 (Task-space Impedance Control)
      - 力 = Kp * (位置误差) - Kd * (末端速度) + 重力补偿
      - 关节力矩 = J^T * 任务空间力 + 重力/科里奥利补偿
    """

    def __init__(self, args):
        """
        初始化仿真器

        参数:
            args: 命令行参数（包含模型路径、增益、延迟等配置）
        """
        # ---- 加载 MuJoCo 模型 ----
        model_path = args.model
        if not os.path.isabs(model_path):
            model_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)), model_path
            )
        print(f"[仿真] 加载模型: {model_path}")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # ---- MuJoCo ID 查找 ----
        # 末端点 site
        self.endpoint_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "endpoint"
        )
        # 底座 site（用作参考坐标系）
        self.armbase_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "armbase"
        )
        # 目标可视化 mocap body
        self.target_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "target"
        )
        # 只检测左夹爪指尖（g2_left_link）的接触力，回传给力反馈笔
        self.contact_body_ids = []
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "g2_left_link")
        if bid >= 0:
            self.contact_body_ids.append(bid)

        # 机械臂关节数量（前 6 个 DOF）
        self.n_arm_joints = 6
        # 总自由度数
        self.nv = self.model.nv

        # ---- 阻抗控制增益 ----
        # 位置增益 (N/m)：控制末端跟踪精度
        self.Kp_pos = np.diag([args.kp_pos] * 3)
        # 位置阻尼 (N·s/m)：控制末端运动平滑度
        self.Kd_pos = np.diag([args.kd_pos] * 3)
        # 姿态增益 (N·m/rad)
        self.Kp_rot = np.diag([args.kp_rot] * 3)
        # 姿态阻尼 (N·m·s/rad)
        self.Kd_rot = np.diag([args.kd_rot] * 3)
        # 空间位置零空间阻尼（防止多余自由度漂移）
        self.null_space_damping = args.null_damping

        # ---- 工作空间映射 ----
        # 位置缩放: 笔 mm → 机器人 m
        self.pos_scale = args.pos_scale
        # 工作空间中心偏移 (m)：机器人坐标系下的默认末端位置
        self.workspace_center = np.array(args.workspace_center)

        # ---- 力反馈参数 ----
        # 力缩放系数：仿真接触力 (N) → 笔力 (N)
        self.force_scale = args.force_scale
        # 笔最大输出力 (N)
        self.max_pen_force = args.max_force
        # 力反馈低通滤波器（仍保留，用于内部原始力平滑，二值化前）
        self.force_filter = LowPassFilter(alpha=args.force_filter_alpha, dim=3)

        # ---- 二值化接触确认力 ----
        # 目标：给操作者"碰到了"的确认感，而非精准力传递
        # contact_threshold: 判定"有接触"的最低接触力阈值 (N)
        self.contact_threshold = args.contact_threshold
        # contact_confirm_force: 有接触时固定输出给笔的力大小 (N)
        # 不管碰多重，都输出这个固定值
        self.contact_confirm_force = args.contact_confirm_force

        # ---- 姿态旋转缩放 ----
        # rot_scale: 笔姿态变化倍率（>1 表示放大）
        # 原因：用户实际操作空间比机械臂操作空间小，
        # 因此把小幅度的笔旋转放大为机械臂的大幅度旋转
        self.rot_scale = args.rot_scale

        # ---- 虚拟墙（力位混控）----
        # 原理：接触发生时记录末端位置作为"墙面"，
        # 后续 desired_pos 被约束在墙面之外（不再穿入），
        # 同时力反馈按穿入深度线性增加（弹簧感，而非恒力）。
        # wall_stiffness: 虚拟墙弹簧刚度 (N/m)，作用在 robot 坐标空间
        self.wall_stiffness = args.wall_stiffness
        self.wall_active = False        # 当前是否检测到接触（墙激活）
        self.wall_entry_pos = None      # 首次接触时记录的末端位置（墙面参考点）
        self.wall_normal = None         # 墙法向量（接触力方向，归一化）
        # 笔映射后的"原始"目标位置（墙约束施加前），用于计算穿入深度
        self._raw_desired_pos = None

        # ---- 延迟缓冲区 ----
        self.cmd_delay = DelayBuffer(args.cmd_delay)
        self.fb_delay = DelayBuffer(args.fb_delay)

        # ---- 控制状态 ----
        self.desired_pos = None           # 目标末端位置 (m)
        self.desired_quat = None          # 目标末端姿态 (四元数)
        # 夹爪目标值（官方 g2 夹爪：0=闭合，0.0366=全开）
        self.gripper_target = 0.0366
        self.pen_origin = None            # 笔的原点（首次有效数据时记录）
        self.pen_offset_initialized = False

        # ---- 姿态跟随状态 ----
        # pen_R0: 笔的初始旋转矩阵（3x3），用于计算相对旋转
        self.pen_R0 = None
        # robot_R0: 机器人初始末端旋转矩阵（3x3），叠加笔的相对旋转
        self.robot_R0 = None
        # R_map: 笔坐标系→机器人坐标系的旋转映射（与位置映射一致）
        # 用户面向机器人：Touch X→Robot -Y（取反），Touch Y→Robot Z，Touch -Z→Robot X
        # R_map 行对应 Robot 轴，列对应 Touch 轴
        self.R_map = np.array([[ 0,  0, -1],
                               [-1,  0,  0],
                               [ 0,  1,  0]], dtype=float)

        # ---- 运行状态 ----
        self.running = True
        self.sim_time = 0.0
        self.step_count = 0
        self.print_interval = int(1.0 / self.model.opt.timestep / 10)  # ~10Hz 打印
        self._btn1_last = False   # 灰色按钮上一帧状态（用于上升沿检测，切换夹爪）
        self._btn2_last = False   # 白色按钮上一帧状态（用于上升沿检测，重置+标定）

        # ---- 力反馈事件记录器 ----
        # 阈值 0.05N：笔有实际输出时才计入事件
        self.force_logger = ForceLogger(threshold=0.05, max_events=9)

        # ---- 力反馈笔接口 ----
        self.haptic = self._create_haptic_interface(args)

        # ---- 重置到初始位姿 ----
        self._reset_to_home()

        print("[仿真] 初始化完成")
        print(f"  物理步长: {self.model.opt.timestep*1000:.1f} ms")
        print(f"  位置增益 Kp: {args.kp_pos} N/m")
        print(f"  阻尼增益 Kd: {args.kd_pos} N·s/m")
        print(f"  位置缩放: {self.pos_scale}")
        print(f"  力缩放: {self.force_scale}")
        print(f"  指令延迟: {args.cmd_delay} ms")
        print(f"  反馈延迟: {args.fb_delay} ms")

    def _create_haptic_interface(self, args) -> HapticInterface:
        """
        根据命令行参数创建力反馈笔接口

        支持三种模式:
          - haptic: 直接连接 3D Systems Touch
          - udp: UDP 网络通信
          - none: 无输入（仅查看）
        """
        mode = args.mode
        if mode == "haptic":
            try:
                return DirectHapticInterface()
            except Exception as e:
                print(f"[警告] 无法连接力反馈笔: {e}")
                print("[警告] 回退到 UDP 模式")
                mode = "udp"

        if mode == "udp":
            return UDPHapticInterface(
                recv_port=args.udp_recv_port,
                send_port=args.udp_send_port,
                send_ip=args.udp_send_ip
            )

        return DummyHapticInterface()

    def _reset_to_home(self):
        """重置仿真到 home 关键帧位姿（所有关节归零）"""
        # 查找 "home" 关键帧
        key_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_KEY, "home"
        )
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        else:
            mujoco.mj_resetData(self.model, self.data)

        # 前向动力学计算（更新所有位姿）
        mujoco.mj_forward(self.model, self.data)

        # 将当前末端位置设为初始目标（FK 实际计算结果）
        self.desired_pos = self.data.site_xpos[self.endpoint_site_id].copy()

        # ★ 自适应 workspace_center：使其等于零位 FK 末端位置
        # 这样当笔在初始（静止）位置时，desired_pos 与 FK 精确对齐，无跳变
        self.workspace_center = self.desired_pos.copy()

        # 将当前末端旋转矩阵设为初始姿态目标
        xmat = self.data.site_xmat[self.endpoint_site_id].reshape(3, 3)
        self.desired_quat = np.zeros(4)
        mujoco.mju_mat2Quat(self.desired_quat, xmat.flatten())

        # 记录机器人初始旋转矩阵（用于后续姿态跟随的基准）
        self.robot_R0 = xmat.copy()
        # 重置笔的初始旋转（等待主循环第一帧时记录）
        self.pen_R0 = None
        self.pen_offset_initialized = False

        print(f"[仿真] 初始末端位置（FK）: {self.desired_pos}")
        print(f"[仿真] 工作空间中心已自适应: {self.workspace_center}")

    # ================================================================
    # 坐标映射
    # ================================================================

    def pen_pos_to_robot(self, pen_pos_mm: np.ndarray) -> np.ndarray:
        """
        力反馈笔位置 (mm) → 机器人末端目标位置 (m)

        坐标映射关系（Touch 设备坐标系 → 机器人坐标系）：
          用户面向机器人坐，所以左右方向需取反：
          Touch X (用户右)    → Robot -Y（取反：使仿真里"右"与用户视角一致）
          Touch Y (上/下)     → Robot Z (上/下，不变)
          Touch Z (朝向用户)  → Robot -X (前/后，取反)
        """
        robot_pos = np.array([
            self.workspace_center[0] - pen_pos_mm[2] * self.pos_scale,  # Z→-X
            self.workspace_center[1] - pen_pos_mm[0] * self.pos_scale,  # X→-Y（取反）
            self.workspace_center[2] + pen_pos_mm[1] * self.pos_scale,  # Y→Z
        ])
        return robot_pos

    def robot_force_to_pen(self, robot_force: np.ndarray) -> np.ndarray:
        """
        机器人末端接触力 (N) → 力反馈笔力输出 (N)

        映射为位置映射的转置（保持虚功守恒）：
          位置映射 X→-Y 取反后，力映射 Y 分量也需取反：
          Robot Fx → Pen -Fz
          Robot Fy → Pen -Fx（取反，对应位置 X→-Y 的转置）
          Robot Fz → Pen Fy
        """
        pen_force = np.array([
            self.force_scale * (-robot_force[1]),   # -Fy_robot → Fx_pen（取反）
            self.force_scale * robot_force[2],      # Fz_robot → Fy_pen
            self.force_scale * (-robot_force[0]),   # -Fx_robot → Fz_pen
        ])

        # 限幅：保护设备和用户安全
        magnitude = np.linalg.norm(pen_force)
        if magnitude > self.max_pen_force:
            pen_force = pen_force * (self.max_pen_force / magnitude)

        return pen_force

    def _scale_rotation(self, R_delta: np.ndarray, scale: float) -> np.ndarray:
        """
        对旋转矩阵进行轴角放大

        原理：
          1. 从旋转矩阵提取轴角（axis-angle）
          2. 将角度乘以 scale
          3. 用 Rodrigues 公式重建放大后的旋转矩阵

        参数:
            R_delta: 3x3 相对旋转矩阵
            scale:   角度放大倍率（如 2.0 表示旋转量翻倍）

        返回:
            放大后的 3x3 旋转矩阵
        """
        # 从旋转矩阵的迹计算旋转角
        # trace(R) = 1 + 2*cos(θ)，因此 cos(θ) = (trace-1)/2
        trace_val = np.clip((np.trace(R_delta) - 1.0) / 2.0, -1.0, 1.0)
        angle = np.arccos(trace_val)

        # 角度极小时（≈恒等变换），直接返回原矩阵避免数值不稳定
        if angle < 1e-6:
            return R_delta.copy()

        # 从反对称部分提取旋转轴（归一化）
        # R - R^T = 2*sin(θ) * [k]_×  其中 [k]_× 为轴的反对称矩阵
        axis = np.array([
            R_delta[2, 1] - R_delta[1, 2],
            R_delta[0, 2] - R_delta[2, 0],
            R_delta[1, 0] - R_delta[0, 1]
        ]) / (2.0 * np.sin(angle))

        # 对角度进行缩放
        scaled_angle = angle * scale

        # Rodrigues 旋转公式：
        # R = I + sin(θ)*K + (1-cos(θ))*K²
        # 其中 K 是轴向量的反对称矩阵
        K = np.array([
            [       0.0, -axis[2],  axis[1]],
            [ axis[2],        0.0, -axis[0]],
            [-axis[1],   axis[0],       0.0]
        ])
        R_scaled = (np.eye(3)
                    + np.sin(scaled_angle) * K
                    + (1.0 - np.cos(scaled_angle)) * (K @ K))
        return R_scaled

    # ================================================================
    # 阻抗控制器
    # ================================================================

    def compute_impedance_control(self) -> np.ndarray:
        """
        计算任务空间阻抗控制的关节力矩

        算法流程：
        1. 获取当前末端位置、速度
        2. 计算末端雅可比矩阵 (6 x n_arm)
        3. 计算位置误差力 F = Kp * (x_des - x_cur) - Kd * v_cur
        4. 计算姿态误差力矩 T = Kp_rot * rot_err - Kd_rot * omega
        5. 通过雅可比转置映射到关节力矩: tau = J^T * [F; T]
        6. 加上重力/科里奥利补偿: tau += qfrc_bias

        返回:
            n_arm_joints 维关节力矩向量
        """
        if self.desired_pos is None:
            # 无目标时仅做重力补偿
            return self.data.qfrc_bias[:self.n_arm_joints].copy()

        site_id = self.endpoint_site_id
        n = self.n_arm_joints

        # ---- 1. 当前末端状态 ----
        pos_cur = self.data.site_xpos[site_id].copy()
        xmat_cur = self.data.site_xmat[site_id].reshape(3, 3)

        # ---- 2. 雅可比矩阵 (3xnv 平移 + 3xnv 旋转) ----
        jacp = np.zeros((3, self.nv))
        jacr = np.zeros((3, self.nv))
        mujoco.mj_jacSite(self.model, self.data, jacp, jacr, site_id)

        # 仅取前 n 列（机械臂关节部分）
        Jp = jacp[:, :n]
        Jr = jacr[:, :n]

        # ---- 3. 位置误差力 ----
        pos_err = self.desired_pos - pos_cur
        vel_cur = Jp @ self.data.qvel[:n]
        F_pos = self.Kp_pos @ pos_err - self.Kd_pos @ vel_cur

        # ---- 4. 姿态误差力矩 ----
        # 计算当前姿态四元数
        quat_cur = np.zeros(4)
        mujoco.mju_mat2Quat(quat_cur, xmat_cur.flatten())

        # 姿态误差（3D 旋转向量）
        rot_err = np.zeros(3)
        if self.desired_quat is not None:
            mujoco.mju_subQuat(rot_err, self.desired_quat, quat_cur)
        omega_cur = Jr @ self.data.qvel[:n]
        T_rot = self.Kp_rot @ rot_err - self.Kd_rot @ omega_cur

        # ---- 5. 关节力矩 = J^T * [F; T] ----
        tau = Jp.T @ F_pos + Jr.T @ T_rot

        # ---- 6. 加上重力/科里奥利补偿 ----
        tau += self.data.qfrc_bias[:n]

        # ---- 7. 空间位置零空间阻尼 ----
        # 在零空间中加入关节速度阻尼，防止自运动
        if self.null_space_damping > 0:
            tau -= self.null_space_damping * self.data.qvel[:n]

        return tau

    # ================================================================
    # 接触力检测
    # ================================================================

    def get_contact_forces(self) -> np.ndarray:
        """
        获取作用在末端（link6 + 手指）上的外部接触力

        使用 MuJoCo 的 cfrc_ext 数据，它存储了每个 body 受到的
        约束力（包含接触力），格式为 [torque(3), force(3)]。

        返回:
            世界坐标系下的 3D 接触力向量 (N)
        """
        force = np.zeros(3)
        for bid in self.contact_body_ids:
            # cfrc_ext[bid] = [tx, ty, tz, fx, fy, fz]
            # 取力的部分 (索引 3:6)
            force += self.data.cfrc_ext[bid, 3:6]
        return force

    def has_significant_contact(self, force: np.ndarray,
                                 threshold: float = 0.5) -> bool:
        """判断接触力是否显著（超过阈值）"""
        return np.linalg.norm(force) > threshold

    # ================================================================
    # 主控制循环
    # ================================================================

    def run(self):
        """
        启动仿真主循环

        循环步骤：
        1. 读取力反馈笔状态
        2. 应用指令延迟缓冲
        3. 坐标映射 → 更新期望位置
        4. 计算阻抗控制力矩
        5. 步进物理仿真
        6. 计算接触力 → 应用反馈延迟 → 发送力反馈
        7. 同步可视化渲染器
        """
        print("\n[仿真] 启动主循环...")
        print("  操作说明:")
        print("  - 力反馈笔: 移动笔尖控制机械臂末端（位置+姿态）")
        print("  - 灰色按钮 (Button1): 切换夹爪开/关（上升沿触发，按一下切换）")
        print("  - 白色按钮 (Button2): ★ 重置位置 + 更新姿态标定")
        print("    步骤: 把笔摆到'正位'(XY水平/Z向上/X朝前) → 按白色按钮")
        print("         → 机器人归零，该笔姿态 = 机器人 home 姿态（绝对映射基准）")
        print("  - Ctrl+C: 退出\n")

        # 注册信号处理（优雅退出）
        def signal_handler(sig, frame):
            print("\n[仿真] 收到退出信号...")
            self.running = False
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        # 启动 MuJoCo 被动渲染器（在独立线程中运行）
        try:
            with mujoco.viewer.launch_passive(
                self.model, self.data,
                show_left_ui=True, show_right_ui=True
            ) as viewer:
                self._main_loop(viewer)
        except Exception as e:
            print(f"[仿真] 渲染器异常: {e}")
            print("[仿真] 尝试无渲染模式...")
            self._main_loop(None)

        # 清理
        self._cleanup()

    def _main_loop(self, viewer):
        """
        核心仿真控制循环

        参数:
            viewer: MuJoCo 被动渲染器句柄（可为 None）
        """
        while self.running:
            # 检查渲染器是否仍在运行
            if viewer is not None and not viewer.is_running():
                self.running = False
                break

            step_start = time.time()

            # ==== 1. 读取力反馈笔状态 ====
            pen_state = self.haptic.get_state()

            if pen_state.valid:
                # 推入指令延迟缓冲区
                # transform 以扁平列表（16个double）存储，避免 numpy array len 歧义
                tf_list = (list(pen_state.transform.flatten())
                           if pen_state.transform is not None
                           else [])
                self.cmd_delay.push({
                    "pos": pen_state.position.copy(),
                    "vel": pen_state.velocity.copy(),
                    "button1": pen_state.button1,
                    "button2": pen_state.button2,
                    "transform": tf_list,   # 16 个 double，列主序
                })

            # ==== 2. 获取经过延迟的指令 ====
            delayed_cmd = self.cmd_delay.pop()
            if delayed_cmd is not None:
                pen_pos = delayed_cmd["pos"]
                tf_raw = delayed_cmd.get("transform", [])

                # --- 首次收到有效数据时初始化参考帧 ---
                if not self.pen_offset_initialized:
                    self.pen_origin = pen_pos.copy()
                    self.pen_offset_initialized = True
                    print(f"[仿真] 笔位置原点已记录: {self.pen_origin}")

                # --- 姿态标定：等待第一帧有效（非全零）transform ---
                # 伺服线程启动后需要几帧才有真实数据，初始 transform 可能全零
                if self.pen_R0 is None and len(tf_raw) == 16:
                    T_raw = np.array(tf_raw).reshape(4, 4)  # M^T（行主序存储的列主序矩阵）
                    R_candidate = T_raw.T[:3, :3]           # M[:3,:3] 实际旋转矩阵
                    # 验证旋转矩阵有效性：det ≈ +1，Frobenius 范数非零
                    det = np.linalg.det(R_candidate)
                    if abs(det - 1.0) < 0.1:               # 旋转矩阵行列式应为 1
                        self.pen_R0 = R_candidate.copy()
                        print(f"[仿真] 姿态标定完成，pen_R0 det={det:.4f}")

                # --- 位置跟随：相对位移 → 机器人目标位置 ---
                relative_pos = pen_pos - self.pen_origin
                self.desired_pos = self.pen_pos_to_robot(relative_pos)

                # --- 姿态跟随：笔相对旋转 → 机器人目标四元数 ---
                # 原理：ΔR_pen = R_cur @ R0^T（笔从初始姿态的相对旋转）
                # 映射到机器人坐标系：ΔR_robot = R_map @ ΔR_pen @ R_map^T
                # 叠加到机器人初始姿态：R_des = ΔR_robot @ robot_R0
                if (len(tf_raw) == 16
                        and self.pen_R0 is not None
                        and self.robot_R0 is not None):
                    T_raw = np.array(tf_raw).reshape(4, 4)  # = M^T
                    R_pen_cur = T_raw.T[:3, :3]              # = M[:3,:3] 实际旋转
                    delta_R_pen = R_pen_cur @ self.pen_R0.T  # 笔的相对旋转（相对初始姿态）
                    # ★ 姿态放大：将笔的旋转角度按 rot_scale 倍放大
                    # 目的：补偿用户操作空间比机械臂工作空间小的问题
                    delta_R_pen = self._scale_rotation(delta_R_pen, self.rot_scale)
                    # 映射到机器人坐标系（与位置映射坐标轴一致）
                    delta_R_robot = self.R_map @ delta_R_pen @ self.R_map.T
                    R_robot_des = delta_R_robot @ self.robot_R0
                    # 正交化（防止数值漂移破坏旋转矩阵正定性）
                    U, _, Vt = np.linalg.svd(R_robot_des)
                    R_robot_des = U @ Vt
                    # 转换为 MuJoCo 四元数
                    quat_new = np.zeros(4)
                    mujoco.mju_mat2Quat(quat_new, R_robot_des.flatten())
                    self.desired_quat = quat_new

                # 更新目标可视化球（位置 + 姿态，三色轴显示期望朝向）
                if self.target_body_id >= 0:
                    mocap_id = self.model.body_mocapid[self.target_body_id]
                    if mocap_id >= 0:
                        self.data.mocap_pos[mocap_id] = self.desired_pos
                        # 同步显示期望姿态（让三色轴随笔的朝向旋转）
                        if self.desired_quat is not None:
                            self.data.mocap_quat[mocap_id] = self.desired_quat

                # ====== 灰色按钮（Button1）：上升沿切换夹爪开/关 ======
                # 官方 g2 夹爪：ctrl=0=闭合，ctrl=0.0366=全开
                if delayed_cmd["button1"] and not self._btn1_last:
                    if self.gripper_target > 0.018:
                        self.gripper_target = 0.0      # 当前是开 → 切换为闭合
                        print("\n[夹爪] 闭合")
                    else:
                        self.gripper_target = 0.0366   # 当前是闭 → 切换为张开
                        print("\n[夹爪] 张开")
                self._btn1_last = delayed_cmd["button1"]

                # ====== 白色按钮（Button2）：重置位置 + 更新姿态标定 ======
                if delayed_cmd["button2"] and not self._btn2_last:
                    # 操作流程：
                    #   1. 把笔调到"正位"（X 水平向前、XY 平面平行地面、Z 垂直向上）
                    #   2. 按白色按钮
                    #   3. 机器人位置复位到 home（关节全零）
                    #   4. 当前笔位置 → 新的位置参考零点
                    #   5. 当前笔姿态 → 更新为姿态标定基准（对应机器人 home 姿态）
                    #      此后 笔 canonical 姿态 = 机器人 home 姿态，其他旋转等比跟随

                    # --- 机器人复位到 home 关键帧（位置归零）---
                    key_id = mujoco.mj_name2id(
                        self.model, mujoco.mjtObj.mjOBJ_KEY, "home"
                    )
                    if key_id >= 0:
                        mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
                    else:
                        mujoco.mj_resetData(self.model, self.data)
                    mujoco.mj_forward(self.model, self.data)

                    # --- 位置参考更新：home FK 位置作为新 workspace_center ---
                    self.workspace_center = self.data.site_xpos[self.endpoint_site_id].copy()
                    self.desired_pos = self.workspace_center.copy()

                    # --- 当前笔空间位置作为新的位置零点 ---
                    self.pen_origin = pen_pos.copy()

                    # --- 姿态标定更新：当前笔姿态 = home EE 姿态的对应关系 ---
                    # 每次按白色按钮都更新（不只是首次），让用户可以随时重新校准
                    if len(tf_raw) == 16:
                        T_raw = np.array(tf_raw).reshape(4, 4)
                        R_candidate = T_raw.T[:3, :3]
                        det = np.linalg.det(R_candidate)
                        if abs(det - 1.0) < 0.15:   # 验证是有效旋转矩阵
                            self.pen_R0 = R_candidate.copy()
                            xmat_home = self.data.site_xmat[
                                self.endpoint_site_id].reshape(3, 3)
                            self.robot_R0 = xmat_home.copy()
                            print(f"[仿真] ★ 姿态标定已更新 (det={det:.4f})")
                        else:
                            print(f"[仿真] ⚠ 姿态标定跳过（transform 无效 det={det:.4f}）")

                    print(f"\n[仿真] ★ 位置原点重置！机器人已归零 | "
                          f"笔位零点: {np.round(self.pen_origin, 1)} mm | "
                          f"workspace_center: {np.round(self.workspace_center, 3)}")
                self._btn2_last = delayed_cmd["button2"]

            # ==== 3. 虚拟墙：约束 desired_pos（力位混控核心）====
            # 保存原始目标位置（墙约束前），用于后续计算穿入深度和力反馈幅值
            self._raw_desired_pos = (self.desired_pos.copy()
                                     if self.desired_pos is not None else None)
            # 若虚拟墙激活，把 desired_pos 投影回墙面（不允许穿入）
            if (self.wall_active
                    and self.wall_entry_pos is not None
                    and self.desired_pos is not None):
                # 计算超出墙面的分量（沿法向量方向的穿入深度）
                excess = np.dot(
                    self.desired_pos - self.wall_entry_pos, self.wall_normal
                )
                if excess > 0:
                    # 把 desired_pos 推回到墙面（沿法向量方向归零超出量）
                    self.desired_pos = self.desired_pos - excess * self.wall_normal

            # ==== 4. 计算阻抗控制力矩 ====
            tau = self.compute_impedance_control()

            # 设置控制量
            self.data.ctrl[:self.n_arm_joints] = tau
            self.data.ctrl[self.n_arm_joints] = self.gripper_target

            # ==== 4. 步进物理仿真 ====
            mujoco.mj_step(self.model, self.data)
            self.sim_time += self.model.opt.timestep
            self.step_count += 1

            # ==== 5. 接触力检测 → 弹簧力直通回传 ====
            contact_force_world = self.get_contact_forces()

            # 低通滤波平滑，滤除单帧噪声
            smoothed_force = self.force_filter.filter(contact_force_world)

            # 直接将仿真接触力（经坐标映射+缩放）发给笔，无虚拟墙状态机
            self.fb_delay.push(smoothed_force.copy())
            delayed_force = self.fb_delay.pop()
            if delayed_force is not None and self.haptic.is_connected():
                pen_force = self.robot_force_to_pen(delayed_force)
                self.haptic.set_force(pen_force[0], pen_force[1], pen_force[2])
                # 记录本帧笔的三轴输出力
                self.force_logger.update(
                    self.sim_time, pen_force[0], pen_force[1], pen_force[2]
                )

            # ==== 6. 同步渲染器 ====
            if viewer is not None:
                viewer.sync()

            # ==== 7. 定时打印状态 ====
            if self.step_count % self.print_interval == 0:
                self._print_status(contact_force_world)

            # ==== 8. 时间同步 ====
            elapsed = time.time() - step_start
            remaining = self.model.opt.timestep - elapsed
            if remaining > 0:
                time.sleep(remaining)

    def _print_status(self, contact_force: np.ndarray):
        """定期打印仿真状态信息"""
        pos = self.data.site_xpos[self.endpoint_site_id]
        force_mag = np.linalg.norm(contact_force)
        contact_str = f"接触力: {force_mag:.2f}N" if force_mag > 0.1 else "无接触"

        des_str = ""
        if self.desired_pos is not None:
            err = np.linalg.norm(self.desired_pos - pos)
            des_str = f" | 跟踪误差: {err*1000:.1f}mm"

        # 官方 g2 夹爪：ctrl > 0.018 = 张开，ctrl ≤ 0.018 = 闭合
        gripper_str = "张开" if self.gripper_target > 0.018 else "闭合"

        sys.stdout.write(
            f"\r[{self.sim_time:.1f}s] "
            f"末端: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}] | "
            f"{contact_str}{des_str} | "
            f"夹爪: {gripper_str}    "
        )
        sys.stdout.flush()

    def _cleanup(self):
        """清理资源，并在仿真结束后绘制力反馈统计图"""
        print("\n[仿真] 正在清理...")
        # 清零力输出
        if self.haptic.is_connected():
            self.haptic.set_force(0.0, 0.0, 0.0)
            time.sleep(0.1)
            self.haptic.close()
        print("[仿真] 已退出")

        # 保存最后一个未结束的接触事件，然后绘图
        self.force_logger.finalize()
        self.force_logger.plot()


# ============================================================================
# 命令行参数
# ============================================================================

def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description="Airbot Play MuJoCo 力控仿真 + 力反馈笔遥操作",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
运行示例:
  # 直接连接力反馈笔
  python3 %(prog)s

  # UDP 模式
  python3 %(prog)s --mode udp

  # 添加 50ms 双向延迟
  python3 %(prog)s --cmd-delay 50 --fb-delay 50

  # 调整增益
  python3 %(prog)s --kp-pos 1500 --kd-pos 80
        """
    )

    # ---- 模式选择 ----
    parser.add_argument(
        "--mode", choices=["haptic", "udp", "none"],
        default="haptic",
        help="输入模式: haptic=直接连接笔, udp=UDP通信, none=无输入 (默认: haptic)"
    )

    # ---- 模型路径 ----
    parser.add_argument(
        "--model", default="20260328-cc-airbot_mujoco_scene.xml",
        help="MuJoCo 场景 XML 文件路径 (默认: 同目录下的场景文件)"
    )

    # ---- 阻抗控制增益 ----
    parser.add_argument("--kp-pos", type=float, default=5000.0,
                        help="位置刚度 Kp (N/m), 默认 5000（提高以改善位置跟随响应）")
    parser.add_argument("--kd-pos", type=float, default=150.0,
                        help="位置阻尼 Kd (N·s/m), 默认 150")
    parser.add_argument("--kp-rot", type=float, default=300.0,
                        help="姿态刚度 (N·m/rad), 默认 300（远高于位置刚度保证姿态严格跟随）")
    parser.add_argument("--kd-rot", type=float, default=30.0,
                        help="姿态阻尼 (N·m·s/rad), 默认 30")
    parser.add_argument("--null-damping", type=float, default=0.5,
                        help="零空间关节阻尼, 默认 0.5（降低以减少与高姿态刚度的冲突）")

    # ---- 工作空间映射 ----
    parser.add_argument("--pos-scale", type=float, default=0.002,
                        help="位置缩放 (m/mm), 默认 0.002")
    parser.add_argument("--workspace-center", type=float, nargs=3,
                        default=[0.3, 0.0, 1.05],
                        help="工作空间中心 (m), 默认 [0.3, 0, 1.05]")

    # ---- 力反馈参数 ----
    parser.add_argument("--force-scale", type=float, default=0.02,
                        help="力缩放系数 (仿真N → 笔N), 默认 0.02")
    parser.add_argument("--max-force", type=float, default=3.0,
                        help="笔最大输出力 (N), 默认 3.0")
    parser.add_argument("--force-filter-alpha", type=float, default=0.3,
                        help="力反馈低通滤波系数 (0~1), 默认 0.3")
    parser.add_argument("--contact-threshold", type=float, default=0.5,
                        help="接触检测阈值 (N), 超过此值才触发确认力, 默认 0.5")
    parser.add_argument("--contact-confirm-force", type=float, default=0.35,
                        help="（已弃用，由虚拟墙取代）接触确认力固定幅值 (N)")
    parser.add_argument("--wall-stiffness", type=float, default=2000.0,
                        help="虚拟墙弹簧刚度 (N/m, robot坐标系), 默认 2000\n"
                             "  穿入1cm → robot力=20N → 笔感受约1N（force_scale=0.05）\n"
                             "  调大：墙更硬；调小：墙更软")
    parser.add_argument("--rot-scale", type=float, default=1.0,
                        help="姿态旋转倍率, 默认 1.0（1:1 映射）")

    # ---- 延迟缓冲 ----
    parser.add_argument("--cmd-delay", type=float, default=0.0,
                        help="指令延迟 (ms), 默认 0")
    parser.add_argument("--fb-delay", type=float, default=0.0,
                        help="力反馈延迟 (ms), 默认 0")

    # ---- UDP 参数 ----
    parser.add_argument("--udp-recv-port", type=int, default=12345,
                        help="UDP 接收端口, 默认 12345")
    parser.add_argument("--udp-send-port", type=int, default=12346,
                        help="UDP 发送端口, 默认 12346")
    parser.add_argument("--udp-send-ip", default="127.0.0.1",
                        help="UDP 发送目标 IP, 默认 127.0.0.1")

    return parser.parse_args()


# ============================================================================
# 入口
# ============================================================================

if __name__ == "__main__":
    args = parse_args()
    sim = AirbotForceSim(args)
    sim.run()
