#!/usr/bin/env python3
"""
键盘遥操作节点 — 独立的控制层

职责单一: 读取物理键盘 → 发布 ROS2 话题
  - /cmd_vel (Twist)          : 底盘速度指令 (车身系)
  - /mujoco/lift_cmd (Float64) : 升降目标位置
  - /mujoco/head_cmd (Vector3Stamped) : 云台指令 (yaw, pitch, stem)

设计原则:
  此节点是"控制层"的一个可替换实现。
  消费端 (MuJoCo 仿真 / MC02 真实电机) 只订阅上述话题，
  不关心指令来源是键盘、鼠标、摇杆还是运动学规划器。

按键映射:
  WASD = 底盘平移 (车身系 X/Y)
  QE   = 底盘旋转 (yaw)
  GH   = 升降 (lift up/down)
  IJKL = 云台俯仰/偏航
  TY   = 云台伸缩
  R    = 全部归零 (急停)
  ESC  = 退出节点

运行:
  ros2 run teleop_mujoco_sim keyboard_teleop_node
"""

import os
import sys
import math
import ctypes
import ctypes.util

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, Vector3Stamped
from std_msgs.msg import Float64

import numpy as np


# ============== X11 键盘轮询 ==============
# 通过 XQueryKeymap 直接读物理键盘状态 — 按住就是 True，松开就是 False
# 不依赖 pynput / GLFW callback，无需窗口聚焦
class X11KeyPoller:
    """X11 XQueryKeymap 每帧轮询键盘物理状态

    工作原理:
      1. 打开 X11 Display 连接
      2. 把 keysym (逻辑按键) 转为 hardware keycode (物理扫描码)
      3. 每帧调 XQueryKeymap 读取 32 字节位图 → 判断每个键是否被按住
      4. 双缓冲 (cur/prev) 支持边沿检测 (pressed = 刚按下)
    """

    # keysym 表 (X11 标准)
    _KEYSYMS = {
        'w': 0x77, 's': 0x73, 'a': 0x61, 'd': 0x64,  # WASD 底盘平移
        'q': 0x71, 'e': 0x65,                          # QE 底盘旋转
        'g': 0x67, 'h': 0x68,                          # GH 升降
        'i': 0x69, 'k': 0x6b, 'j': 0x6a, 'l': 0x6c,  # IJKL 云台
        't': 0x74, 'y': 0x79,                          # TY 云台伸缩
        'r': 0x72,                                      # R=全部归零
        'esc': 0xff1b,                                  # ESC 退出
    }

    def __init__(self):
        xlib_name = ctypes.util.find_library('X11')
        if not xlib_name:
            raise RuntimeError('libX11 not found')
        self._xlib = ctypes.cdll.LoadLibrary(xlib_name)

        # 函数签名声明
        self._xlib.XOpenDisplay.restype = ctypes.c_void_p
        self._xlib.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self._xlib.XKeysymToKeycode.restype = ctypes.c_ubyte
        self._xlib.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self._xlib.XQueryKeymap.argtypes = [ctypes.c_void_p, ctypes.c_char * 32]
        self._xlib.XCloseDisplay.argtypes = [ctypes.c_void_p]

        display_env = os.environ.get('DISPLAY', ':0').encode()
        self._dpy = self._xlib.XOpenDisplay(display_env)
        if not self._dpy:
            raise RuntimeError(f'Cannot open X11 display {display_env}')

        # keysym → X11 hardware keycode (启动时查一次，不会变)
        self._kc = {}
        for name, keysym in self._KEYSYMS.items():
            self._kc[name] = int(self._xlib.XKeysymToKeycode(self._dpy, keysym))

        # 双缓冲: current (本帧) / prev (上帧) 用于边沿检测
        self._buf = (ctypes.c_char * 32)()
        self._cur = {name: False for name in self._kc}
        self._prev = {name: False for name in self._kc}

    def poll(self):
        """每帧调用: 刷新键盘状态"""
        self._xlib.XQueryKeymap(self._dpy, self._buf)
        raw = bytes(self._buf)
        self._prev = dict(self._cur)
        for name, kc in self._kc.items():
            self._cur[name] = bool(raw[kc // 8] & (1 << (kc % 8)))

    def held(self, name):
        """键是否被按住 (本帧)"""
        return self._cur.get(name, False)

    def pressed(self, name):
        """键是否刚按下 (上升沿: 上帧 False → 本帧 True)"""
        return self._cur.get(name, False) and not self._prev.get(name, False)

    def close(self):
        if self._dpy:
            self._xlib.XCloseDisplay(self._dpy)
            self._dpy = None


# ============== 控制参数 ==============
# 底盘速度极限 (车身系)
CHASSIS_SPEED = 0.6       # m/s, WASD 开关量映射的速度
CHASSIS_OMEGA_SPEED = 1.5 # rad/s, QE 旋转速度

# 斜率限幅 (平滑加减速, 防止阶跃冲击)
CHASSIS_VX_SLEW = 3.0     # m/s², X 方向加速度限制
CHASSIS_VY_SLEW = 3.0     # m/s², Y 方向加速度限制
CHASSIS_OMEGA_SLEW = 6.0  # rad/s², 角速度加速度限制

# 升降参数
LIFT_MIN, LIFT_MAX, LIFT_INIT = 0.0, 0.8, 0.0
LIFT_RATE = 0.5           # m/s, 升降速度

# 云台参数
HEAD_YAW_MIN, HEAD_YAW_MAX = -1.5708, 1.5708
HEAD_PITCH_MIN, HEAD_PITCH_MAX = -0.7854, 0.7854
HEAD_RATE = 2.5           # rad/s
HEAD_STEM_MIN, HEAD_STEM_MAX = -0.05, 0.40
HEAD_STEM_RATE = 0.30     # m/s

# 控制回路频率
CONTROL_HZ = 50           # 50Hz, 与仿真发布频率匹配


def slew(current, target, slew_rate, dt):
    """斜率限幅: 限制每帧最大变化量, 实现平滑加减速"""
    step = slew_rate * dt
    d = target - current
    return current + (math.copysign(step, d) if abs(d) > step else d)


class KeyboardTeleopNode(Node):
    """键盘遥操作节点 — 读取物理键盘, 发布控制话题

    数据流:
      X11 物理键盘 → 本节点 → /cmd_vel (底盘)
                             → /mujoco/lift_cmd (升降)
                             → /mujoco/head_cmd (云台)
                             ↓
                      消费端订阅 (仿真 / 真实电机 / 其他)
    """

    def __init__(self):
        super().__init__('keyboard_teleop_node')

        # ---- QoS ----
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)

        # ---- 发布者 ----
        # /cmd_vel: 底盘速度指令 (车身系 Twist)
        #   linear.x = 前进/后退, linear.y = 左移/右移, angular.z = 旋转
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', qos)

        # /mujoco/lift_cmd: 升降目标位置 (m)
        self.pub_lift = self.create_publisher(Float64, '/mujoco/lift_cmd', qos)

        # /mujoco/head_cmd: 云台指令 (x=yaw, y=pitch, z=stem)
        self.pub_head = self.create_publisher(
            Vector3Stamped, '/mujoco/head_cmd', qos)

        # ---- 控制状态 (带斜率限幅的平滑值) ----
        self.smoothed_vx = 0.0
        self.smoothed_vy = 0.0
        self.smoothed_omega = 0.0
        self.target_lift = LIFT_INIT
        self.target_head_yaw = 0.0
        self.target_head_pitch = 0.0
        self.target_head_stem = 0.0

        # ---- X11 键盘轮询 ----
        try:
            self.keys = X11KeyPoller()
            self.get_logger().info('键盘控制节点已启动 (X11 全局按键检测)')
            self.get_logger().info('  按住: WASD=平移 QE=旋转 GH=升降 IJKL=云台 TY=伸缩')
            self.get_logger().info('  单按: R=全部归零 ESC=退出')
            self.get_logger().info('发布话题:')
            self.get_logger().info('  /cmd_vel (Twist) — 底盘速度')
            self.get_logger().info('  /mujoco/lift_cmd (Float64) — 升降位置')
            self.get_logger().info('  /mujoco/head_cmd (Vector3Stamped) — 云台')
        except RuntimeError as e:
            self.get_logger().error(f'X11 键盘初始化失败: {e}')
            self.get_logger().error('请确认 DISPLAY 环境变量已设置 (如 DISPLAY=:0)')
            self.keys = None
            return

        # ---- 定时器: 固定频率控制回路 ----
        self._dt = 1.0 / CONTROL_HZ
        self._timer = self.create_timer(self._dt, self._control_loop)
        self._exit_request = False

    def _control_loop(self):
        """固定频率控制回路: 每帧读键盘 → 平滑 → 发布话题"""
        if not self.keys or self._exit_request:
            return

        k = self.keys
        dt = self._dt

        # ---- 1. 轮询键盘状态 ----
        k.poll()

        # ---- 2. ESC 退出 ----
        if k.pressed('esc'):
            self.get_logger().info('ESC 按下, 退出键盘控制节点')
            self._exit_request = True
            # 发一帧零速度确保停车
            self._publish_zero()
            raise SystemExit(0)

        # ---- 3. R 全部归零 (急停) ----
        if k.pressed('r'):
            self.smoothed_vx = 0.0
            self.smoothed_vy = 0.0
            self.smoothed_omega = 0.0
            self.target_lift = LIFT_INIT
            self.target_head_yaw = 0.0
            self.target_head_pitch = 0.0
            self.target_head_stem = 0.0
            self.get_logger().info('RESET: 全部归零')
            self._publish_zero()
            return

        # ---- 4. 底盘: 键盘开关量 → 期望速度 → 斜率限幅 ----
        # WASD: 车身系速度. W=车头前进(+X), A=左移(+Y), Q=左转(+ω)
        desired_vx = ((1 if k.held('w') else 0) -
                      (1 if k.held('s') else 0)) * CHASSIS_SPEED
        desired_vy = ((1 if k.held('a') else 0) -
                      (1 if k.held('d') else 0)) * CHASSIS_SPEED
        desired_omega = ((1 if k.held('q') else 0) -
                         (1 if k.held('e') else 0)) * CHASSIS_OMEGA_SPEED

        # 斜率限幅: 防止速度阶跃, 实现平滑加减速
        self.smoothed_vx = slew(self.smoothed_vx, desired_vx,
                                CHASSIS_VX_SLEW, dt)
        self.smoothed_vy = slew(self.smoothed_vy, desired_vy,
                                CHASSIS_VY_SLEW, dt)
        self.smoothed_omega = slew(self.smoothed_omega, desired_omega,
                                   CHASSIS_OMEGA_SLEW, dt)

        # 死区: 消除微小残留速度
        if abs(self.smoothed_vx) < 0.005:
            self.smoothed_vx = 0.0
        if abs(self.smoothed_vy) < 0.005:
            self.smoothed_vy = 0.0
        if abs(self.smoothed_omega) < 0.005:
            self.smoothed_omega = 0.0

        # 发布 /cmd_vel
        twist = Twist()
        twist.linear.x = self.smoothed_vx
        twist.linear.y = self.smoothed_vy
        twist.angular.z = self.smoothed_omega
        self.pub_cmd_vel.publish(twist)

        # ---- 5. 升降: G=升 H=降 ----
        lift_dir = (1 if k.held('g') else 0) - (1 if k.held('h') else 0)
        if lift_dir != 0:
            self.target_lift = np.clip(
                self.target_lift + lift_dir * LIFT_RATE * dt,
                LIFT_MIN, LIFT_MAX)
        lift_msg = Float64()
        lift_msg.data = float(self.target_lift)
        self.pub_lift.publish(lift_msg)

        # ---- 6. 云台: IJKL=偏航/俯仰, TY=伸缩 ----
        step = HEAD_RATE * dt
        pitch_dir = (1 if k.held('k') else 0) - (1 if k.held('i') else 0)
        if pitch_dir != 0:
            self.target_head_pitch = np.clip(
                self.target_head_pitch + pitch_dir * step,
                HEAD_PITCH_MIN, HEAD_PITCH_MAX)

        yaw_dir = (1 if k.held('j') else 0) - (1 if k.held('l') else 0)
        if yaw_dir != 0:
            self.target_head_yaw = np.clip(
                self.target_head_yaw + yaw_dir * step,
                HEAD_YAW_MIN, HEAD_YAW_MAX)

        stem_dir = (1 if k.held('t') else 0) - (1 if k.held('y') else 0)
        if stem_dir != 0:
            self.target_head_stem = np.clip(
                self.target_head_stem + stem_dir * HEAD_STEM_RATE * dt,
                HEAD_STEM_MIN, HEAD_STEM_MAX)

        head_msg = Vector3Stamped()
        head_msg.header.stamp = self.get_clock().now().to_msg()
        head_msg.vector.x = float(self.target_head_yaw)
        head_msg.vector.y = float(self.target_head_pitch)
        head_msg.vector.z = float(self.target_head_stem)
        self.pub_head.publish(head_msg)

    def _publish_zero(self):
        """发布全零指令 (急停/退出时调用)"""
        self.pub_cmd_vel.publish(Twist())
        lift_msg = Float64()
        lift_msg.data = float(self.target_lift)
        self.pub_lift.publish(lift_msg)
        head_msg = Vector3Stamped()
        head_msg.header.stamp = self.get_clock().now().to_msg()
        head_msg.vector.x = float(self.target_head_yaw)
        head_msg.vector.y = float(self.target_head_pitch)
        head_msg.vector.z = float(self.target_head_stem)
        self.pub_head.publish(head_msg)

    def destroy_node(self):
        """清理 X11 资源"""
        if self.keys:
            self.keys.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
