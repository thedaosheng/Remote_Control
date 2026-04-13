#!/usr/bin/env python3
"""键盘遥操作节点 — 独立控制层，读键盘 → 发布 /cmd_vel 等话题"""
import os, sys, math, ctypes, ctypes.util
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, Vector3Stamped
from std_msgs.msg import Float64

class X11KeyPoller:
    _KEYSYMS = {'w':0x77,'s':0x73,'a':0x61,'d':0x64,'q':0x71,'e':0x65,
                'g':0x67,'h':0x68,'i':0x69,'k':0x6b,'j':0x6a,'l':0x6c,
                't':0x74,'y':0x79,'r':0x72,'esc':0xff1b}
    def __init__(self):
        xlib_name = ctypes.util.find_library('X11')
        if not xlib_name: raise RuntimeError('libX11 not found')
        self._xlib = ctypes.cdll.LoadLibrary(xlib_name)
        self._xlib.XOpenDisplay.restype = ctypes.c_void_p
        self._xlib.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self._xlib.XKeysymToKeycode.restype = ctypes.c_ubyte
        self._xlib.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self._xlib.XQueryKeymap.argtypes = [ctypes.c_void_p, ctypes.c_char * 32]
        self._xlib.XCloseDisplay.argtypes = [ctypes.c_void_p]
        display_env = os.environ.get('DISPLAY', ':0').encode()
        self._dpy = self._xlib.XOpenDisplay(display_env)
        if not self._dpy: raise RuntimeError(f'Cannot open X11 display')
        self._kc = {n: int(self._xlib.XKeysymToKeycode(self._dpy, ks)) for n, ks in self._KEYSYMS.items()}
        self._buf = (ctypes.c_char * 32)()
        self._cur = {n: False for n in self._kc}
        self._prev = dict(self._cur)
    def poll(self):
        self._xlib.XQueryKeymap(self._dpy, self._buf)
        raw = bytes(self._buf)
        self._prev = dict(self._cur)
        for n, kc in self._kc.items():
            self._cur[n] = bool(raw[kc // 8] & (1 << (kc % 8)))
    def held(self, n): return self._cur.get(n, False)
    def pressed(self, n): return self._cur.get(n, False) and not self._prev.get(n, False)
    def close(self):
        if self._dpy: self._xlib.XCloseDisplay(self._dpy); self._dpy = None

CHASSIS_SPEED, CHASSIS_OMEGA_SPEED = 0.6, 1.5
CHASSIS_VX_SLEW, CHASSIS_VY_SLEW, CHASSIS_OMEGA_SLEW = 3.0, 3.0, 6.0
LIFT_MIN, LIFT_MAX, LIFT_INIT, LIFT_RATE = 0.0, 0.8, 0.0, 0.5
HEAD_YAW_MIN, HEAD_YAW_MAX = -1.5708, 1.5708
HEAD_PITCH_MIN, HEAD_PITCH_MAX = -0.7854, 0.7854
HEAD_RATE, HEAD_STEM_MIN, HEAD_STEM_MAX, HEAD_STEM_RATE = 2.5, -0.05, 0.40, 0.30
CONTROL_HZ = 50

def slew(cur, tgt, rate, dt):
    step = rate * dt; d = tgt - cur
    return cur + (math.copysign(step, d) if abs(d) > step else d)

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', qos)
        self.pub_lift = self.create_publisher(Float64, '/mujoco/lift_cmd', qos)
        self.pub_head = self.create_publisher(Vector3Stamped, '/mujoco/head_cmd', qos)
        self.vx = self.vy = self.omega = 0.0
        self.lift = LIFT_INIT; self.yaw = self.pitch = self.stem = 0.0
        try:
            self.keys = X11KeyPoller()
            self.get_logger().info('键盘控制节点已启动 (WASD=平移 QE=旋转 GH=升降 IJKL=云台 R=归零 ESC=退出)')
        except RuntimeError as e:
            self.get_logger().error(f'X11 键盘初始化失败: {e}'); self.keys = None; return
        self._dt = 1.0 / CONTROL_HZ
        self._timer = self.create_timer(self._dt, self._loop)
    def _loop(self):
        if not self.keys: return
        k, dt = self.keys, self._dt; k.poll()
        if k.pressed('esc'): raise SystemExit(0)
        if k.pressed('r'):
            self.vx=self.vy=self.omega=0.0; self.lift=LIFT_INIT; self.yaw=self.pitch=self.stem=0.0
            self.pub_cmd_vel.publish(Twist()); return
        dvx = ((1 if k.held('w') else 0)-(1 if k.held('s') else 0))*CHASSIS_SPEED
        dvy = ((1 if k.held('a') else 0)-(1 if k.held('d') else 0))*CHASSIS_SPEED
        dom = ((1 if k.held('q') else 0)-(1 if k.held('e') else 0))*CHASSIS_OMEGA_SPEED
        self.vx=slew(self.vx,dvx,CHASSIS_VX_SLEW,dt); self.vy=slew(self.vy,dvy,CHASSIS_VY_SLEW,dt); self.omega=slew(self.omega,dom,CHASSIS_OMEGA_SLEW,dt)
        for attr in ('vx','vy','omega'):
            if abs(getattr(self,attr))<0.005: setattr(self,attr,0.0)
        t=Twist(); t.linear.x=self.vx; t.linear.y=self.vy; t.angular.z=self.omega; self.pub_cmd_vel.publish(t)
        ld=(1 if k.held('g') else 0)-(1 if k.held('h') else 0)
        if ld: self.lift=np.clip(self.lift+ld*LIFT_RATE*dt,LIFT_MIN,LIFT_MAX)
        m=Float64(); m.data=float(self.lift); self.pub_lift.publish(m)
        s=HEAD_RATE*dt
        pd=(1 if k.held('k') else 0)-(1 if k.held('i') else 0)
        if pd: self.pitch=np.clip(self.pitch+pd*s,HEAD_PITCH_MIN,HEAD_PITCH_MAX)
        yd=(1 if k.held('j') else 0)-(1 if k.held('l') else 0)
        if yd: self.yaw=np.clip(self.yaw+yd*s,HEAD_YAW_MIN,HEAD_YAW_MAX)
        sd=(1 if k.held('t') else 0)-(1 if k.held('y') else 0)
        if sd: self.stem=np.clip(self.stem+sd*HEAD_STEM_RATE*dt,HEAD_STEM_MIN,HEAD_STEM_MAX)
        h=Vector3Stamped(); h.header.stamp=self.get_clock().now().to_msg()
        h.vector.x=float(self.yaw); h.vector.y=float(self.pitch); h.vector.z=float(self.stem)
        self.pub_head.publish(h)
    def destroy_node(self):
        if self.keys: self.keys.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args); node=KeyboardTeleopNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit): pass
    finally: node.destroy_node(); rclpy.try_shutdown()

if __name__=='__main__': main()
