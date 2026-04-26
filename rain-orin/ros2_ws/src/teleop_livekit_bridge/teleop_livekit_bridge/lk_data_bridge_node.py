#!/usr/bin/env python3
"""
==============================================================
  lk_data_bridge_node — LiveKit data-only ROS2 桥接节点
==============================================================

2026-04-22 新增 (by Claude Code, YZL 要求硬编红利):
  跟原 `livekit_bridge_node` 区别:
    - 原 node 跑 sender_core.main() 推视频 + 接 data (软编, CPU 170%)
    - 本 node 只接 LiveKit data_channel, 不推视频
    - 视频由独立的 systemd user service (zed-hw-gst + zed-hw-lk) 硬编推

职责:
  1. 连 LiveKit room (identity=zed-mini-pose-recv)
  2. 订阅 data_channel, 解析 VP head pose
  3. publish:
       /vp/head_pose       teleop_msgs/HeadPose
       /vp/head_pose_rpy   geometry_msgs/Vector3Stamped

线程模型 (沿袭 livekit_bridge_node 做法):
  - 主线程: asyncio event loop (LiveKit Python SDK 要求)
  - spin 线程: rclpy.spin() 独立线程, publisher.publish 线程安全

启动:
  ros2 run teleop_livekit_bridge lk_data_bridge

环境变量:
  LIVEKIT_URL     默认 ws://39.102.113.104:7880
  LK_IDENTITY     默认 zed-mini-pose-recv

作者: Claude Code (2026-04-22)
"""
import asyncio
import json
import math
import os
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_srvs.srv import Trigger

from teleop_msgs.msg import HeadPose
from livekit import rtc, api


# Map VP data_channel {"type":"cmd","cmd":"XXX","arm":"left|right"} → ROS2 service path
#   arm 字段可选; 缺省 / cmd 含 "_all" → 走 /arms/ bus-wide
_CMD_MAP = {
    "emergency_stop":     "/arm/{arm}/emergency_stop",
    "emergency_stop_all": "/arms/emergency_stop_all",
    "freeze":             "/arm/{arm}/freeze",
    "clear_errors":       "/arm/{arm}/clear_errors",
    "return_zero":        "/arm/{arm}/return_zero",
    "re_enable":          "/arm/{arm}/re_enable",
    "status":             "/arm/{arm}/status",
    "status_all":         "/arms/status",
}


# ==================== 配置 ====================
LIVEKIT_URL = os.environ.get("LIVEKIT_URL", "ws://39.102.113.104:7880")
# 凭证仅从 env 读取, 不再带默认值 (旧默认值已被公开,见 SECURITY.md)
LIVEKIT_API_KEY = os.environ.get("LIVEKIT_API_KEY")
LIVEKIT_API_SECRET = os.environ.get("LIVEKIT_API_SECRET")
if not (LIVEKIT_API_KEY and LIVEKIT_API_SECRET):
    raise RuntimeError(
        "LIVEKIT_API_KEY and LIVEKIT_API_SECRET env vars required "
        "(see top-level .env.example and SECURITY.md)"
    )
LIVEKIT_ROOM = os.environ.get("LIVEKIT_ROOM", "teleop-room")
LIVEKIT_IDENTITY = os.environ.get("LK_IDENTITY", "zed-mini-pose-recv")


# ==================== 四元数 → 欧拉角 ====================
def quat_to_euler(qx, qy, qz, qw):
    """ZYX 内旋欧拉角 (航空惯例), 返回 (pitch, roll, yaw) 弧度, 万向锁已钳位."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return pitch, roll, yaw


class LkDataBridge(Node):
    """LiveKit data → ROS2 topic 桥接 (仅 pose, 不碰视频)."""

    def __init__(self):
        super().__init__('lk_data_bridge')

        # ROS 参数 (launch 里可以 override)
        self.declare_parameter('pose_topic', '/vp/head_pose')
        self.declare_parameter('pose_rpy_topic', '/vp/head_pose_rpy')
        pose_topic = self.get_parameter('pose_topic').value
        rpy_topic = self.get_parameter('pose_rpy_topic').value

        # Publisher (QoS depth=10, 30Hz pose 足够)
        self.pose_pub = self.create_publisher(HeadPose, pose_topic, 10)
        self.rpy_pub = self.create_publisher(Vector3Stamped, rpy_topic, 10)

        self.room = None
        self.count = 0
        self.first_pose_time = None

        self.get_logger().info(
            f'[lk_data_bridge] ready. Publishing to {pose_topic} + {rpy_topic}'
        )
        self.get_logger().info(
            f'[lk_data_bridge] LiveKit: {LIVEKIT_URL}, identity={LIVEKIT_IDENTITY}'
        )

    # ---------- LiveKit data 回调 ----------
    def _on_data(self, data_packet: rtc.DataPacket):
        """VP 发 JSON → dispatch by type."""
        try:
            msg = json.loads(data_packet.data.decode('utf-8'))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return

        msg_type = msg.get('type', '')
        if msg_type == 'pose':
            self._handle_pose(msg)
        elif msg_type == 'cmd':
            self._handle_command(msg)

    # ---------- Safety command pass-through (VP → ROS2 service) ----------
    def _handle_command(self, msg):
        """VP publish_data {"type":"cmd","cmd":"emergency_stop_all"} → /arms/emergency_stop_all"""
        cmd = msg.get('cmd', '')
        arm = msg.get('arm', 'left')  # 默认 left; 若 cmd 是 _all 类会走 bus-wide
        tmpl = _CMD_MAP.get(cmd)
        if not tmpl:
            self.get_logger().warn(f'[cmd] unknown: {cmd}')
            return
        srv_path = tmpl.format(arm=arm)

        # lazy-create client, cached by path
        if not hasattr(self, '_srv_clients'):
            self._srv_clients = {}
        client = self._srv_clients.get(srv_path)
        if client is None:
            client = self.create_client(Trigger, srv_path)
            self._srv_clients[srv_path] = client

        if not client.wait_for_service(timeout_sec=0.3):
            self.get_logger().warn(f'[cmd] service {srv_path} unavailable (teleop_safety node running?)')
            return

        self.get_logger().info(f'[cmd] dispatch {cmd}(arm={arm}) → {srv_path}')
        future = client.call_async(Trigger.Request())
        # fire-and-forget: rclpy executor thread 会 completes future; 我们不 await
        # (data_received 在 asyncio loop, 不能 block)

    def _handle_pose(self, msg):
        """VP 发 pose JSON → 解析 → publish 到 ROS topic."""
        pos = msg.get('p', [0, 0, 0])
        quat = msg.get('q', [0, 0, 0, 1])
        ts = msg.get('t', 0)

        # 数学欧拉 → 物理含义 (沿用 sender_core 的映射约定)
        #   physical pitch (点头)   ← math.roll  (绕 X)
        #   physical yaw   (转头)   ← math.pitch (绕 Y)
        #   physical roll  (侧倾)   ← math.yaw   (绕 Z)
        pitch, roll, yaw = quat_to_euler(quat[0], quat[1], quat[2], quat[3])
        phys_pitch_deg = math.degrees(roll)
        phys_yaw_deg = math.degrees(pitch)
        phys_roll_deg = math.degrees(yaw)

        stamp = self.get_clock().now().to_msg()

        # 1. 完整 HeadPose
        hp = HeadPose()
        hp.header.stamp = stamp
        hp.header.frame_id = 'vp_head_initial'
        hp.pitch_deg = float(phys_pitch_deg)
        hp.yaw_deg = float(phys_yaw_deg)
        hp.roll_deg = float(phys_roll_deg)
        hp.pos_x = float(pos[0])
        hp.pos_y = float(pos[1])
        hp.pos_z = float(pos[2])
        hp.quat_x = float(quat[0])
        hp.quat_y = float(quat[1])
        hp.quat_z = float(quat[2])
        hp.quat_w = float(quat[3])
        self.pose_pub.publish(hp)

        # 2. 简化版 Vector3Stamped (rqt_plot 友好)
        rpy = Vector3Stamped()
        rpy.header.stamp = stamp
        rpy.header.frame_id = 'vp_head_initial'
        rpy.vector.x = float(phys_roll_deg)
        rpy.vector.y = float(phys_pitch_deg)
        rpy.vector.z = float(phys_yaw_deg)
        self.rpy_pub.publish(rpy)

        self.count += 1
        if self.count == 1:
            self.get_logger().info('[lk_data_bridge] ✓ 首个 pose 到达')
        if self.count % 150 == 0:
            self.get_logger().info(
                f'[lk_data_bridge] #{self.count} '
                f'P={phys_pitch_deg:+.1f}° Y={phys_yaw_deg:+.1f}° R={phys_roll_deg:+.1f}°'
            )

    # ---------- 连 LiveKit room ----------
    async def connect_livekit(self):
        self.room = rtc.Room()
        self.room.on('data_received', self._on_data)

        def _on_joined(p: rtc.RemoteParticipant):
            self.get_logger().info(f'[lk_data_bridge] 参与者加入: {p.identity}')

        def _on_left(p: rtc.RemoteParticipant):
            self.get_logger().info(f'[lk_data_bridge] 参与者离开: {p.identity}')

        self.room.on('participant_connected', _on_joined)
        self.room.on('participant_disconnected', _on_left)

        token = api.AccessToken(LIVEKIT_API_KEY, LIVEKIT_API_SECRET)
        token.with_identity(LIVEKIT_IDENTITY)
        token.with_grants(api.VideoGrants(
            room_join=True,
            room=LIVEKIT_ROOM,
            can_publish=False,        # 本节点不推视频 (视频走 lk CLI)
            can_publish_data=True,    # 允许 debug 发 data
            can_subscribe=True,
        ))
        jwt = token.to_jwt()

        self.get_logger().info(f'[lk_data_bridge] 连接 {LIVEKIT_URL}...')
        await self.room.connect(LIVEKIT_URL, jwt)
        self.get_logger().info(f'[lk_data_bridge] ✓ 已加入 room={self.room.name}')

    async def run(self):
        await self.connect_livekit()
        # 保持连接, 由 data callback 处理 pose
        while rclpy.ok():
            await asyncio.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    node = LkDataBridge()

    # ROS2 spin 跑独立线程 (publisher.publish 线程安全)
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    # LiveKit asyncio 主循环
    try:
        asyncio.run(node.run())
    except KeyboardInterrupt:
        node.get_logger().info('[lk_data_bridge] KeyboardInterrupt, 退出')
    except Exception as e:
        node.get_logger().error(f'[lk_data_bridge] 异常: {e}')
    finally:
        if node.room is not None:
            try:
                loop = asyncio.new_event_loop()
                loop.run_until_complete(node.room.disconnect())
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
