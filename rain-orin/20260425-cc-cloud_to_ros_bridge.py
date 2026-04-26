#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cloud_to_ros_bridge — 单向: LiveKit data → ROS2 /cmd_vel + /lift_cmd

拆分自 cloud_teleop_bridge 的上半段. 与 ros_to_cloud_bridge.py 并行跑.

JSON 协议 (LiveKit data channel, reliable):
  {"type":"cmd_vel", "vx":±X, "vy":±X, "wz":±X, "t_sent_ns":<int>}
    → ROS 2 /cmd_vel (Twist) + /cmd_t_sent_ns (UInt64)

  {"type":"lift_cmd", "speed":±X}     # PR-3 新增
    → ROS 2 /lift_cmd (Float32), value = 归一化速度 [-1.0, +1.0]
       (+1 = max 上行, -1 = max 下行, 0 = 停; CangJie chassis_controller
        把它乘 lift_max_current_a 得电流值, 写 DM3519 一拖四 0x200 帧)
"""
import asyncio
import json
import os
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, UInt64

from livekit import api, rtc

LIVEKIT_URL        = os.environ.get("LIVEKIT_URL",        "ws://39.102.113.104:7880")
# 凭证仅从 env 读取, 不再带默认值 (旧默认值已被公开,见 SECURITY.md)
LIVEKIT_API_KEY    = os.environ.get("LIVEKIT_API_KEY")
LIVEKIT_API_SECRET = os.environ.get("LIVEKIT_API_SECRET")
if not (LIVEKIT_API_KEY and LIVEKIT_API_SECRET):
    raise RuntimeError(
        "LIVEKIT_API_KEY and LIVEKIT_API_SECRET env vars required "
        "(see ../.env.example and ../SECURITY.md)"
    )
LIVEKIT_ROOM       = os.environ.get("LIVEKIT_ROOM",       "teleop-room")
LIVEKIT_IDENTITY   = os.environ.get("LK_IDENTITY",        "orin-cloud2ros-bridge")


class Cloud2RosBridge(Node):
    def __init__(self):
        super().__init__('cloud_to_ros_bridge')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tpub = self.create_publisher(UInt64, '/cmd_t_sent_ns', 10)
        # PR-3 新增: lift 命令 publisher (Float32, 归一化 [-1, +1])
        self.lift_pub = self.create_publisher(Float32, '/lift_cmd', 10)
        self.cnt = 0
        self.lift_cnt = 0
        self.get_logger().info('[c2r] /cmd_vel + /lift_cmd publishers ready')


async def livekit_receiver(node: Cloud2RosBridge):
    room = rtc.Room()

    def on_data(pkt: rtc.DataPacket):
        try:
            msg = json.loads(pkt.data.decode('utf-8'))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return
        mtype = msg.get('type')
        if mtype == 'cmd_vel':
            t = Twist()
            t.linear.x  = float(msg.get('vx', 0.0))
            t.linear.y  = float(msg.get('vy', 0.0))
            t.angular.z = float(msg.get('wz', 0.0))
            node.pub.publish(t)
            ts = UInt64(); ts.data = int(msg.get('t_sent_ns', 0)); node.tpub.publish(ts)
            node.cnt += 1
            if node.cnt <= 3 or node.cnt % 100 == 0:
                node.get_logger().info(
                    f'[c2r] ← cmd_vel #{node.cnt} vx={t.linear.x:+.2f} '
                    f'vy={t.linear.y:+.2f} wz={t.angular.z:+.2f}')
        elif mtype == 'lift_cmd':
            # PR-3 新增: ASUS GUI G/H 键 → lift speed normalized [-1, +1]
            speed = float(msg.get('speed', 0.0))
            speed = max(-1.0, min(1.0, speed))   # clamp 防越界
            f = Float32(); f.data = speed
            node.lift_pub.publish(f)
            node.lift_cnt += 1
            if node.lift_cnt <= 3 or node.lift_cnt % 100 == 0:
                node.get_logger().info(
                    f'[c2r] ← lift_cmd #{node.lift_cnt} speed={speed:+.2f}')
        # 其他 type 静默丢弃 (向后兼容: 未来 head_pose 等不会撞这条 bridge)

    room.on('data_received', on_data)
    room.on('participant_connected', lambda p: node.get_logger().info(f'[c2r] join: {p.identity}'))
    room.on('participant_disconnected', lambda p: node.get_logger().info(f'[c2r] leave: {p.identity}'))

    token = api.AccessToken(LIVEKIT_API_KEY, LIVEKIT_API_SECRET)
    token.with_identity(LIVEKIT_IDENTITY)
    token.with_grants(api.VideoGrants(
        room_join=True, room=LIVEKIT_ROOM,
        can_publish=False, can_publish_data=False, can_subscribe=True,
    ))
    jwt = token.to_jwt()
    node.get_logger().info(f'[c2r] connecting {LIVEKIT_URL}...')
    await room.connect(LIVEKIT_URL, jwt)
    node.get_logger().info(f'[c2r] ✓ joined {LIVEKIT_ROOM}')
    while True:
        await asyncio.sleep(1)


def main():
    rclpy.init()
    node = Cloud2RosBridge()

    # rclpy.spin_once 轮询足够 (这个 process 不订阅 ROS2, 只 publish)
    # 但为保持 /rosout 等服务活, 起 spin daemon
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    try:
        asyncio.run(livekit_receiver(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
