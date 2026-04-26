#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ros_to_cloud_bridge — 单向: ROS2 /chassis_state → LiveKit data

拆分自 cloud_teleop_bridge 的下半段。同进程里 rclpy subscription + livekit
SDK 会相互阻塞, 只能拆开跑两个进程.

配套另一个进程 cloud_to_ros_bridge.py (LiveKit → /cmd_vel).
"""
import asyncio
import json
import os
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from livekit import api, rtc

LIVEKIT_URL        = os.environ.get("LIVEKIT_URL",        "ws://39.102.113.104:7880")
# 凭证仅从 env 读取, 不再带默认值 (旧默认值已被公开,见 ../SECURITY.md)
LIVEKIT_API_KEY    = os.environ.get("LIVEKIT_API_KEY")
LIVEKIT_API_SECRET = os.environ.get("LIVEKIT_API_SECRET")
if not (LIVEKIT_API_KEY and LIVEKIT_API_SECRET):
    raise RuntimeError(
        "LIVEKIT_API_KEY and LIVEKIT_API_SECRET env vars required "
        "(see ../.env.example and ../SECURITY.md)"
    )
LIVEKIT_ROOM       = os.environ.get("LIVEKIT_ROOM",       "teleop-room")
LIVEKIT_IDENTITY   = os.environ.get("LK_IDENTITY",        "cangjie-ros2cloud-bridge")


class Ros2CloudBridge(Node):
    def __init__(self, q: queue.Queue):
        super().__init__('ros_to_cloud_bridge')
        self.q = q
        self.cnt = 0
        self.sub = self.create_subscription(String, '/chassis_state', self._on, 10)
        self.get_logger().info('[r2c] subscribed /chassis_state')

    def _on(self, msg: String):
        self.cnt += 1
        try:
            self.q.put_nowait(msg.data)
        except queue.Full:
            try: self.q.get_nowait()
            except queue.Empty: pass
            try: self.q.put_nowait(msg.data)
            except queue.Full: pass


async def livekit_sender(q: queue.Queue, log):
    room = rtc.Room()
    token = api.AccessToken(LIVEKIT_API_KEY, LIVEKIT_API_SECRET)
    token.with_identity(LIVEKIT_IDENTITY)
    token.with_grants(api.VideoGrants(
        room_join=True, room=LIVEKIT_ROOM,
        can_publish=False, can_publish_data=True, can_subscribe=True,
    ))
    jwt = token.to_jwt()
    log.info(f'[r2c] connecting {LIVEKIT_URL}...')
    await room.connect(LIVEKIT_URL, jwt)
    log.info(f'[r2c] ✓ joined {LIVEKIT_ROOM}')

    sent = 0
    while True:
        try:
            data_str = q.get(timeout=0.5)
        except queue.Empty:
            continue
        try:
            payload = json.loads(data_str)
        except json.JSONDecodeError:
            continue
        payload['t_orin_fwd_ns'] = time.time_ns()
        try:
            await room.local_participant.publish_data(
                json.dumps(payload).encode('utf-8'), reliable=True)
            sent += 1
            if sent <= 3 or sent % 100 == 0:
                log.info(f'[r2c] → chassis_state #{sent}')
        except Exception as e:
            log.warn(f'[r2c] publish_data err: {e}')


def main():
    rclpy.init()
    q = queue.Queue(maxsize=100)
    node = Ros2CloudBridge(q)

    def rclpy_thread():
        rclpy.spin(node)
    threading.Thread(target=rclpy_thread, daemon=True).start()

    try:
        asyncio.run(livekit_sender(q, node.get_logger()))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
