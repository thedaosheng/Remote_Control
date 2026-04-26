#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
====================================================================
  cloud_teleop_bridge_node — LiveKit ↔ ROS2 双向桥 (Orin 上跑)
====================================================================

用途:
  打通 "182 → 阿里云 LiveKit → Orin → ROS2 → CangJie" 链路.

  下行 (cmd_vel): 182 publish_data → Aliyun → Orin 收 → 转 /cmd_vel ROS2 topic
                  → CangJie 订阅 (跨机 DDS LAN auto-discover) → 舵轮 IK 解算
  上行 (ack):    CangJie publish /chassis_state (String JSON) → Orin 订阅
                  → 带上 Orin 缓存的 t_sent → publish_data 回 LiveKit
                  → 182 收到, 展示 4 轮解算 + 往返延时

注:
  - 与现有 lk_data_bridge_node (处理 VP head_pose + safety cmd) 完全独立
  - 用不同 identity "orin-cloud-teleop-bridge" 避免跟 "zed-mini-pose-recv" 冲突
  - 同一个 room "teleop-room", 所以 182 和 VP 都在里面, 但 type 字段区分
"""
import asyncio
import json
import os
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from livekit import api, rtc


LIVEKIT_URL        = os.environ.get("LIVEKIT_URL",        "ws://39.102.113.104:7880")
# 凭证仅从 env 读取, 不再带默认值 (旧默认值已被公开,见 SECURITY.md)
# 注意: 本节点已被 cloud_to_ros_bridge.py 取代 (livekit + rclpy 同进程冲突, 见 SECURITY.md/feedback)
LIVEKIT_API_KEY    = os.environ.get("LIVEKIT_API_KEY")
LIVEKIT_API_SECRET = os.environ.get("LIVEKIT_API_SECRET")
if not (LIVEKIT_API_KEY and LIVEKIT_API_SECRET):
    raise RuntimeError(
        "LIVEKIT_API_KEY and LIVEKIT_API_SECRET env vars required "
        "(see ../.env.example and ../SECURITY.md)"
    )
LIVEKIT_ROOM       = os.environ.get("LIVEKIT_ROOM",       "teleop-room")
LIVEKIT_IDENTITY   = os.environ.get("LK_IDENTITY",        "orin-cloud-teleop-bridge")


class CloudTeleopBridge(Node):
    def __init__(self):
        super().__init__('cloud_teleop_bridge')

        # ROS2 出口: /cmd_vel (CangJie 订阅它)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ROS2 入口: /chassis_state (CangJie 每次 IK 之后发布)
        self.state_sub = self.create_subscription(
            String, '/chassis_state', self._on_chassis_state, 10)

        self.room = None
        self._last_cmd_t_sent_ns = 0   # 缓存 182 发来的 t_sent, 回包时补上供 RTT 计算
        self._last_cmd_seq = 0

        # 跨线程 queue: rclpy callback (任意 thread) → asyncio consumer task
        # 容量 100, 满了丢最旧的 (不堵塞 ROS2 callback)
        self._state_queue = queue.Queue(maxsize=100)

        self._cnt_cmd_in = 0
        self._cnt_state_out = 0

        self.get_logger().info(
            f'[cloud_teleop] bridge ready  '
            f'LiveKit={LIVEKIT_URL} identity={LIVEKIT_IDENTITY} room={LIVEKIT_ROOM}')

    # ---------- LiveKit data → ROS2 /cmd_vel ----------
    def _on_lk_data(self, packet: rtc.DataPacket):
        try:
            msg = json.loads(packet.data.decode('utf-8'))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return
        if msg.get('type') != 'cmd_vel':
            return  # 放过 VP pose 等其它 type

        # 缓存 t_sent 和 seq, 回包时带上
        self._last_cmd_t_sent_ns = int(msg.get('t_sent_ns', 0))
        self._last_cmd_seq = int(msg.get('seq', 0))

        t = Twist()
        t.linear.x  = float(msg.get('vx', 0.0))
        t.linear.y  = float(msg.get('vy', 0.0))
        t.angular.z = float(msg.get('wz', 0.0))
        self.cmd_vel_pub.publish(t)

        self._cnt_cmd_in += 1
        if self._cnt_cmd_in <= 3 or self._cnt_cmd_in % 100 == 0:
            self.get_logger().info(
                f'[cloud_teleop] ← cmd_vel #{self._cnt_cmd_in} '
                f'vx={t.linear.x:+.2f} vy={t.linear.y:+.2f} wz={t.angular.z:+.2f} '
                f'seq={self._last_cmd_seq}')

    # ---------- ROS2 /chassis_state → queue (仅 enqueue, 不碰 LiveKit) ----------
    def _on_chassis_state(self, msg: String):
        """rclpy callback 仅负责入队, 绝不跨线程调 asyncio — 让 asyncio 自己 drain"""
        try:
            self._state_queue.put_nowait(msg.data)
        except queue.Full:
            # 丢最旧一条, 再塞新的
            try:
                self._state_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._state_queue.put_nowait(msg.data)
            except queue.Full:
                pass

    # ---------- asyncio task: drain queue + publish to LiveKit ----------
    async def _forward_states(self):
        """独立 asyncio task. rclpy.callback 入队, 这里出队 + await publish."""
        loop = asyncio.get_event_loop()
        while rclpy.ok():
            # 非阻塞出队; 空就让出
            try:
                data_str = self._state_queue.get_nowait()
            except queue.Empty:
                await asyncio.sleep(0.01)
                continue

            if self.room is None:
                continue
            try:
                payload = json.loads(data_str)
            except json.JSONDecodeError:
                continue

            payload['t_sent_ns'] = self._last_cmd_t_sent_ns
            payload['seq_echo'] = self._last_cmd_seq
            payload['t_orin_fwd_ns'] = time.time_ns()
            out = json.dumps(payload).encode('utf-8')
            try:
                await self.room.local_participant.publish_data(out, reliable=True)
                self._cnt_state_out += 1
                if self._cnt_state_out <= 3 or self._cnt_state_out % 100 == 0:
                    self.get_logger().info(
                        f'[cloud_teleop] → chassis_state #{self._cnt_state_out} '
                        f'seq_echo={self._last_cmd_seq}')
            except Exception as e:
                self.get_logger().warn(f'[cloud_teleop] publish_data err: {e}')

    # ---------- LiveKit 连接 ----------
    async def connect(self):
        self.room = rtc.Room()
        self.room.on('data_received', self._on_lk_data)

        def _joined(p: rtc.RemoteParticipant):
            self.get_logger().info(f'[cloud_teleop] join: {p.identity}')
        def _left(p: rtc.RemoteParticipant):
            self.get_logger().info(f'[cloud_teleop] leave: {p.identity}')
        self.room.on('participant_connected', _joined)
        self.room.on('participant_disconnected', _left)

        token = api.AccessToken(LIVEKIT_API_KEY, LIVEKIT_API_SECRET)
        token.with_identity(LIVEKIT_IDENTITY)
        token.with_grants(api.VideoGrants(
            room_join=True, room=LIVEKIT_ROOM,
            can_publish=False, can_publish_data=True, can_subscribe=True,
        ))
        jwt = token.to_jwt()

        self.get_logger().info(f'[cloud_teleop] connecting {LIVEKIT_URL}...')
        await self.room.connect(LIVEKIT_URL, jwt)
        self.get_logger().info(f'[cloud_teleop] ✓ joined room={self.room.name}')

    async def run(self):
        await self.connect()
        drain_task = asyncio.create_task(self._forward_states())
        try:
            # 同线程 轮询 rclpy + 让 asyncio: 避免两个独立 loop 线程间的谜之阻塞
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0)
                await asyncio.sleep(0.002)  # 500 Hz 轮询, 对 Pi/Orin CPU 开销可忽略
        finally:
            drain_task.cancel()


def main():
    rclpy.init()
    node = CloudTeleopBridge()
    # 不再开 spin daemon thread — run() 内部 spin_once 轮询
    try:
        asyncio.run(node.run())
    except KeyboardInterrupt:
        node.get_logger().info('[cloud_teleop] KeyboardInterrupt')
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
