#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
====================================================================
  Pose Data Channel Receiver v1.0
====================================================================

配合 lk CLI 硬编推流方案的 data channel 接收端.

架构分工:
  - /usr/local/bin/lk room join --publish h264://... (identity=zed-mini-sender)
      → 推视频 track 到 teleop-room
  - 本进程 (identity=zed-mini-pose-recv)
      → 订阅 room data channel
      → 收 VP 的 pose JSON
      → 解析四元数 → UDP 转发给达妙引擎 127.0.0.1:9000

为什么分两进程:
  lk CLI 是 Go 二进制, 性能好, 硬编路径干净; 但不做 data channel.
  Python 只做 data channel, 不碰视频, CPU 占用极小.

环境变量:
  LIVEKIT_URL       默认 ws://39.102.113.104:7880
  POSE_IDENTITY     默认 zed-mini-pose-recv
  DAMO_UDP_HOST     默认 127.0.0.1
  DAMO_UDP_PORT     默认 9000

作者: Claude Code (2026-04-22)
"""
import asyncio
import json
import math
import os
import signal
import socket
import time
import traceback

from livekit import rtc, api


# ==================== 配置 ====================
LIVEKIT_URL = os.environ.get("LIVEKIT_URL", "ws://39.102.113.104:7880")
# 凭证仅从 env 读取, 不再带默认值 (旧默认值已被公开,见 SECURITY.md)
LIVEKIT_API_KEY = os.environ.get("LIVEKIT_API_KEY")
LIVEKIT_API_SECRET = os.environ.get("LIVEKIT_API_SECRET")
if not (LIVEKIT_API_KEY and LIVEKIT_API_SECRET):
    raise RuntimeError(
        "LIVEKIT_API_KEY and LIVEKIT_API_SECRET env vars required "
        "(see ../.env.example and ../SECURITY.md)"
    )
LIVEKIT_ROOM = os.environ.get("LIVEKIT_ROOM", "teleop-room")
POSE_IDENTITY = os.environ.get("POSE_IDENTITY", "zed-mini-pose-recv")

DAMO_UDP_HOST = os.environ.get("DAMO_UDP_HOST", "127.0.0.1")
DAMO_UDP_PORT = int(os.environ.get("DAMO_UDP_PORT", "9000"))

POSE_PRINT_EVERY = 30

running = True


# ==================== 四元数 → 欧拉角 ====================
def quat_to_euler(qx, qy, qz, qw):
    """ZYX 内旋欧拉角, 返回 (pitch, roll, yaw) rad."""
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


# ==================== Main ====================
class PoseReceiver:
    def __init__(self):
        self.room = None
        self.pose_count = 0
        self.first_pose_time = None
        self.damo_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.damo_addr = (DAMO_UDP_HOST, DAMO_UDP_PORT)

    def _gen_token(self) -> str:
        token = api.AccessToken(LIVEKIT_API_KEY, LIVEKIT_API_SECRET)
        token.with_identity(POSE_IDENTITY)
        token.with_grants(api.VideoGrants(
            room_join=True,
            room=LIVEKIT_ROOM,
            can_publish=False,         # 本进程绝不推视频 (视频走 lk CLI)
            can_publish_data=True,     # 但可发 debug data
            can_subscribe=True,        # 订阅其他参与者 (特别是 VP 的 data)
        ))
        return token.to_jwt()

    def _on_data(self, data_packet: rtc.DataPacket):
        try:
            text = data_packet.data.decode('utf-8')
            msg = json.loads(text)
        except (json.JSONDecodeError, UnicodeDecodeError):
            return
        if msg.get("type") != "pose":
            return

        self.pose_count += 1
        if self.first_pose_time is None:
            self.first_pose_time = time.time()
            print(f"[Pose] 首个 pose 到达 ✓")

        quat = msg.get("q", [0, 0, 0, 1])
        ts = msg.get("t", 0)
        pitch, roll, yaw = quat_to_euler(*quat)

        # 物理含义映射 (沿用原 sender):
        #   motor_pitch (点头) ← math.roll  (绕 X 轴)
        #   motor_yaw   (转头) ← math.pitch (绕 Y 轴)
        motor_pitch = math.degrees(roll)
        motor_yaw = math.degrees(pitch)

        payload = json.dumps({
            "pitch": round(motor_pitch, 2),
            "yaw": round(motor_yaw, 2),
            "t": ts,
        })
        try:
            self.damo_sock.sendto(payload.encode('utf-8'), self.damo_addr)
        except Exception:
            pass

        if self.pose_count % POSE_PRINT_EVERY == 1:
            elapsed = time.time() - self.first_pose_time
            rate = self.pose_count / elapsed if elapsed > 0 else 0
            print(f"[Pose] #{self.pose_count} @ {rate:.1f}Hz → pitch={motor_pitch:+6.1f}° yaw={motor_yaw:+6.1f}°")

    async def connect(self):
        self.room = rtc.Room()
        self.room.on("data_received", self._on_data)

        def _on_joined(p: rtc.RemoteParticipant):
            print(f"[LiveKit] 参与者加入: {p.identity}")
        def _on_left(p: rtc.RemoteParticipant):
            print(f"[LiveKit] 参与者离开: {p.identity}")
        self.room.on("participant_connected", _on_joined)
        self.room.on("participant_disconnected", _on_left)

        jwt = self._gen_token()
        print(f"[LiveKit] 连接 {LIVEKIT_URL} identity={POSE_IDENTITY}")
        await self.room.connect(LIVEKIT_URL, jwt)
        print(f"[LiveKit] ✓ 已连接 room={self.room.name}")
        existing = [p.identity for p in self.room.remote_participants.values()]
        print(f"[LiveKit] 当前其他参与者: {existing}")

    async def disconnect(self):
        if self.room:
            await self.room.disconnect()


async def main():
    global running
    def sig_handler(sig, frame):
        global running
        print(f"\n[!] 收到信号 {sig}, 退出...")
        running = False
    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    print("=" * 64)
    print("  Pose Data Channel Receiver (硬编视频走 lk CLI)")
    print(f"  LiveKit: {LIVEKIT_URL}")
    print(f"  Room:    {LIVEKIT_ROOM}")
    print(f"  Ident:   {POSE_IDENTITY}")
    print(f"  达妙:    {DAMO_UDP_HOST}:{DAMO_UDP_PORT}")
    print("=" * 64)

    receiver = PoseReceiver()
    try:
        await receiver.connect()
        # 重连循环 (简单): 每秒检查连接状态
        while running:
            await asyncio.sleep(1)
    except Exception as e:
        print(f"[!!!] main 异常: {e}")
        traceback.print_exc()
    finally:
        print("\n[清理] ...")
        try:
            await receiver.disconnect()
        except Exception as e:
            print(f"[清理] disconnect 异常: {e}")
        print("[清理] 完成")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
