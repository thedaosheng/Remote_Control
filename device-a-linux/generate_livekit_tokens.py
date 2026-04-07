#!/usr/bin/env python3
"""
LiveKit JWT Token 生成脚本

为 teleop 系统的各个参与者生成 LiveKit 访问令牌。
Token 用于认证 LiveKit Room 连接。

使用方法:
    python3 20260406-cc-generate_livekit_tokens.py

依赖:
    pip install livekit-api

配置:
    - API Key / Secret 与 LiveKit Server 配置一致
    - Room 名称需要与 sender/receiver 代码中一致
"""

from livekit import api
import datetime
import json
import base64

# ============================================================
# LiveKit Server 配置
# ============================================================
LIVEKIT_API_KEY = "teleop_key"
LIVEKIT_API_SECRET = "teleop_secret_key_2026"
LIVEKIT_ROOM = "teleop-room"
LIVEKIT_URL = "ws://39.102.113.104:7880"

# Token 有效期（天）
TOKEN_TTL_DAYS = 30


def generate_receiver_token():
    """
    生成 Vision Pro receiver 端的 token。

    权限:
      - room_join: True     → 允许加入房间
      - can_subscribe: True → 允许订阅远端轨道（接收视频）
      - can_publish: False  → 不需要发布音视频
      - can_publish_data: True → 通过 Data Channel 发送 pose 数据
    """
    token = api.AccessToken(LIVEKIT_API_KEY, LIVEKIT_API_SECRET)
    token.with_identity("vision-pro-receiver")
    token.with_name("Vision Pro Receiver")
    token.with_grants(api.VideoGrants(
        room_join=True,
        room=LIVEKIT_ROOM,
        can_subscribe=True,
        can_publish=False,
        can_publish_data=True,
    ))
    token.with_ttl(datetime.timedelta(days=TOKEN_TTL_DAYS))
    return token.to_jwt()


def generate_sender_token():
    """
    生成 Linux sender 端的 token。

    权限:
      - room_join: True       → 允许加入房间
      - can_subscribe: True   → 允许订阅（接收 receiver 发的 data）
      - can_publish: True     → 允许发布视频轨道
      - can_publish_data: True → 允许发送 data（如机器人状态）
    """
    token = api.AccessToken(LIVEKIT_API_KEY, LIVEKIT_API_SECRET)
    token.with_identity("linux-sender")
    token.with_name("Linux ZED Sender")
    token.with_grants(api.VideoGrants(
        room_join=True,
        room=LIVEKIT_ROOM,
        can_subscribe=True,
        can_publish=True,
        can_publish_data=True,
    ))
    token.with_ttl(datetime.timedelta(days=TOKEN_TTL_DAYS))
    return token.to_jwt()


def decode_token_info(jwt_token):
    """解码 JWT payload 查看 token 内容（不验证签名）"""
    parts = jwt_token.split(".")
    if len(parts) != 3:
        return {}
    # 补齐 base64 padding
    payload = parts[1] + "=" * (4 - len(parts[1]) % 4)
    decoded = base64.urlsafe_b64decode(payload)
    return json.loads(decoded)


if __name__ == "__main__":
    print("=" * 60)
    print("LiveKit Token 生成器")
    print(f"Server: {LIVEKIT_URL}")
    print(f"Room:   {LIVEKIT_ROOM}")
    print(f"有效期: {TOKEN_TTL_DAYS} 天")
    print("=" * 60)

    # 生成 Receiver token
    receiver_token = generate_receiver_token()
    receiver_info = decode_token_info(receiver_token)
    print(f"\n--- RECEIVER Token ---")
    print(f"Identity: {receiver_info.get('sub', '?')}")
    print(f"Expires:  {datetime.datetime.fromtimestamp(receiver_info.get('exp', 0))}")
    print(f"Token:\n{receiver_token}")

    # 生成 Sender token
    sender_token = generate_sender_token()
    sender_info = decode_token_info(sender_token)
    print(f"\n--- SENDER Token ---")
    print(f"Identity: {sender_info.get('sub', '?')}")
    print(f"Expires:  {datetime.datetime.fromtimestamp(sender_info.get('exp', 0))}")
    print(f"Token:\n{sender_token}")

    print("\n" + "=" * 60)
    print("请将上述 token 分别填入:")
    print("  - Receiver: LiveKitManager.swift → LiveKitConfig.receiverToken")
    print("  - Sender:   Python sender 脚本的 token 参数")
    print("=" * 60)
