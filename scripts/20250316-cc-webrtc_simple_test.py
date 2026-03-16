#!/usr/bin/env python3
"""
简单的 WebRTC 发送端测试
"""

import asyncio
import websockets
import json

SERVER = "ws://39.102.113.104:8765"

print("=" * 50)
print("  WebRTC 发送端测试")
print("=" * 50)
print(f"\n连接到: {SERVER}\n")

async def main():
    try:
        async with websockets.connect(SERVER) as ws:
            print("[✓] 连接成功！\n")

            # 发送注册消息
            msg = {"type": "register", "role": "sender", "ip": "36.110.28.59"}
            await ws.send(json.dumps(msg))
            print(f"[>] 已发送: {msg}\n")

            # 等待消息
            print("[*] 等待接收端连接...")
            try:
                response = await asyncio.wait_for(ws.recv(), timeout=60)
                print(f"[<] 收到: {response}\n")
            except asyncio.TimeoutError:
                print("[*] 60秒无响应，继续等待...\n")

    except Exception as e:
        print(f"[!] 错误: {e}\n")

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[!] 已停止")
