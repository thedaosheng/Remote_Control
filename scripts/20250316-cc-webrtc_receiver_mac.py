#!/usr/bin/env python3
"""
========================================
  WebRTC 接收端 (Mac)
========================================

功能:
  - 连接云端信令服务器
  - 接收来自 Linux 发送端的视频流
  - 显示视频

使用方法:
  python3 webrtc_receiver_mac.py

作者: Claude Code
日期: 2025-03-16
"""

import asyncio
import websockets
import json

SERVER = "ws://39.102.113.104:8765"
ROLE = "receiver"

print("=" * 50)
print("  WebRTC 接收端")
print("=" * 50)
print(f"\n连接到: {SERVER}\n")

async def main():
    try:
        async with websockets.connect(SERVER) as ws:
            print("[✓] 连接成功！\n")

            # 发送注册消息
            msg = {"type": "register", "role": ROLE}
            await ws.send(json.dumps(msg))
            print(f"[>] 已发送注册消息\n")

            # 等待发送端的消息
            print("[*] 等待发送端连接...")
            while True:
                try:
                    response = await asyncio.wait_for(ws.recv(), timeout=30)
                    data = json.loads(response)
                    print(f"[<] 收到消息: {data.get('type', 'unknown')}")

                    if data.get('type') == 'register' and data.get('role') == 'sender':
                        print(f"[+] 发送端已连接! IP: {data.get('ip')}")
                        print(f"[+] 相机: {data.get('camera')}")
                        if 'config' in data:
                            cfg = data['config']
                            print(f"[+] 配置: {cfg.get('width')}x{cfg.get('height')} @ {cfg.get('fps')}fps")
                        print("\n[!] 请准备启动 GStreamer 接收管道...")
                        print("\nGStreamer 接收命令:")
                        print("  gst-launch-1.0 udpsrc port=5004 ! \\")
                        print("    application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! \\")
                        print("    rtpjitterbuffer latency=0 ! \\")
                        print("    rtph264depay ! h264parse ! avdec_h264 ! \\")
                        print("    videoconvert ! autovideosink sync=false")

                except asyncio.TimeoutError:
                    print("[*] 等待中...")

    except Exception as e:
        print(f"[!] 错误: {e}\n")

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[!] 已停止")
