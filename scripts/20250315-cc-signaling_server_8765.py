#!/usr/bin/env python3
"""
========================================
  WebRTC 信令服务器 (端口 8765)
========================================

功能:
  - WebSocket 信令服务器，用于 WebRTC SDP 和 ICE 交换
  - 简单的广播模式：将收到的消息转发给所有其他连接的客户端

配置:
  - 监听端口: 8765 (避开敏感端口 8080)
  - 监听地址: 0.0.0.0 (允许所有 IP 连接)

使用:
  python3 signaling_server_8765.py

部署位置:
  云端服务器 39.102.113.114

作者: Claude Code
日期: 2025-03-15
"""

import asyncio
import websockets

# 这个集合用来存储当前连接进来的设备
connected_clients = set()

# 处理每个客户端连接的核心逻辑
async def signaling_handler(websocket):
    connected_clients.add(websocket)
    print(f"[+] 新设备已连接！当前在线设备数: {len(connected_clients)}")
    print(f"    远程地址: {websocket.remote_address}")

    try:
        # 持续监听该设备发来的消息
        async for message in websocket:
            print(f"[>] 收到消息 (长度: {len(message)} 字节)，准备转发...")

            # 将收到的消息，广播给房间里除了自己以外的其他设备
            for client in connected_clients:
                if client != websocket:
                    try:
                        await client.send(message)
                        print(f"[<] 消息已成功转发给目标设备")
                    except Exception as e:
                        print(f"[!] 转发失败: {e}")

    except websockets.exceptions.ConnectionClosed as e:
        print(f"[-] 设备连接断开: {e}")
    finally:
        # 设备断开时，把它从集合中清理掉
        connected_clients.discard(websocket)
        print(f"[-] 设备已清理，当前在线设备数: {len(connected_clients)}")

# 主函数：启动并维持服务器
async def main():
    print("🚀  信令服务器准备启动...")
    print("监听地址: ws://0.0.0.0:8765")
    print("等待设备连接...\n")

    # 使用 async with 来启动服务
    async with websockets.serve(signaling_handler, "0.0.0.0", 8765):
        await asyncio.Future()  # 挂起程序，让服务永久运行

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n🛑  信令服务器已手动停止。")
