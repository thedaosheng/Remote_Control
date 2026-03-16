#!/usr/bin/env python3
"""
========================================
  WebRTC 信令服务器（优化版）
========================================

功能:
  - 管理 WebRTC 发送端和接收端的连接
  - 转发 Offer / Answer / ICE 消息
  - 处理连接时序问题
  - 详细的调试日志

作者: Claude Code
日期: 2025-03-16
"""

import asyncio
import websockets
import json
import logging
from datetime import datetime

# ============================================
# 配置
# ============================================
PORT = 8765
HOST = "0.0.0.0"

# ============================================
# 日志配置
# ============================================
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s | %(levelname)s | %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

# ============================================
# 全局状态管理
# ============================================
class SignalingState:
    """信令服务器状态管理"""
    def __init__(self):
        self.senders = {}      # {websocket: client_info}
        self.receivers = {}    # {websocket: client_info}
        self.pending_offers = []  # 存储等待接收端的 Offer
        self.offer_cache = {}  # {sender_ws: offer} - 缓存最近的 Offer

    def add_sender(self, ws, client_info):
        self.senders[ws] = client_info
        logger.info(f"➕ 发送端加入: {client_info['ip']} (总计: {len(self.senders)})")

    def add_receiver(self, ws, client_info):
        self.receivers[ws] = client_info
        logger.info(f"➕ 接收端加入: {client_info['ip']} (总计: {len(self.receivers)})")

    def remove_sender(self, ws):
        if ws in self.senders:
            info = self.senders[ws]
            del self.senders[ws]
            if ws in self.offer_cache:
                del self.offer_cache[ws]
            logger.info(f"➖ 发送端离开: {info['ip']} (剩余: {len(self.senders)})")

    def remove_receiver(self, ws):
        if ws in self.receivers:
            info = self.receivers[ws]
            del self.receivers[ws]
            logger.info(f"➖ 接收端离开: {info['ip']} (剩余: {len(self.receivers)})")

    def get_sender_count(self):
        return len(self.senders)

    def get_receiver_count(self):
        return len(self.receivers)


state = SignalingState()


# ============================================
# 消息处理
# ============================================
async def handle_offer(sender_ws, data, client_ip):
    """处理 Offer 消息"""
    sdp_preview = data.get('sdp', '')[:100].replace('\n', ' ')
    logger.info(f"📩 收到 Offer 来自 {client_ip}")
    logger.info(f"   SDP 预览: {sdp_preview}...")

    # 缓存 Offer（用于后来连接的接收端）
    state.offer_cache[sender_ws] = data

    receiver_count = state.get_receiver_count()
    logger.info(f"   当前接收端数量: {receiver_count}")

    if receiver_count == 0:
        logger.warning(f"⚠️  没有接收端在线！Offer 已缓存，等待接收端连接...")
        state.pending_offers.append(data)
        return

    # 转发给所有接收端
    for i, (recv_ws, recv_info) in enumerate(state.receivers.items(), 1):
        try:
            await recv_ws.send(json.dumps(data))
            logger.info(f"   ✅ Offer 已转发给接收端 #{i} ({recv_info['ip']})")
        except Exception as e:
            logger.error(f"   ❌ 转发失败给接收端 #{i}: {e}")


async def handle_answer(data, client_ip):
    """处理 Answer 消息"""
    sdp_preview = data.get('sdp', '')[:100].replace('\n', ' ')
    logger.info(f"📤 收到 Answer 来自 {client_ip}")
    logger.info(f"   SDP 预览: {sdp_preview}...")

    sender_count = state.get_sender_count()
    logger.info(f"   当前发送端数量: {sender_count}")

    if sender_count == 0:
        logger.warning(f"⚠️  没有发送端在线！Answer 无法转发...")
        return

    # 转发给所有发送端（通常只有一个）
    for i, (sender_ws, sender_info) in enumerate(state.senders.items(), 1):
        try:
            await sender_ws.send(json.dumps(data))
            logger.info(f"   ✅ Answer 已转发给发送端 #{i} ({sender_info['ip']})")
        except Exception as e:
            logger.error(f"   ❌ 转发失败给发送端 #{i}: {e}")


async def handle_ice(data, client_ip, source_role):
    """处理 ICE 候选消息"""
    candidate = data.get('candidate', '')[:60]
    logger.info(f"🌐 收到 ICE 来自 {client_ip} (来源: {source_role})")
    logger.info(f"   候选: {candidate}...")

    if source_role == 'sender':
        # 发送端的 ICE -> 转发给所有接收端
        target_name = "接收端"
        targets = state.receivers
    else:
        # 接收端的 ICE -> 转发给所有发送端
        target_name = "发送端"
        targets = state.senders

    target_count = len(targets)
    logger.info(f"   目标: {target_name}, 数量: {target_count}")

    if target_count == 0:
        logger.warning(f"   ⚠️  没有{target_name}在线！ICE 候选无法转发...")
        return

    for i, (target_ws, target_info) in enumerate(targets.items(), 1):
        try:
            await target_ws.send(json.dumps(data))
            logger.info(f"   ✅ ICE 已转发给{target_name} #{i} ({target_info['ip']})")
        except Exception as e:
            logger.error(f"   ❌ 转发失败给{target_name} #{i}: {e}")


# ============================================
# 信令处理主函数
# ============================================
async def signaling_handler(websocket):
    """处理 WebSocket 连接"""
    client_ip = websocket.remote_address[0]
    logger.info(f"🔗 新连接来自: {client_ip}")

    current_role = None
    client_name = None

    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                msg_type = data.get('type', '')

                # --- 注册消息 ---
                if msg_type == 'register':
                    current_role = data.get('role', 'unknown')
                    client_name = data.get('name', 'unnamed')

                    client_info = {
                        'ip': client_ip,
                        'name': client_name,
                        'role': current_role,
                        'connected_at': datetime.now()
                    }

                    if current_role == 'sender':
                        state.add_sender(websocket, client_info)
                    elif current_role == 'receiver':
                        state.add_receiver(websocket, client_info)

                        # 接收端刚连接，检查是否有缓存的 Offer
                        if state.offer_cache and state.senders:
                            logger.info(f"📦 检测到缓存的 Offer，自动转发给新连接的接收端...")
                            for sender_ws, offer_data in state.offer_cache.items():
                                try:
                                    await websocket.send(json.dumps(offer_data))
                                    sender_info = state.senders.get(sender_ws, {})
                                    logger.info(f"   ✅ 缓存 Offer 已转发 (来自: {sender_info.get('ip', 'unknown')})")
                                except Exception as e:
                                    logger.error(f"   ❌ 转发缓存 Offer 失败: {e}")

                # --- Offer 消息 ---
                elif msg_type == 'offer':
                    if current_role != 'sender':
                        logger.warning(f"⚠️  非发送端 ({current_role}) 尝试发送 Offer!")
                    await handle_offer(websocket, data, client_ip)

                # --- Answer 消息 ---
                elif msg_type == 'answer':
                    if current_role != 'receiver':
                        logger.warning(f"⚠️  非接收端 ({current_role}) 尝试发送 Answer!")
                    await handle_answer(data, client_ip)

                # --- ICE 候选消息 ---
                elif msg_type == 'ice':
                    await handle_ice(data, client_ip, current_role)

                # --- 心跳消息 ---
                elif msg_type == 'heartbeat':
                    # 心跳响应
                    await websocket.send(json.dumps({'type': 'heartbeat_ack'}))

                # --- 未知消息 ---
                else:
                    logger.info(f"❓ 未知消息类型: {msg_type} | 完整数据: {data}")

            except json.JSONDecodeError as e:
                logger.error(f"🚫 JSON 解析错误: {e} | 原始消息: {message[:200]}")
            except Exception as e:
                logger.error(f"🚫 消息处理错误: {e}")

    except websockets.exceptions.ConnectionClosed:
        logger.info(f"🔌 连接关闭: {client_ip} (角色: {current_role})")
    finally:
        # 清理断开的连接
        if current_role == 'sender':
            state.remove_sender(websocket)
        elif current_role == 'receiver':
            state.remove_receiver(websocket)
        else:
            logger.info(f"🔌 未注册连接断开: {client_ip}")


# ============================================
# 主程序
# ============================================
async def main():
    print("\n" + "=" * 50)
    print("  WebRTC 信令服务器（优化版）")
    print("=" * 50)
    print(f"  监听地址: {HOST}:{PORT}")
    print(f"  启动时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 50 + "\n")

    logger.info(f"🚀 服务器启动在 {HOST}:{PORT}")

    async with websockets.serve(signaling_handler, HOST, PORT):
        # 保持服务器运行
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n" + "=" * 50)
        print("🛑  服务器已停止")
        print("=" * 50)
