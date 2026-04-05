import asyncio
import websockets
import ssl
import json
import time
import os

# 核心状态存储
senders = {}
receivers = {}
pose_senders = {}

msg_counter = 0
pose_counter = 0

# 缓存：receiver 先连上发了 request_offer，但 sender 还没连
pending_request_offer = False
cached_offer = None

def log(msg):
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}")

def log_status():
    log(f"  sender={len(senders)} receiver={len(receivers)} pose={len(pose_senders)}")

async def signaling_handler(websocket):
    global msg_counter, pose_counter, pending_request_offer, cached_offer
    client_ip = websocket.remote_address[0]
    current_role = None

    log(f"新连接 [{client_ip}]")

    try:
        async for message in websocket:
            msg_counter += 1
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                continue

            msg_type = data.get("type", "unknown")

            # ===== 注册 =====
            if msg_type == "register":
                role = data.get("role")
                current_role = role
                name = data.get("name", "?")
                ip = data.get("ip", client_ip)
                if role == "sender":
                    senders[websocket] = ip
                    log(f"sender 注册 [{ip}] name={name}")
                    log_status()
                    # 如果之前有 receiver 请求过 offer，立刻通知
                    if pending_request_offer:
                        log("补发缓存的 request_offer 给新 sender")
                        try:
                            await websocket.send(json.dumps({"type": "request_offer"}))
                        except:
                            pass
                        pending_request_offer = False
                elif role == "receiver":
                    receivers[websocket] = ip
                    log(f"receiver 注册 [{ip}] name={name}")
                    log_status()
                    # 如果有缓存的 offer，立刻发给新 receiver
                    if cached_offer:
                        log("补发缓存的 offer 给新 receiver")
                        try:
                            await websocket.send(cached_offer)
                        except:
                            pass
                elif role == "pose-sender":
                    pose_senders[websocket] = ip
                    log(f"pose-sender 注册 [{ip}]")
                    log_status()

            # ===== request_offer: receiver 请求 sender 发 offer =====
            elif msg_type == "request_offer":
                if len(senders) > 0:
                    log(f"request_offer -> 转发给 {len(senders)} 个 sender")
                    for s in list(senders.keys()):
                        try:
                            await s.send(message)
                        except:
                            pass
                else:
                    pending_request_offer = True
                    log("request_offer 已缓存（等 sender 连上）")

            # ===== offer: sender -> 所有 receiver =====
            elif msg_type == "offer":
                sdp_len = len(data.get("sdp", ""))
                cached_offer = message  # 缓存原始 offer
                log(f"offer ({sdp_len} chars) -> {len(receivers)} 个 receiver (已缓存)")
                for r in list(receivers.keys()):
                    try:
                        await r.send(message)
                    except:
                        pass

            # ===== answer: receiver -> 所有 sender =====
            elif msg_type == "answer":
                sdp_len = len(data.get("sdp", ""))
                log(f"answer ({sdp_len} chars) -> {len(senders)} 个 sender")
                for s in list(senders.keys()):
                    try:
                        await s.send(message)
                    except:
                        pass

            # ===== ice: 按角色双向转发 =====
            elif msg_type in ("ice", "candidate"):
                if current_role == "sender":
                    for r in list(receivers.keys()):
                        try:
                            await r.send(message)
                        except:
                            pass
                elif current_role == "receiver":
                    for s in list(senders.keys()):
                        try:
                            await s.send(message)
                        except:
                            pass

            # ===== ping: sender -> receiver =====
            elif msg_type == "ping":
                for r in list(receivers.keys()):
                    try:
                        await r.send(message)
                    except:
                        pass

            # ===== pong: receiver -> sender =====
            elif msg_type == "pong":
                for s in list(senders.keys()):
                    try:
                        await s.send(message)
                    except:
                        pass

            # ===== pose: -> sender + 其他 receiver =====
            elif msg_type == "pose":
                pose_counter += 1
                for s in list(senders.keys()):
                    try:
                        await s.send(message)
                    except:
                        pass
                for r in list(receivers.keys()):
                    if r != websocket:
                        try:
                            await r.send(message)
                        except:
                            pass

    except websockets.exceptions.ConnectionClosed as e:
        log(f"断开: {client_ip} - {e}")
    except Exception as e:
        log(f"异常: {client_ip} - {e}")
    finally:
        if websocket in senders:
            del senders[websocket]
            log(f"sender [{client_ip}] 已移除")
            log_status()
        elif websocket in receivers:
            del receivers[websocket]
            log(f"receiver [{client_ip}] 已移除")
            log_status()
        elif websocket in pose_senders:
            del pose_senders[websocket]
            log(f"pose-sender [{client_ip}] 已移除")
            log_status()

async def main():
    print("=" * 40)
    print("  信令服务器 v9")
    print(f"  {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 40)

    ws_server = await websockets.serve(signaling_handler, "0.0.0.0", 8765)
    print("ws://0.0.0.0:8765 就绪")

    cert_file = os.path.expanduser("~/cert.pem")
    key_file = os.path.expanduser("~/key.pem")
    if os.path.exists(cert_file) and os.path.exists(key_file):
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_context.load_cert_chain(cert_file, key_file)
        wss_server = await websockets.serve(signaling_handler, "0.0.0.0", 8766, ssl=ssl_context)
        print("wss://0.0.0.0:8766 就绪")
    else:
        print("未找到 cert/key，WSS 未启用")

    print("等待连接...\n")
    await asyncio.Future()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n已停止")
