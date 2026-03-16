#!/usr/bin/env python3
"""
RealSense 双目 WebRTC 信令客户端（修复版）

核心修复:
  - 不再依赖 GStreamer 自动触发 on-negotiation-needed
  - 改为 WebSocket 注册成功后，手动延迟 2 秒再启动管道
  - send_message 加了详细 debug，能看到消息是否真的发出去了
  - 所有 GLib 线程回调都加了 try/except，防止静默失败

配置:
  - 信令服务器: ws://39.102.113.104:8765
  - TURN 服务器: turn://remote:Wawjxyz3!@39.102.113.104:3478
  - 左相机: /dev/video4
"""

import asyncio
import websockets
import json
import signal
import sys
import time
import threading
import traceback
import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
gi.require_version('GstSdp', '1.0')
from gi.repository import Gst, GstWebRTC, GstSdp, GLib

# ============================================
# 配置区
# ============================================
SIGNALING_SERVER = "ws://39.102.113.104:8765"
TURN_SERVER = "turn://remote:Wawjxyz3%21@39.102.113.104:3478"
PUBLIC_IP = "36.110.28.59"

PIPELINE_DESC = f'''
webrtcbin name=sendbin turn-server="{TURN_SERVER}"
v4l2src device=/dev/video4 do-timestamp=true
! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1
! videoconvert
! video/x-raw,format=I420
! nvh264enc preset=5 zerolatency=true bitrate=2000 rc-mode=cbr
! rtph264pay config-interval=1
! application/x-rtp,media=video,encoding-name=H264,payload=96
! queue max-size-buffers=0
! sendbin.
'''

# 全局事件循环
loop = None
running = True


class WebRTCSender:
    def __init__(self):
        print("[*] 初始化 GStreamer 管道...")
        self.pipe = Gst.parse_launch(PIPELINE_DESC)
        self.webrtc = self.pipe.get_by_name('sendbin')
        self.ws = None
        self.connected = False
        self.offer_sent = False  # 防止重复发送 Offer

        # 管道错误监控
        bus = self.pipe.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.on_bus_message)

        # 绑定 WebRTC 信号
        self.webrtc.connect('on-negotiation-needed', self.on_negotiation_needed)
        self.webrtc.connect('on-ice-candidate', self.send_ice_candidate_message)

        print("[✓] GStreamer 管道已初始化")

    def on_bus_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"\n[!!!] GStreamer 致命错误: {err}")
            print(f"[!!!] 详细原因: {debug}\n")
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            print(f"[WARN] 管道警告: {err}")

    def on_negotiation_needed(self, element):
        """
        GStreamer 自动触发的协商信号。
        关键修复：检查 WebSocket 是否真的就绪，如果没有就跳过，
        后面会由 connect_signaling 里的手动触发来补上。
        """
        print(f"[DEBUG] on-negotiation-needed 触发! ws={self.ws is not None}, offer_sent={self.offer_sent}")
        if self.ws is None:
            print("[!] WebSocket 还没连上，跳过本次自动触发，等手动触发")
            return
        if self.offer_sent:
            print("[!] Offer 已经发过了，跳过重复触发")
            return
        self._do_create_offer(element)

    def _do_create_offer(self, element):
        """真正创建 Offer 的方法"""
        print("[*] 开始生成 SDP Offer...")
        try:
            promise = Gst.Promise.new_with_change_func(self.on_offer_created, element, None)
            element.emit('create-offer', None, promise)
        except Exception as e:
            print(f"[!!!] create-offer 异常: {e}")
            traceback.print_exc()

    def on_offer_created(self, promise, element, _):
        """SDP Offer 创建完成回调 (在 GLib 线程中执行)"""
        try:
            promise.wait()
            reply = promise.get_reply()
            if reply is None:
                print("[!!!] create-offer 返回了 None reply!")
                return
            offer = reply.get_value('offer')
            if offer is None:
                print("[!!!] reply 中没有 offer 值!")
                return

            print("[✓] Offer 生成成功，设置本地描述...")
            promise2 = Gst.Promise.new()
            element.emit('set-local-description', offer, promise2)

            text = offer.sdp.as_text()
            msg = {
                'type': 'offer',
                'sdp': text,
                'sender_ip': PUBLIC_IP
            }

            print(f"[DEBUG] 准备发送 Offer, sdp 长度={len(text)}, loop={loop is not None}")

            if loop is None:
                print("[!!!] asyncio event loop 是 None! Offer 无法发送!")
                return

            future = asyncio.run_coroutine_threadsafe(self.send_message(msg), loop)
            # 等待发送完成，最多 5 秒
            try:
                future.result(timeout=5)
                self.offer_sent = True
                print("[✓] SDP Offer 已确认发送至信令服务器！")
            except Exception as e:
                print(f"[!!!] Offer 发送 future 异常: {e}")

        except Exception as e:
            print(f"[!!!] on_offer_created 异常: {e}")
            traceback.print_exc()

    def send_ice_candidate_message(self, element, mlineindex, candidate):
        try:
            msg = {
                'type': 'ice',
                'sdpMLineIndex': mlineindex,
                'candidate': candidate
            }
            if loop is not None:
                asyncio.run_coroutine_threadsafe(self.send_message(msg), loop)
                print(f"[ICE] 候选已提交: {candidate[:60]}...")
            else:
                print(f"[!!!] ICE 候选丢失: loop 是 None")
        except Exception as e:
            print(f"[!!!] send_ice_candidate 异常: {e}")

    async def send_message(self, msg):
        """发送消息到信令服务器 - 带详细 debug"""
        msg_type = msg.get('type', 'unknown')
        msg_json = json.dumps(msg)
        msg_size = len(msg_json)

        if self.ws is None:
            print(f"[!!!] send_message 失败: ws 是 None (type={msg_type}, size={msg_size})")
            return False

        # websockets.ClientConnection 没有 closed 属性，用 state 检查
        # OPEN = 1, CLOSING = 2, CLOSED = 3
        from websockets.connection import State
        if hasattr(self.ws, 'state') and self.ws.state != State.OPEN:
            print(f"[!!!] send_message 失败: ws 状态={self.ws.state} (type={msg_type})")
            return False

        try:
            await self.ws.send(msg_json)
            print(f"[>>>] 消息已发送: type={msg_type}, size={msg_size} 字节")
            return True
        except Exception as e:
            print(f"[!!!] send_message 异常: {e} (type={msg_type})")
            traceback.print_exc()
            return False

    async def connect_signaling(self):
        global running, loop

        print(f"[*] 正在连接信令服务器: {SIGNALING_SERVER}")
        print(f"[*] TURN 服务器: {TURN_SERVER}")
        print(f"[*] 公网 IP: {PUBLIC_IP}")
        print("")

        retry_count = 0

        while running:
            try:
                async with websockets.connect(SIGNALING_SERVER, close_timeout=5) as websocket:
                    self.ws = websocket
                    self.connected = True
                    self.offer_sent = False  # 重连时重置

                    print(f"[✓] 信令服务器连接成功 (尝试 {retry_count + 1})")

                    # 1. 先注册
                    register_msg = json.dumps({
                        "type": "register",
                        "role": "sender",
                        "ip": PUBLIC_IP,
                        "name": "Linux-Stereo-Sender",
                        "cameras": {"left": 51004, "right": 51006}
                    })
                    await websocket.send(register_msg)
                    print(f"[>>>] 注册消息已发送: {register_msg}")

                    retry_count = 0
                    print("[✓] 已注册为发送端")
                    print("")

                    # 2. 等 1 秒让注册消息确认到达云端
                    print("[*] 等待 1 秒确保注册完成...")
                    await asyncio.sleep(1)

                    # 3. 启动管道
                    print("[*] 启动相机推流管道...")
                    self.pipe.set_state(Gst.State.PLAYING)
                    print("[✓] 相机管道已启动")

                    # 4. 再等 2 秒让管道稳定 + negotiation 自然触发
                    print("[*] 等待 2 秒让管道稳定...")
                    await asyncio.sleep(2)

                    # 5. 如果 Offer 还没发（negotiation 没自动触发或被跳过），手动补发
                    if not self.offer_sent:
                        print("[!] Offer 尚未发送，手动触发 create-offer...")
                        self._do_create_offer(self.webrtc)
                        await asyncio.sleep(1)  # 给 GLib 线程时间处理

                    if self.offer_sent:
                        print("\n" + "="*50)
                        print("  ✅ Offer 已成功发送！等待接收端 Answer...")
                        print("="*50 + "\n")
                    else:
                        print("\n[!!!] 警告: Offer 仍未发送，可能是管道/摄像头问题")
                        print("[!!!] 请检查摄像头 /dev/video4 是否被占用\n")

                    # 6. 消息监听循环
                    async for message in websocket:
                        if not running:
                            break
                        try:
                            data = json.loads(message)
                            msg_type = data.get('type', 'unknown')
                            print(f"[<<<] 收到消息: type={msg_type}, size={len(message)} 字节")
                            await self.handle_signaling_message(data)
                        except json.JSONDecodeError:
                            print(f"[!] 收到无效 JSON: {message[:100]}")
                        except Exception as e:
                            print(f"[!] 处理消息失败: {e}")
                            traceback.print_exc()

            except websockets.exceptions.ConnectionClosed:
                if running:
                    print("\n[!] 信令连接已断开，5秒后重连...")
            except Exception as e:
                if running:
                    print(f"[!] 连接异常，5秒后重连... ({e})")
                    traceback.print_exc()

            if running:
                self.ws = None
                self.connected = False
                await asyncio.sleep(5)
                retry_count += 1

    async def handle_signaling_message(self, data):
        msg_type = data.get('type', '')

        if msg_type == 'answer':
            print("[<] 收到来自接收端的 SDP Answer！")
            try:
                res, sdpmsg = GstSdp.SDPMessage.new()
                GstSdp.sdp_message_parse_buffer(bytes(data['sdp'].encode()), sdpmsg)
                answer = GstWebRTC.WebRTCSessionDescription.new(
                    GstWebRTC.WebRTCSDPType.ANSWER, sdpmsg)
                promise = Gst.Promise.new()
                self.webrtc.emit('set-remote-description', answer, promise)
                print("[✓] 远程描述已设置，WebRTC 握手完成！")
                print("[*] 视频流正在通过 TURN 中继传输...")
            except Exception as e:
                print(f"[!] 设置远程描述失败: {e}")
                traceback.print_exc()

        elif msg_type == 'ice':
            print(f"[<] 收到接收端的 ICE 候选")
            try:
                self.webrtc.emit('add-ice-candidate',
                                data['sdpMLineIndex'],
                                data['candidate'])
            except Exception as e:
                print(f"[!] 添加 ICE 候选失败: {e}")

        else:
            if msg_type != 'heartbeat_ack':
                print(f"[?] 收到未知消息类型: {msg_type}")


def start_glib_loop():
    glib_loop = GLib.MainLoop()
    try:
        glib_loop.run()
    except KeyboardInterrupt:
        glib_loop.quit()


def signal_handler(sig, frame):
    global running
    print("\n[!] 收到停止信号...")
    running = False
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


if __name__ == "__main__":
    print("=" * 50)
    print("  RealSense 双目 WebRTC 发送端 (修复版)")
    print("=" * 50)
    print("")

    Gst.init(None)

    glib_thread = threading.Thread(target=start_glib_loop, daemon=True)
    glib_thread.start()
    print("[✓] GLib 主循环已启动")

    sender = WebRTCSender()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    try:
        loop.run_until_complete(sender.connect_signaling())
    except KeyboardInterrupt:
        print("\n[!] 正在关闭摄像头并退出...")
        sender.pipe.set_state(Gst.State.NULL)
        sys.exit(0)