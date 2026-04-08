#!/usr/bin/python3
"""
========================================
  RealSense 双目 WebRTC 发送端
========================================

功能:
  - 连接云端信令服务器 (ws://39.102.113.104:8765)
  - 采集 RealSense 双目相机视频
  - NVENC 硬件编码 H.264
  - 通过 WebRTC + TURN 中继推送到公网

配置:
  - 信令服务器: ws://39.102.113.104:8765
  - TURN 服务器: turn://remote:Wawjxyz3!@39.102.113.104:3478
  - 左相机: /dev/video4
  - 右相机: /dev/video16

注意: 必须使用系统 Python (/usr/bin/python3) 以获得 gi 模块支持

作者: Claude Code
日期: 2025-03-16
"""

import asyncio
import websockets
import json
import threading
import sys
import os
import gi

# GStreamer 相关初始化
gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
gi.require_version('GstSdp', '1.0')
from gi.repository import Gst, GstWebRTC, GstSdp, GLib

# ============================================
# 配置区
# ============================================
# 注意：原代码中 IP 是 39.102.113.104，已修正为 39.102.113.104
SIGNALING_SERVER = "ws://39.102.113.104:8765"
TURN_SERVER = "turn://remote:Wawjxyz3!@39.102.113.104:3478"

# 本机公网 IP (自动获取或手动设置)
PUBLIC_IP = "36.110.28.59"  # 你的公网 IP

# ============================================
# GStreamer 管道配置
# ============================================
# 左相机管道 (单路测试，后续可扩展为双路)
PIPELINE_DESC = f'''
    webrtcbin name=webrtcbin turn-server="{TURN_SERVER}"
    v4l2src device=/dev/video4 do-timestamp=true
    ! video/x-raw,format=YUY2,width=1280,height=720,framerate=30/1
    ! videoconvert
    ! video/x-raw,format=I420
    ! nvh264enc preset=5 zerolatency=true bitrate=4000 rc-mode=cbr
    ! rtph264pay config-interval=1
    ! application/x-rtp,media=video,encoding-name=H264,payload=96
    ! webrtcbin.
'''

# ============================================
# WebRTC 发送端类
# ============================================
class WebRTCSender:
    def __init__(self):
        """初始化 WebRTC 发送端"""
        # 解析 GStreamer 管道
        self.pipe = Gst.parse_launch(PIPELINE_DESC)
        self.webrtc = self.pipe.get_by_name('webrtcbin')
        self.ws = None
        self.connected = False

        # 绑定 WebRTC 信号
        self.webrtc.connect('on-negotiation-needed', self.on_negotiation_needed)
        self.webrtc.connect('on-ice-candidate', self.send_ice_candidate_message)

    def on_negotiation_needed(self, element):
        """WebRTC 协商信号处理"""
        print("[!] 需要协商，开始生成 SDP Offer...")
        promise = Gst.Promise.new_with_change_func(self.on_offer_created, element, None)
        element.emit('create-offer', None, promise)

    def on_offer_created(self, promise, element, _):
        """SDP Offer 创建完成回调"""
        promise.wait()
        reply = promise.get_reply()
        offer = reply.get_value('offer')

        print("[+] Offer 生成成功，设置本地描述...")
        promise2 = Gst.Promise.new()
        element.emit('set-local-description', offer, promise2)

        # 将 Offer 转换为文本并通过 WebSocket 发送
        text = offer.sdp.as_text()
        msg = {
            'type': 'offer',
            'sdp': text,
            'sender_ip': PUBLIC_IP
        }
        asyncio.run_coroutine_threadsafe(self.send_message(msg), loop)
        print("[>] SDP Offer 已发送至信令服务器！")

    def send_ice_candidate_message(self, element, mlineindex, candidate):
        """ICE 候选发送"""
        msg = {
            'type': 'ice',
            'sdpMLineIndex': mlineindex,
            'candidate': candidate
        }
        asyncio.run_coroutine_threadsafe(self.send_message(msg), loop)
        print(f"[>] ICE 候选已发送: {candidate[:50]}...")

    async def send_message(self, msg):
        """发送消息到信令服务器"""
        if self.ws and not self.ws.closed:
            try:
                await self.ws.send(json.dumps(msg))
            except Exception as e:
                print(f"[!] 发送消息失败: {e}")

    async def connect_signaling(self):
        """连接信令服务器"""
        print(f"[*] 正在连接信令服务器: {SIGNALING_SERVER}")
        try:
            async with websockets.connect(SIGNALING_SERVER) as websocket:
                self.ws = websocket
                self.connected = True
                print("[+] 信令服务器连接成功！")

                # 发送注册消息（包含公网 IP）
                register_msg = {
                    'type': 'register',
                    'role': 'sender',
                    'ip': PUBLIC_IP,
                    'camera': 'realsense-d435-stereo',
                    'left_port': 5004,
                    'right_port': 5006
                }
                await websocket.send(json.dumps(register_msg))
                print(f"[>] 已发送注册消息: {register_msg}")

                print("[!] 启动相机推流管道...")
                self.pipe.set_state(Gst.State.PLAYING)
                print("[+] 相机管道已启动，开始采集视频...")

                # 持续监听来自接收端的消息
                async for message in websocket:
                    try:
                        data = json.loads(message)
                        await self.handle_signaling_message(data)
                    except json.JSONDecodeError:
                        print(f"[!] 收到无效 JSON: {message}")
                    except Exception as e:
                        print(f"[!] 处理消息失败: {e}")

        except Exception as e:
            print(f"[!] 信令服务器连接失败: {e}")
            import traceback
            traceback.print_exc()

    async def handle_signaling_message(self, data):
        """处理信令消息"""
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
                print("[+] 远程描述已设置，WebRTC 握手完成！")
                print("[*] 视频流正在通过 TURN 中继传输...")
            except Exception as e:
                print(f"[!] 设置远程描述失败: {e}")

        elif msg_type == 'ice':
            print(f"[<] 收到接收端的 ICE 候选")
            try:
                self.webrtc.emit('add-ice-candidate',
                                data['sdpMLineIndex'],
                                data['candidate'])
            except Exception as e:
                print(f"[!] 添加 ICE 候选失败: {e}")

        else:
            print(f"[?] 收到未知消息类型: {msg_type}")

# ============================================
# GLib 主循环（GStreamer 需要）
# ============================================
def start_glib_loop():
    """在后台线程启动 GStreamer 主循环"""
    loop = GLib.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt:
        loop.quit()

# ============================================
# 主程序
# ============================================
if __name__ == '__main__':
    # 初始化 GStreamer
    Gst.init(None)

    print("=" * 50)
    print("  RealSense 双目 WebRTC 发送端")
    print("=" * 50)
    print(f"信令服务器: {SIGNALING_SERVER}")
    print(f"TURN 服务器: {TURN_SERVER}")
    print(f"公网 IP: {PUBLIC_IP}")
    print("=" * 50)

    # 在后台线程启动 GStreamer 引擎
    glib_thread = threading.Thread(target=start_glib_loop, daemon=True)
    glib_thread.start()

    # 创建 WebRTC 发送端
    sender = WebRTCSender()

    # 创建事件循环
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    try:
        # 启动 WebSocket 监听
        loop.run_until_complete(sender.connect_signaling())
    except KeyboardInterrupt:
        print("\n[!] 正在关闭摄像头并退出...")
        sender.pipe.set_state(Gst.State.NULL)
        sys.exit(0)
