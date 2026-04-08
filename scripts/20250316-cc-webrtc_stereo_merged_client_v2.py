#!/usr/bin/env python3
"""
========================================
  RealSense 双目拼接 WebRTC 发送端 V2
  修复 not-negotiated 问题
========================================
"""

import asyncio
import websockets
import json
import signal
import sys
import threading
import traceback
import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
gi.require_version('GstSdp', '1.0')
from gi.repository import Gst, GstWebRTC, GstSdp, GLib

# 配置
SIGNALING_SERVER = "ws://39.102.113.104:8765"
TURN_SERVER = "turn://remote:Wawjxyz3%21@39.102.113.104:3478"
PUBLIC_IP = "36.110.28.59"

# 全局变量
loop = None
running = True


class StereoMergedSenderV2:
    """双目拼接 WebRTC 发送端 V2 - 使用程序化管道构建"""

    def __init__(self):
        print("[*] 初始化 GStreamer...")
        print("    左相机 /dev/video4  -> 左半屏 (0-960)")
        print("    右相机 /dev/video10 -> 右半屏 (960-1920)")
        print("    输出分辨率: 1920x540 @ 30fps")
        print("")

        self.pipe = None
        self.webrtc = None
        self.ws = None
        self.connected = False
        self.offer_sent = False
        self.pipeline_started = False

        try:
            self._build_pipeline()
            print("[✓] 管道已构建\n")
        except Exception as e:
            print(f"[!!!] 管道构建失败: {e}")
            traceback.print_exc()

    def _build_pipeline(self):
        """程序化构建管道，避免 parse_launch 的协商问题"""
        # 创建管道
        self.pipe = Gst.Pipeline()

        # 创建元素
        # 左相机源
        src_left = Gst.ElementFactory.make("v4l2src", "src_left")
        src_left.set_property("device", "/dev/video4")
        src_left.set_property("do-timestamp", True)

        conv_left = Gst.ElementFactory.make("videoconvert", "conv_left")

        # 右相机源
        src_right = Gst.ElementFactory.make("v4l2src", "src_right")
        src_right.set_property("device", "/dev/video10")
        src_right.set_property("do-timestamp", True)

        conv_right = Gst.ElementFactory.make("videoconvert", "conv_right")

        # Compositor (视频拼接)
        compositor = Gst.ElementFactory.make("compositor", "mix")
        compositor.set_property("background", 1)  # 黑色背景

        # 设置每个 sink pad 的位置
        # 注意：需要在元素添加到管道后设置

        # 编码器
        encoder = Gst.ElementFactory.make("nvh264enc", "encoder")
        encoder.set_property("preset", 5)  # fastest
        encoder.set_property("zerolatency", True)
        encoder.set_property("bitrate", 6000)
        encoder.set_property("rc-mode", "cbr")

        # RTP payloader
        payloader = Gst.ElementFactory.make("rtph264pay", "pay")
        payloader.set_property("config-interval", 1)

        # WebRTC bin
        self.webrtc = Gst.ElementFactory.make("webrtcbin", "sendbin")
        self.webrtc.set_property("turn-server", TURN_SERVER)

        # 添加所有元素到管道
        for elem in [src_left, conv_left, src_right, conv_right,
                     compositor, encoder, payloader, self.webrtc]:
            self.pipe.add(elem)

        # 设置 caps filter (左相机)
        caps_left = Gst.ElementFactory.make("capsfilter", "caps_left")
        caps_left.set_property("caps",
            Gst.Caps.from_string("video/x-raw,format=YUY2,width=960,height=540,framerate=30/1"))
        self.pipe.add(caps_left)

        # 设置 caps filter (右相机)
        caps_right = Gst.ElementFactory.make("capsfilter", "caps_right")
        caps_right.set_property("caps",
            Gst.Caps.from_string("video/x-raw,format=YUY2,width=960,height=540,framerate=30/1"))
        self.pipe.add(caps_right)

        # 设置 caps filter (compositor 输出)
        caps_out = Gst.ElementFactory.make("capsfilter", "caps_out")
        caps_out.set_property("caps",
            Gst.Caps.from_string("video/x-raw,width=1920,height=540,framerate=30/1"))
        self.pipe.add(caps_out)

        # videoconvert: 将 compositor 输出转为 I420 (nvh264enc 需要的格式)
        conv_enc = Gst.ElementFactory.make("videoconvert", "conv_enc")
        self.pipe.add(conv_enc)

        # 设置 caps filter (编码器输入 - I420)
        caps_enc = Gst.ElementFactory.make("capsfilter", "caps_enc")
        caps_enc.set_property("caps",
            Gst.Caps.from_string("video/x-raw,format=I420"))
        self.pipe.add(caps_enc)

        # 连接左相机链
        if not src_left.link(caps_left):
            raise Exception("链接 src_left -> caps_left 失败")
        if not caps_left.link(conv_left):
            raise Exception("链接 caps_left -> conv_left 失败")

        # 连接右相机链
        if not src_right.link(caps_right):
            raise Exception("链接 src_right -> caps_right 失败")
        if not caps_right.link(conv_right):
            raise Exception("链接 caps_right -> conv_right 失败")

        # 连接 compositor -> conv_enc -> caps_enc -> encoder -> payloader
        if not compositor.link(caps_out):
            raise Exception("链接 compositor -> caps_out 失败")
        if not caps_out.link(conv_enc):
            raise Exception("链接 caps_out -> conv_enc 失败")
        if not conv_enc.link(caps_enc):
            raise Exception("链接 conv_enc -> caps_enc 失败")
        if not caps_enc.link(encoder):
            raise Exception("链接 caps_enc -> encoder 失败")
        if not encoder.link(payloader):
            raise Exception("链接 encoder -> payloader 失败")

        # 获取 compositor 的 sink pads
        compositor_sink_0 = compositor.request_pad_simple("sink_0")
        compositor_sink_1 = compositor.request_pad_simple("sink_1")

        if not compositor_sink_0 or not compositor_sink_1:
            raise Exception("获取 compositor sink pads 失败")

        # 设置位置
        compositor_sink_0.set_property("xpos", 0)
        compositor_sink_0.set_property("ypos", 0)
        compositor_sink_1.set_property("xpos", 960)
        compositor_sink_1.set_property("ypos", 0)

        # 连接左相机到 compositor (使用 pad link，注意返回值是 PadLinkReturn 枚举)
        conv_left_src = conv_left.get_static_pad("src")
        ret = conv_left_src.link(compositor_sink_0)
        if ret != Gst.PadLinkReturn.OK:
            raise Exception(f"链接 conv_left -> compositor sink_0 失败: {ret}")

        # 连接右相机到 compositor
        conv_right_src = conv_right.get_static_pad("src")
        ret = conv_right_src.link(compositor_sink_1)
        if ret != Gst.PadLinkReturn.OK:
            raise Exception(f"链接 conv_right -> compositor sink_1 失败: {ret}")

        # 连接 payloader 到 webrtcbin (使用 request pad)
        payloader_src = payloader.get_static_pad("src")

        # webrtcbin 需要动态获取 request pad
        def on_payloader_linking(pad, info):
            webrtc_sink = self.webrtc.get_request_pad("sink_0")
            if webrtc_sink:
                pad.link(webrtc_sink)
                print("[DEBUG] payloader -> webrtcbin 已连接")
            else:
                print("[!!!] 获取 webrtcbin sink_0 失败")

        # 直接连接 payloader -> webrtcbin
        webrtc_sink = self.webrtc.request_pad_simple("sink_0")
        if not webrtc_sink:
            raise Exception("获取 webrtcbin sink pad 失败")

        ret = payloader_src.link(webrtc_sink)
        if ret != Gst.PadLinkReturn.OK:
            raise Exception(f"链接 payloader -> webrtcbin 失败: {ret}")

        # 绑定 WebRTC 信号
        self.webrtc.connect('on-negotiation-needed', self.on_negotiation_needed)
        self.webrtc.connect('on-ice-candidate', self.send_ice_candidate_message)

        # 总线监控
        bus = self.pipe.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.on_bus_message)

    def on_bus_message(self, bus, message):
        """处理 GStreamer 总线消息"""
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"\n[!!!] GStreamer 错误: {err}")
            print(f"[!!!] 详情: {debug}\n")
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            print(f"[WARN] 警告: {err}")
        elif t == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipe:
                old, new, pending = message.parse_state_changed()
                if new == Gst.State.PLAYING and not self.pipeline_started:
                    self.pipeline_started = True
                    print("[✓] 管道已进入 PLAYING 状态")

    def on_negotiation_needed(self, element):
        """WebRTC 协商信号"""
        print("[!] on-negotiation-needed 触发")
        if self.offer_sent:
            print("[!] Offer 已发送，跳过")
            return
        if loop is None or self.ws is None:
            print("[!] WebSocket 未就绪，稍后手动触发")
            return
        self._create_offer(element)

    def _create_offer(self, element):
        """创建 SDP Offer"""
        print("[*] 创建 SDP Offer...")
        try:
            promise = Gst.Promise.new_with_change_func(self._on_offer_created, element, None)
            element.emit('create-offer', None, promise)
        except Exception as e:
            print(f"[!!!] create-offer 失败: {e}")
            traceback.print_exc()

    def _on_offer_created(self, promise, element, _):
        """Offer 创建完成"""
        try:
            promise.wait()
            reply = promise.get_reply()
            offer = reply.get_value('offer')

            # 设置本地描述
            promise2 = Gst.Promise.new()
            element.emit('set-local-description', offer, promise2)

            text = offer.sdp.as_text()
            msg = {
                'type': 'offer',
                'sdp': text,
                'sender_ip': PUBLIC_IP
            }

            if loop:
                future = asyncio.run_coroutine_threadsafe(self.send_message(msg), loop)
                try:
                    future.result(timeout=5)
                    self.offer_sent = True
                    print("[✓] SDP Offer 已发送！(1920x540)")
                except Exception as e:
                    print(f"[!!!] 发送失败: {e}")

        except Exception as e:
            print(f"[!!!] on_offer_created 异常: {e}")
            traceback.print_exc()

    def send_ice_candidate_message(self, element, mlineindex, candidate):
        """发送 ICE 候选"""
        msg = {
            'type': 'ice',
            'sdpMLineIndex': mlineindex,
            'candidate': candidate
        }
        if loop:
            asyncio.run_coroutine_threadsafe(self.send_message(msg), loop)
            print(f"[ICE] 候选: {candidate[:40]}...")

    async def send_message(self, msg):
        """发送消息"""
        if self.ws is None:
            return False

        from websockets.connection import State
        if hasattr(self.ws, 'state') and self.ws.state != State.OPEN:
            return False

        try:
            await self.ws.send(json.dumps(msg))
            msg_type = msg.get('type', 'unknown')
            print(f"[>>>] 已发送: {msg_type}")
            return True
        except Exception as e:
            print(f"[!!!] 发送异常: {e}")
            return False

    async def connect_signaling(self):
        """连接信令服务器"""
        global running

        print(f"[*] 连接信令服务器: {SIGNALING_SERVER}")

        while running:
            try:
                async with websockets.connect(SIGNALING_SERVER, close_timeout=5) as websocket:
                    self.ws = websocket
                    self.connected = True
                    self.offer_sent = False

                    # 注册
                    await websocket.send(json.dumps({
                        "type": "register",
                        "role": "sender",
                        "name": "Linux-Stereo-Merged",
                        "cameras": {"left": 51004, "right": 51006}
                    }))
                    print("[✓] 已注册")

                    # 启动管道
                    print("[*] 启动管道...")
                    ret = self.pipe.set_state(Gst.State.PLAYING)
                    if ret == Gst.StateChangeReturn.SUCCESS:
                        print("[✓] 管道启动成功")
                    else:
                        print(f"[!!!] 管道启动失败: {ret}")

                    # 等待管道稳定
                    await asyncio.sleep(3)

                    # 手动触发 Offer
                    if not self.offer_sent:
                        print("[!] 手动触发 create-offer...")
                        self._create_offer(self.webrtc)

                    # 消息循环
                    async for message in websocket:
                        if not running:
                            break
                        try:
                            data = json.loads(message)
                            await self.handle_message(data)
                        except Exception as e:
                            print(f"[!] 处理消息失败: {e}")

            except websockets.exceptions.ConnectionClosed:
                if running:
                    print("\n[!] 连接断开，5秒后重连...")
            except Exception as e:
                if running:
                    print(f"[!] 连接异常: {e}")

            if running:
                await asyncio.sleep(5)

    async def handle_message(self, data):
        """处理信令消息"""
        msg_type = data.get('type', '')

        if msg_type == 'answer':
            print("[<] 收到 Answer")
            try:
                res, sdpmsg = GstSdp.SDPMessage.new()
                GstSdp.sdp_message_parse_buffer(bytes(data['sdp'].encode()), sdpmsg)
                answer = GstWebRTC.WebRTCSessionDescription.new(
                    GstWebRTC.WebRTCSDPType.ANSWER, sdpmsg)
                promise = Gst.Promise.new()
                self.webrtc.emit('set-remote-description', answer, promise)
                print("[✓] 远程描述已设置，视频流传输中...")
            except Exception as e:
                print(f"[!] 设置远程描述失败: {e}")

        elif msg_type == 'ice':
            self.webrtc.emit('add-ice-candidate',
                            data['sdpMLineIndex'],
                            data['candidate'])


def start_glib_loop():
    """启动 GLib 主循环"""
    loop = GLib.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt:
        loop.quit()


def signal_handler(sig, frame):
    global running
    print("\n[!] 停止信号...")
    running = False
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


if __name__ == "__main__":
    print("=" * 50)
    print("  RealSense 双目拼接 WebRTC V2")
    print("  (修复 not-negotiated 问题)")
    print("=" * 50)
    print("")

    Gst.init(None)

    glib_thread = threading.Thread(target=start_glib_loop, daemon=True)
    glib_thread.start()
    print("[✓] GLib 主循环已启动\n")

    sender = StereoMergedSenderV2()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    try:
        loop.run_until_complete(sender.connect_signaling())
    except KeyboardInterrupt:
        print("\n[!] 关闭中...")
        if sender.pipe:
            sender.pipe.set_state(Gst.State.NULL)
        sys.exit(0)
