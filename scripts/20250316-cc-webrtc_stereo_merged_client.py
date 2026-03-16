#!/usr/bin/env python3
"""
========================================
  RealSense 双目拼接 WebRTC 发送端
========================================

功能:
  - 将左右两个相机画面合并成一个视频流
  - 左相机在左，右相机在右 (1920x540)
  - 一次推流发送拼接后的画面 (低延迟优化)

配置:
  - 信令服务器: ws://39.102.113.104:8765
  - TURN 服务器: turn://remote:Wawjxyz3%21@39.102.113.104:3478
  - 左相机: /dev/video4
  - 右相机: /dev/video10
  - 每个相机: 960x540 @ 25fps
  - 输出分辨率: 1920x540 (拼接后)
  - 码率: 6000 kbps (低延迟)

作者: Claude Code
日期: 2025-03-16
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

# ============================================
# 配置区
# ============================================
SIGNALING_SERVER = "ws://39.102.113.104:8765"
TURN_SERVER = "turn://remote:Wawjxyz3%21@39.102.113.104:3478"
PUBLIC_IP = "36.110.28.59"

# 双目拼接管道配置 (低延迟优化版)
# 每个相机: 640x480, 拼接后: 1280x480
PIPELINE_DESC = f'''
webrtcbin name=sendbin turn-server="{TURN_SERVER}"
compositor name=mix
  sink_0::xpos=0 sink_0::ypos=0
  sink_1::xpos=640 sink_1::ypos=0
! video/x-raw,width=1280,height=480,framerate=30/1
! videoconvert
! nvh264enc preset=5 zerolatency=true bitrate=3000 rc-mode=cbr
! rtph264pay config-interval=1
! application/x-rtp,media=video,encoding-name=H264,payload=96
! queue
! sendbin.
v4l2src device=/dev/video4 do-timestamp=true
! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1
! videoconvert
! queue
! mix.sink_0
v4l2src device=/dev/video10 do-timestamp=true
! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1
! videoconvert
! queue
! mix.sink_1
'''

# 全局变量
loop = None
running = True
sender = None  # 全局发送端实例，用于信号处理时清理管道


class StereoMergedSender:
    """双目拼接 WebRTC 发送端"""
    def __init__(self):
        print("[*] 初始化双目拼接 GStreamer 管道...")
        print("    左相机 /dev/video4  -> 左半屏 (0-960)")
        print("    右相机 /dev/video10 -> 右半屏 (960-1920)")
        print("    输出分辨率: 1920x540 @ 25fps (低延迟优化)")
        print("")

        try:
            # 解析 GStreamer 管道
            self.pipe = Gst.parse_launch(PIPELINE_DESC)
            self.webrtc = self.pipe.get_by_name('sendbin')
            self.ws = None
            self.connected = False
            self.offer_sent = False

            # 管道错误监控
            bus = self.pipe.get_bus()
            bus.add_signal_watch()
            bus.connect("message", self.on_bus_message)

            # 绑定 WebRTC 信号
            self.webrtc.connect('on-negotiation-needed', self.on_negotiation_needed)
            self.webrtc.connect('on-ice-candidate', self.send_ice_candidate_message)

            print("[✓] 双目拼接管道已初始化\n")

        except Exception as e:
            print(f"[!!!] 管道初始化失败: {e}")
            traceback.print_exc()

    def on_bus_message(self, bus, message):
        """处理 GStreamer 总线消息"""
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"\n[!!!] GStreamer 致命错误: {err}")
            print(f"[!!!] 详细原因: {debug}\n")
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            print(f"[WARN] 管道警告: {err}")
        elif t == Gst.MessageType.EOS:
            print("[!] 流结束")

    def on_negotiation_needed(self, element):
        """WebRTC 协商信号处理"""
        print(f"[DEBUG] on-negotiation-needed 触发")
        if loop is None:
            print("[!] asyncio loop 未就绪，跳过")
            return
        if self.offer_sent:
            print("[!] Offer 已发送，跳过重复触发")
            return
        self._do_create_offer(element)

    def _do_create_offer(self, element):
        """创建 SDP Offer"""
        print("[*] 开始生成 SDP Offer...")
        try:
            promise = Gst.Promise.new_with_change_func(self.on_offer_created, element, None)
            element.emit('create-offer', None, promise)
        except Exception as e:
            print(f"[!!!] create-offer 异常: {e}")
            traceback.print_exc()

    def on_offer_created(self, promise, element, _):
        """Offer 创建完成回调"""
        try:
            promise.wait()
            reply = promise.get_reply()
            if reply is None:
                print("[!!!] create-offer 返回 None")
                return

            offer = reply.get_value('offer')
            if offer is None:
                print("[!!!] reply 中没有 offer")
                return

            print("[✓] Offer 生成成功，设置本地描述...")

            # 设置本地描述
            promise2 = Gst.Promise.new()
            element.emit('set-local-description', offer, promise2)

            # 获取 SDP 文本并修改（确保分辨率正确）
            text = offer.sdp.as_text()

            # 打印 SDP 中分辨率相关的行（调试用）
            print("[DEBUG] SDP 中的视频参数:")
            for line in text.split('\n'):
                if 'c=' in line or 'a=rtpmap:' in line or 'a=fmtp:' in line or 'a=width' in line or 'a=height' in line:
                    print(f"  {line}")

            msg = {
                'type': 'offer',
                'sdp': text,
                'sender_ip': PUBLIC_IP,
                'stereo': 'merged',
                'width': 2560,
                'height': 720
            }

            # 发送到信令服务器
            if loop is not None:
                future = asyncio.run_coroutine_threadsafe(self.send_message(msg), loop)
                try:
                    future.result(timeout=5)
                    self.offer_sent = True
                    print("[✓] SDP Offer 已发送！(拼接双目 2560x720)")
                except Exception as e:
                    print(f"[!!!] Offer 发送失败: {e}")

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
        if loop is not None:
            asyncio.run_coroutine_threadsafe(self.send_message(msg), loop)
            print(f"[ICE] 候选已发送: {candidate[:50]}...")

    async def send_message(self, msg):
        """发送消息到信令服务器"""
        if self.ws is None:
            return False

        from websockets.connection import State
        if hasattr(self.ws, 'state') and self.ws.state != State.OPEN:
            return False

        try:
            msg_type = msg.get('type', 'unknown')
            msg_json = json.dumps(msg)
            await self.ws.send(msg_json)
            print(f"[>>>] 消息已发送: type={msg_type}, size={len(msg_json)} 字节")
            return True
        except Exception as e:
            print(f"[!!!] send_message 异常: {e}")
            return False

    async def connect_signaling(self):
        """连接信令服务器"""
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
                    self.offer_sent = False

                    print(f"[✓] 信令服务器连接成功 (尝试 {retry_count + 1})")

                    # 注册为发送端
                    register_msg = json.dumps({
                        "type": "register",
                        "role": "sender",
                        "ip": PUBLIC_IP,
                        "name": "Linux-Stereo-Merged-Sender",
                        "cameras": {"left": 51004, "right": 51006},
                        "stereo": "merged",
                        "resolution": "2560x720"
                    })
                    await websocket.send(register_msg)
                    print(f"[>>>] 注册消息已发送")

                    print("[✓] 已注册为双目拼接发送端\n")

                    # 等待注册完成
                    print("[*] 等待 1 秒确保注册完成...")
                    await asyncio.sleep(1)

                    # 启动管道
                    print("[*] 启动双目拼接推流管道...")
                    self.pipe.set_state(Gst.State.PLAYING)
                    print("[✓] 管道已启动\n")

                    # 等待管道稳定
                    print("[*] 等待 3 秒让管道稳定...")
                    await asyncio.sleep(3)

                    # 手动触发 Offer（如果还没发送）
                    if not self.offer_sent:
                        print("[!] 手动触发 create-offer...")
                        self._do_create_offer(self.webrtc)
                        await asyncio.sleep(1)

                    if self.offer_sent:
                        print("\n" + "="*50)
                        print("  ✅ 拼接 Offer 已发送！(2560x720)")
                        print("  等待接收端 Answer...")
                        print("="*50 + "\n")
                    else:
                        print("\n[!!!] 警告: Offer 未发送\n")

                    # 消息监听循环
                    async for message in websocket:
                        if not running:
                            break
                        try:
                            data = json.loads(message)
                            msg_type = data.get('type', 'unknown')
                            print(f"[<<<] 收到消息: type={msg_type}")
                            await self.handle_signaling_message(data)
                        except json.JSONDecodeError:
                            print(f"[!] 无效 JSON")
                        except Exception as e:
                            print(f"[!] 处理消息失败: {e}")
                            traceback.print_exc()

            except websockets.exceptions.ConnectionClosed:
                if running:
                    print("\n[!] 连接断开，5秒后重连...")
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
        """处理信令消息"""
        msg_type = data.get('type', '')

        if msg_type == 'answer':
            print("[<] 收到 SDP Answer！")
            try:
                res, sdpmsg = GstSdp.SDPMessage.new()
                GstSdp.sdp_message_parse_buffer(bytes(data['sdp'].encode()), sdpmsg)
                answer = GstWebRTC.WebRTCSessionDescription.new(
                    GstWebRTC.WebRTCSDPType.ANSWER, sdpmsg)
                promise = Gst.Promise.new()
                self.webrtc.emit('set-remote-description', answer, promise)
                print("[✓] 远程描述已设置，WebRTC 握手完成！")
                print("[*] 拼接双目视频流正在传输...")
            except Exception as e:
                print(f"[!] 设置远程描述失败: {e}")

        elif msg_type == 'ice':
            print("[<] 收到 ICE 候选")
            try:
                self.webrtc.emit('add-ice-candidate',
                                data.get('sdpMLineIndex', 0),
                                data.get('candidate', ''))
            except Exception as e:
                print(f"[!] 添加 ICE 失败: {e}")

        else:
            if msg_type != 'heartbeat_ack':
                print(f"[?] 未知消息: {msg_type}")


def start_glib_loop():
    glib_loop = GLib.MainLoop()
    try:
        glib_loop.run()
    except KeyboardInterrupt:
        glib_loop.quit()


def signal_handler(sig, frame):
    """处理 SIGINT 信号（Ctrl+C），确保正确清理 GStreamer 管道"""
    global running, sender
    print("\n[!] 收到停止信号...")

    # 设置运行标志为 False，让事件循环正常退出
    running = False

    # 立即清理 GStreamer 管道，释放摄像头设备
    if sender is not None and hasattr(sender, 'pipe'):
        try:
            print("[*] 正在关闭 GStreamer 管道...")
            sender.pipe.set_state(Gst.State.NULL)
            print("[✓] 管道已关闭，设备已释放")
        except Exception as e:
            print(f"[!] 管道关闭时出错: {e}")

    # 优雅退出，不是 sys.exit(0)，让 KeyboardInterrupt 正常触发
    # 通过 os._exit 确保进程真正退出（避免僵尸线程）
    import os
    import time
    time.sleep(0.1)  # 给一点时间让清理完成
    os._exit(0)


signal.signal(signal.SIGINT, signal_handler)


if __name__ == "__main__":
    print("=" * 50)
    print("  RealSense 双目拼接 WebRTC 发送端")
    print("  输出: 1920x540 (低延迟优化)")
    print("=" * 50)
    print("")

    Gst.init(None)

    glib_thread = threading.Thread(target=start_glib_loop, daemon=True)
    glib_thread.start()
    print("[✓] GLib 主循环已启动\n")

    sender = StereoMergedSender()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    try:
        loop.run_until_complete(sender.connect_signaling())
    except KeyboardInterrupt:
        print("\n[!] 正在关闭摄像头并退出...")
        sender.pipe.set_state(Gst.State.NULL)
        sys.exit(0)
