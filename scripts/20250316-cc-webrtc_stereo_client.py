#!/usr/bin/env python3
"""
========================================
  RealSense 双目 WebRTC 发送端
========================================

功能:
  - 双目相机同时推流
  - 左相机: /dev/video4
  - 右相机: /dev/video10
  - 通过 WebRTC + TURN 中继推送到公网

配置:
  - 信令服务器: ws://39.102.113.104:8765
  - TURN 服务器: turn://remote:Wawjxyz3!@39.102.113.104:3478

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

# 左相机管道配置
LEFT_PIPELINE_DESC = f'''
webrtcbin name=sendbin turn-server="{TURN_SERVER}"
v4l2src device=/dev/video4 do-timestamp=true
! video/x-raw,format=YUY2,width=1280,height=720,framerate=30/1
! videoconvert
! video/x-raw,format=I420
! nvh264enc preset=5 zerolatency=true bitrate=4000 rc-mode=cbr
! rtph264pay config-interval=1
! application/x-rtp,media=video,encoding-name=H264,payload=96
! queue max-size-buffers=0
! sendbin.
'''

# 右相机管道配置
RIGHT_PIPELINE_DESC = f'''
webrtcbin name=sendbin turn-server="{TURN_SERVER}"
v4l2src device=/dev/video10 do-timestamp=true
! video/x-raw,format=YUY2,width=1280,height=720,framerate=30/1
! videoconvert
! video/x-raw,format=I420
! nvh264enc preset=5 zerolatency=true bitrate=4000 rc-mode=cbr
! rtph264pay config-interval=1
! application/x-rtp,media=video,encoding-name=H264,payload=96
! queue max-size-buffers=0
! sendbin.
'''

# 全局事件循环
loop = None
running = True


# ============================================
# 单个相机的 WebRTC 发送端类
# ============================================
class CameraSender:
    """单个相机的 WebRTC 发送端"""
    def __init__(self, name, device, pipeline_desc):
        """
        初始化相机发送端

        参数:
            name: 相机名称 ("left" 或 "right")
            device: 设备路径 ("/dev/video4" 或 "/dev/video10")
            pipeline_desc: GStreamer 管道描述
        """
        self.name = name
        self.device = device
        self.pipe = None
        self.webrtc = None
        self.offer_sent = False

        print(f"[*] 初始化 {name.upper()} 相机管道 ({device})...")

        try:
            # 解析 GStreamer 管道
            self.pipe = Gst.parse_launch(pipeline_desc)
            self.webrtc = self.pipe.get_by_name('sendbin')

            # 管道错误监控
            bus = self.pipe.get_bus()
            bus.add_signal_watch()
            bus.connect("message", self.on_bus_message)

            # 绑定 WebRTC 信号
            self.webrtc.connect('on-negotiation-needed', self.on_negotiation_needed)
            self.webrtc.connect('on-ice-candidate', self.send_ice_candidate_message)

            print(f"[✓] {name.upper()} 相机管道已初始化")

        except Exception as e:
            print(f"[!!!] {name.upper()} 相机初始化失败: {e}")
            traceback.print_exc()

    def on_bus_message(self, bus, message):
        """处理 GStreamer 总线消息"""
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"\n[{self.name.upper()}] [!!!] GStreamer 致命错误: {err}")
            print(f"[{self.name.upper()}] [!!!] 详细原因: {debug}\n")
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            print(f"[{self.name.upper()}] [WARN] 管道警告: {err}")

    def on_negotiation_needed(self, element):
        """WebRTC 协商信号处理"""
        print(f"[{self.name.upper()}] [DEBUG] on-negotiation-needed 触发")
        if loop is None:
            print(f"[{self.name.upper()}] [!] asyncio loop 未就绪，跳过")
            return
        if self.offer_sent:
            print(f"[{self.name.upper()}] [!] Offer 已发送，跳过重复触发")
            return
        self._do_create_offer(element)

    def _do_create_offer(self, element):
        """创建 SDP Offer"""
        print(f"[{self.name.upper()}] [*] 开始生成 SDP Offer...")
        try:
            promise = Gst.Promise.new_with_change_func(self.on_offer_created, element, None)
            element.emit('create-offer', None, promise)
        except Exception as e:
            print(f"[{self.name.upper()}] [!!!] create-offer 异常: {e}")
            traceback.print_exc()

    def on_offer_created(self, promise, element, _):
        """Offer 创建完成回调"""
        try:
            promise.wait()
            reply = promise.get_reply()
            if reply is None:
                print(f"[{self.name.upper()}] [!!!] create-offer 返回 None")
                return

            offer = reply.get_value('offer')
            if offer is None:
                print(f"[{self.name.upper()}] [!!!] reply 中没有 offer")
                return

            print(f"[{self.name.upper()}] [✓] Offer 生成成功，设置本地描述...")

            # 设置本地描述
            promise2 = Gst.Promise.new()
            element.emit('set-local-description', offer, promise2)

            # 构造发送消息
            text = offer.sdp.as_text()
            msg = {
                'type': 'offer',
                'camera': self.name,  # 添加相机标识
                'sdp': text,
                'sender_ip': PUBLIC_IP
            }

            # 发送到信令服务器
            if loop is not None:
                future = asyncio.run_coroutine_threadsafe(send_message_to_server(msg), loop)
                try:
                    future.result(timeout=5)
                    self.offer_sent = True
                    print(f"[{self.name.upper()}] [✓] SDP Offer 已发送！")
                except Exception as e:
                    print(f"[{self.name.upper()}] [!!!] Offer 发送失败: {e}")

        except Exception as e:
            print(f"[{self.name.upper()}] [!!!] on_offer_created 异常: {e}")
            traceback.print_exc()

    def send_ice_candidate_message(self, element, mlineindex, candidate):
        """发送 ICE 候选"""
        msg = {
            'type': 'ice',
            'camera': self.name,  # 添加相机标识
            'sdpMLineIndex': mlineindex,
            'candidate': candidate
        }
        if loop is not None:
            asyncio.run_coroutine_threadsafe(send_message_to_server(msg), loop)
            print(f"[{self.name.upper()}] [ICE] 候选已发送: {candidate[:50]}...")

    def start(self):
        """启动相机管道"""
        if self.pipe:
            self.pipe.set_state(Gst.State.PLAYING)
            print(f"[{self.name.upper()}] [✓] 相机管道已启动")

    def stop(self):
        """停止相机管道"""
        if self.pipe:
            self.pipe.set_state(Gst.State.NULL)


# ============================================
# 全局 WebSocket 引用（用于相机发送）
# ============================================
global_ws = None


async def send_message_to_server(msg):
    """发送消息到信令服务器"""
    global global_ws

    if global_ws is None:
        print(f"[!!!] send_message 失败: ws 是 None")
        return False

    msg_type = msg.get('type', 'unknown')
    msg_json = json.dumps(msg)

    try:
        await global_ws.send(msg_json)
        camera = msg.get('camera', 'unknown')
        print(f"[>>>] 消息已发送: type={msg_type}, camera={camera}, size={len(msg_json)} 字节")
        return True
    except Exception as e:
        print(f"[!!!] send_message 异常: {e}")
        return False


# ============================================
# 双目发送端管理器
# ============================================
class StereoSender:
    """双目 WebRTC 发送端管理器"""
    def __init__(self):
        print("[*] 初始化双目 WebRTC 发送端...")

        # 创建两个相机发送端
        self.left = CameraSender("left", "/dev/video4", LEFT_PIPELINE_DESC)
        self.right = CameraSender("right", "/dev/video10", RIGHT_PIPELINE_DESC)

        self.cameras = [self.left, self.right]
        print("[✓] 双目发送端已初始化\n")

    async def connect_signaling(self):
        """连接信令服务器"""
        global running, loop, global_ws

        print(f"[*] 正在连接信令服务器: {SIGNALING_SERVER}")
        print(f"[*] TURN 服务器: {TURN_SERVER}")
        print(f"[*] 公网 IP: {PUBLIC_IP}\n")

        retry_count = 0

        while running:
            try:
                async with websockets.connect(SIGNALING_SERVER, close_timeout=5) as websocket:
                    global_ws = websocket

                    print(f"[✓] 信令服务器连接成功 (尝试 {retry_count + 1})")

                    # 注册为发送端（双目）
                    register_msg = json.dumps({
                        "type": "register",
                        "role": "sender",
                        "ip": PUBLIC_IP,
                        "name": "Linux-Stereo-Sender",
                        "cameras": {"left": 51004, "right": 51006},
                        "stereo": True  # 标识为双目发送端
                    })
                    await websocket.send(register_msg)
                    print(f"[>>>] 注册消息已发送")

                    print("[✓] 已注册为双目发送端\n")

                    # 等待注册完成
                    print("[*] 等待 1 秒确保注册完成...")
                    await asyncio.sleep(1)

                    # 启动两个相机管道
                    print("[*] 启动双目相机推流管道...")
                    for camera in self.cameras:
                        camera.start()
                    print("[✓] 双目相机管道已启动\n")

                    # 等待管道稳定
                    print("[*] 等待 3 秒让管道稳定...")
                    await asyncio.sleep(3)

                    # 检查 Offer 发送状态
                    left_status = "✅" if self.left.offer_sent else "❌"
                    right_status = "✅" if self.right.offer_sent else "❌"

                    print("\n" + "="*50)
                    print(f"  左相机 Offer 状态: {left_status}")
                    print(f"  右相机 Offer 状态: {right_status}")
                    print("="*50 + "\n")

                    # 消息监听循环
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
                global_ws = None
                await asyncio.sleep(5)
                retry_count += 1
                # 重连时重置 Offer 状态
                for camera in self.cameras:
                    camera.offer_sent = False

    async def handle_signaling_message(self, data):
        """处理信令消息"""
        msg_type = data.get('type', '')
        camera_name = data.get('camera', 'left')  # 默认左相机

        # 找到对应的相机
        target_camera = None
        if camera_name == 'left':
            target_camera = self.left
        elif camera_name == 'right':
            target_camera = self.right
        else:
            # 如果没有指定相机，根据消息类型推断
            if msg_type in ['answer', 'ice']:
                print(f"[?] 消息未指定相机，尝试转发给所有相机")
                for cam in self.cameras:
                    await self._handle_camera_message(cam, msg_type, data)
                return

        if target_camera:
            await self._handle_camera_message(target_camera, msg_type, data)

    async def _handle_camera_message(self, camera, msg_type, data):
        """处理单个相机的消息"""
        if msg_type == 'answer':
            print(f"[{camera.name.upper()}] [<] 收到 SDP Answer！")
            try:
                res, sdpmsg = GstSdp.SDPMessage.new()
                GstSdp.sdp_message_parse_buffer(bytes(data['sdp'].encode()), sdpmsg)
                answer = GstWebRTC.WebRTCSessionDescription.new(
                    GstWebRTC.WebRTCSDPType.ANSWER, sdpmsg)
                promise = Gst.Promise.new()
                camera.webrtc.emit('set-remote-description', answer, promise)
                print(f"[{camera.name.upper()}] [✓] 远程描述已设置，WebRTC 握手完成！")
            except Exception as e:
                print(f"[{camera.name.upper()}] [!] 设置远程描述失败: {e}")

        elif msg_type == 'ice':
            print(f"[{camera.name.upper()}] [<] 收到 ICE 候选")
            try:
                camera.webrtc.emit('add-ice-candidate',
                                 data.get('sdpMLineIndex', 0),
                                 data.get('candidate', ''))
            except Exception as e:
                print(f"[{camera.name.upper()}] [!] 添加 ICE 候选失败: {e}")

        elif msg_type != 'heartbeat_ack':
            print(f"[?] 收到未知消息类型: {msg_type}")


# ============================================
# GLib 主循环
# ============================================
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


# ============================================
# 主程序
# ============================================
if __name__ == "__main__":
    print("=" * 50)
    print("  RealSense 双目 WebRTC 发送端")
    print("=" * 50)
    print("")

    # 初始化 GStreamer
    Gst.init(None)

    # 启动 GLib 主循环
    glib_thread = threading.Thread(target=start_glib_loop, daemon=True)
    glib_thread.start()
    print("[✓] GLib 主循环已启动\n")

    # 创建双目发送端
    sender = StereoSender()

    # 创建事件循环
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    try:
        loop.run_until_complete(sender.connect_signaling())
    except KeyboardInterrupt:
        print("\n[!] 正在关闭摄像头并退出...")
        for camera in sender.cameras:
            camera.stop()
        sys.exit(0)
