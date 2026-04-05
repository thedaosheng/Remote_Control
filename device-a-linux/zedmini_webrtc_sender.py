#!/usr/bin/python3
"""
========================================
  ZED Mini 双目 WebRTC 发送端 v7.3
========================================
改动:
  - v7.1: 修复 SDP 缺少 fmtp
  - v7.2: 修复 set-local-description 用原始 offer
  - v7.3: 不主动发 offer，只响应 request_offer，避免新旧 offer 冲突
"""

import asyncio
import websockets
import json
import signal
import sys
import time
import threading
import traceback
import math
import socket
import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
gi.require_version('GstSdp', '1.0')
from gi.repository import Gst, GstWebRTC, GstSdp, GLib

SIGNALING_SERVER = "ws://39.102.113.104:8765"
TURN_SERVER = "turn://remote:Wawjxyz3!@39.102.113.104:3478"
PUBLIC_IP = "36.110.28.59"

ZED_DEVICE = "/dev/video0"

MOTOR_MODE = "dummy"
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200
CAN_INTERFACE = "can0"
CAN_BITRATE = 1000000
POSE_PRINT_EVERY = 30
DAMO_UDP_HOST = "127.0.0.1"
DAMO_UDP_PORT = 9000

loop = None
running = True
sender = None


class MotorController:
    def __init__(self):
        self.enabled = False
        self.pose_count = 0
        self.latest_pos = [0, 0, 0]
        self.latest_quat = [0, 0, 0, 1]

    def open(self):
        self.enabled = True

    def close(self):
        self.enabled = False

    def send_command(self, motor_id, angle_deg, speed=0):
        pass

    def on_pose(self, position, quaternion, timestamp):
        self.pose_count += 1
        self.latest_pos = position
        self.latest_quat = quaternion
        if not self.enabled:
            return
        qx, qy, qz, qw = quaternion
        pitch, roll, yaw = self.quat_to_euler(qx, qy, qz, qw)
        yaw_deg   = max(-90, min(90, math.degrees(yaw)))
        pitch_deg = max(-90, min(90, math.degrees(pitch)))
        roll_deg  = max(-90, min(90, math.degrees(roll)))
        self.send_command(0, yaw_deg)
        self.send_command(1, pitch_deg)
        self.send_command(2, roll_deg)

    @staticmethod
    def quat_to_euler(qx, qy, qz, qw):
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        std_roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            std_pitch = math.copysign(math.pi / 2, sinp)
        else:
            std_pitch = math.asin(sinp)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        std_yaw = math.atan2(siny_cosp, cosy_cosp)
        return std_pitch, std_roll, std_yaw


class DummyMotorController(MotorController):
    def open(self):
        super().open()
        print("[Motor] Dummy 模式")

    def send_command(self, motor_id, angle_deg, speed=0):
        pass


class SerialMotorController(MotorController):
    def __init__(self, port, baudrate):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.ser = None

    def open(self):
        try:
            import serial
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.01)
            super().open()
            print(f"[Motor] 串口已打开: {self.port} @ {self.baudrate}")
        except ImportError:
            print("[!!!] 缺少 pyserial")
        except Exception as e:
            print(f"[!!!] 串口打开失败: {e}")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().close()

    def send_command(self, motor_id, angle_deg, speed=0):
        if not self.ser or not self.ser.is_open:
            return
        angle_raw = max(-9000, min(9000, int(angle_deg * 100)))
        high = (angle_raw >> 8) & 0xFF
        low  = angle_raw & 0xFF
        checksum = (0xAA + motor_id + high + low + speed) & 0xFF
        try:
            self.ser.write(bytes([0xAA, motor_id, high, low, speed, checksum]))
        except Exception as e:
            if self.pose_count % 100 == 0:
                print(f"[Motor] 串口写入失败: {e}")


class CANMotorController(MotorController):
    def __init__(self, interface, bitrate):
        super().__init__()
        self.interface = interface
        self.bitrate = bitrate
        self.bus = None

    def open(self):
        try:
            import can, subprocess
            subprocess.run(['ip','link','set',self.interface,'type','can','bitrate',str(self.bitrate)], check=False)
            subprocess.run(['ip','link','set','up',self.interface], check=False)
            self.bus = can.interface.Bus(channel=self.interface, bustype='socketcan')
            super().open()
            print(f"[Motor] CAN 已打开: {self.interface} @ {self.bitrate}")
        except ImportError:
            print("[!!!] 缺少 python-can")
        except Exception as e:
            print(f"[!!!] CAN 打开失败: {e}")

    def close(self):
        if self.bus:
            self.bus.shutdown()
        super().close()

    def send_command(self, motor_id, angle_deg, speed=0):
        if not self.bus:
            return
        try:
            import can
            angle_raw = max(-9000, min(9000, int(angle_deg * 100)))
            speed_raw = int(speed * 100)
            data = [0x01, (angle_raw>>8)&0xFF, angle_raw&0xFF,
                    (speed_raw>>8)&0xFF, speed_raw&0xFF, 0, 0, 0]
            self.bus.send(can.Message(arbitration_id=0x100+motor_id, data=data, is_extended_id=False))
        except Exception as e:
            if self.pose_count % 100 == 0:
                print(f"[Motor] CAN 发送失败: {e}")


def create_motor_controller():
    if MOTOR_MODE == "serial":
        return SerialMotorController(SERIAL_PORT, SERIAL_BAUD)
    elif MOTOR_MODE == "can":
        return CANMotorController(CAN_INTERFACE, CAN_BITRATE)
    elif MOTOR_MODE == "dummy":
        return DummyMotorController()
    else:
        return MotorController()


class ZedMiniSender:
    def __init__(self, motor_ctrl):
        print("[*] 初始化 ZED Mini GStreamer 管道...")
        print(f"    设备: {ZED_DEVICE}  (ZED Mini WVGA side-by-side)")
        print("    分辨率: 1344x376 @ 60fps (每眼 672x376, 旋转180°)")
        print(f"    电机模式: {MOTOR_MODE}")
        print("")

        self.motor = motor_ctrl
        self.ws          = None
        self.connected   = False
        self.offer_sent  = False
        self.offer_count = 0
        self._session_active = False  # 整个会话只处理一次 request_offer

        self.ping_count      = 0
        self.rtt_samples     = []
        self.pose_recv_count = 0

        self.damo_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.damo_addr = (DAMO_UDP_HOST, DAMO_UDP_PORT)

        self.pipe = None
        self.webrtc = None
        # 管道描述 — 跟旧版 v6 保持一致的简单结构
        self.pipeline_desc = f'''
            webrtcbin name=sendbin
            v4l2src device={ZED_DEVICE} do-timestamp=true
            ! video/x-raw,format=YUY2,width=1344,height=376,framerate=60/1
            ! videoconvert
            ! video/x-raw,format=I420
            ! videoflip method=rotate-180
            ! nvh264enc preset=5 zerolatency=true bitrate=8000 rc-mode=cbr gop-size=30
            ! rtph264pay config-interval=1
            ! application/x-rtp,media=video,encoding-name=H264,payload=96
            ! queue
            ! sendbin.
        '''

        print(f"[✓] 达妙 UDP 转发已就绪 -> {DAMO_UDP_HOST}:{DAMO_UDP_PORT}")
        print("[✓] Sender 已初始化 (管道将在连接后启动)\n")

    def _build_and_start_pipeline(self):
        if self.pipe:
            print("[*] 销毁旧管道...")
            self.pipe.set_state(Gst.State.NULL)
            self.pipe = None
            self.webrtc = None

        print("[*] 创建新管道...")
        try:
            self.pipe = Gst.parse_launch(self.pipeline_desc)
            self.webrtc = self.pipe.get_by_name('sendbin')

            bus = self.pipe.get_bus()
            bus.add_signal_watch()
            bus.connect("message", self.on_bus_message)

            # TURN 配置
            self.webrtc.set_property('turn-server', TURN_SERVER)

            self.webrtc.connect('on-negotiation-needed', self.on_negotiation_needed)
            self.webrtc.connect('on-ice-candidate', self.send_ice_candidate_message)

            self.offer_sent = False

            print("[*] 启动管道...")
            self.pipe.set_state(Gst.State.PLAYING)
            print("[✓] 新管道已启动\n")
        except Exception as e:
            print(f"[!!!] 管道创建失败: {e}")
            traceback.print_exc()

    def on_bus_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"\n[!!!] GStreamer 致命错误: {err}")
            print(f"[!!!] 详细原因: {debug}\n")
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            print(f"[WARN] {err}")

    def on_negotiation_needed(self, element):
        if loop is None or self.offer_sent:
            return
        # 临时改回主动发 offer（用于 HTML receiver 测试）
        print("[*] on-negotiation-needed 触发 → 主动生成 Offer")
        self._do_create_offer(element)

    def _do_create_offer(self, element):
        print("[*] 生成 SDP Offer...")
        try:
            promise = Gst.Promise.new_with_change_func(self.on_offer_created, element, None)
            element.emit('create-offer', None, promise)
        except Exception as e:
            print(f"[!!!] create-offer 异常: {e}")

    def _patch_sdp_fmtp(self, sdp_text):
        if 'a=fmtp:96' in sdp_text:
            return sdp_text
        print("[!] SDP 缺少 fmtp，手动补上 packetization-mode=1")
        patched = sdp_text.replace(
            'a=rtpmap:96 H264/90000\r\n',
            'a=rtpmap:96 H264/90000\r\n'
            'a=fmtp:96 packetization-mode=1;profile-level-id=42c020;level-asymmetry-allowed=1\r\n'
        )
        if 'a=fmtp:96' not in patched:
            patched = sdp_text.replace(
                'a=rtpmap:96 H264/90000\n',
                'a=rtpmap:96 H264/90000\n'
                'a=fmtp:96 packetization-mode=1;profile-level-id=42c020;level-asymmetry-allowed=1\n'
            )
        if 'a=fmtp:96' in patched:
            print("[✓] fmtp 已补上")
        else:
            print("[!] 警告: fmtp 补丁未能匹配")
        return patched

    def on_offer_created(self, promise, element, _):
        """Offer 创建回调 — 跟旧版 v6 一样的简单逻辑"""
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

            # 设置 local description（不等完成，跟旧版一样）
            promise2 = Gst.Promise.new()
            element.emit('set-local-description', offer, promise2)

            text = offer.sdp.as_text()
            text = self._patch_sdp_fmtp(text)

            self.offer_count += 1
            msg = {
                'type': 'offer',
                'sdp': text,
                'sender_ip': PUBLIC_IP,
                'stereo': 'merged',
                'width': 1344,
                'height': 376
            }

            if loop is not None:
                future = asyncio.run_coroutine_threadsafe(self.send_message(msg), loop)
                try:
                    future.result(timeout=5)
                    self.offer_sent = True
                    has_fmtp = 'a=fmtp:96' in text
                    has_sprop = 'sprop-parameter-sets' in text
                    print(f"[✓] SDP Offer #{self.offer_count} 已发送！({len(text)} chars)")
                    print(f"    fmtp={has_fmtp}  sprop={has_sprop}")
                except Exception as e:
                    print(f"[!!!] Offer 发送失败: {e}")

        except Exception as e:
            print(f"[!!!] on_offer_created 异常: {e}")
            traceback.print_exc()

    def send_ice_candidate_message(self, element, mlineindex, candidate):
        print(f"[ICE] 本地候选: {candidate}")
        msg = {'type': 'ice', 'sdpMLineIndex': mlineindex, 'sdpMid': 'video0', 'candidate': candidate}
        if loop is not None:
            asyncio.run_coroutine_threadsafe(self.send_message(msg), loop)

    async def send_message(self, msg):
        if self.ws is None:
            return False
        try:
            await self.ws.send(json.dumps(msg))
            return True
        except Exception as e:
            print(f"[!!!] send_message 异常: {e}")
            return False

    async def ping_loop(self):
        while running and self.ws is not None:
            try:
                self.ping_count += 1
                await self.send_message({
                    'type': 'ping',
                    'sender_time': time.time(),
                    'seq': self.ping_count
                })
            except:
                pass
            await asyncio.sleep(3)

    async def connect_signaling(self):
        global running, loop

        print(f"[*] 信令服务器: {SIGNALING_SERVER}")
        print(f"[*] TURN 服务器: {TURN_SERVER}")
        print(f"[*] 公网 IP: {PUBLIC_IP}")
        print("")

        retry_count = 0

        while running:
            try:
                async with websockets.connect(
                    SIGNALING_SERVER,
                    compression=None,
                    proxy=None,
                    ping_interval=None,
                    close_timeout=5
                ) as websocket:
                    self.ws = websocket
                    self.connected = True
                    self.offer_sent = False
                    self._session_active = False  # 新连接，允许新的 request_offer

                    retry_count += 1
                    print(f"\n[✓] 信令服务器连接成功 (第 {retry_count} 次)")

                    await websocket.send(json.dumps({
                        "type": "register",
                        "role": "sender",
                        "ip": PUBLIC_IP,
                        "name": "ZED-Mini-Sender",
                        "stereo": "merged",
                        "resolution": "1344x376"
                    }))
                    print("[✓] 已注册为发送端")

                    # ★ 只启动管道，不发 offer。等 request_offer 再发。
                    await asyncio.sleep(1)
                    self._build_and_start_pipeline()
                    print("[*] 管道已就绪，等待 receiver 触发 request_offer...\n")

                    ping_task = asyncio.ensure_future(self.ping_loop())

                    async for message in websocket:
                        if not running:
                            break
                        try:
                            data = json.loads(message)
                            await self.handle_signaling_message(data)
                        except json.JSONDecodeError:
                            pass
                        except Exception as e:
                            print(f"[!] 处理消息失败: {e}")

                    ping_task.cancel()

            except websockets.exceptions.ConnectionClosed:
                if running:
                    print("\n[!] 信令断开，3秒后重连...")
            except Exception as e:
                if running:
                    print(f"[!] 连接异常: {e}")

            if running:
                self.ws = None
                self.connected = False
                self.offer_sent = False
                await asyncio.sleep(3)

    async def handle_signaling_message(self, data):
        msg_type = data.get('type', '')

        if msg_type == 'request_offer':
            # 防抖：无论 offer 有没有发成功，15 秒内不重复重建管道
            now = time.time()
            last_rebuild = getattr(self, '_last_rebuild_time', 0)
            if now - last_rebuild < 15:
                print(f"[*] 忽略 request_offer（距上次重建仅 {now - last_rebuild:.1f}s < 15s）")
                return
            self._last_rebuild_time = now
            # 重置状态，准备新会话
            self.offer_sent = False
            print("\n" + "★"*50)
            print("  收到 request_offer — 重建管道，发送新鲜 Offer")
            print("★"*50 + "\n")
            self._build_and_start_pipeline()
            print("[*] 等待编码器就绪 (3秒)...")
            await asyncio.sleep(3)
            if not self.offer_sent and self.webrtc:
                self._do_create_offer(self.webrtc)
            pass  # _session_active 保持 True 直到 WebSocket 重连

        elif msg_type == 'answer':
            print(f"[<] 收到 SDP Answer！({len(data.get('sdp',''))} chars)")
            try:
                res, sdpmsg = GstSdp.SDPMessage.new()
                GstSdp.sdp_message_parse_buffer(bytes(data['sdp'].encode()), sdpmsg)
                answer = GstWebRTC.WebRTCSessionDescription.new(
                    GstWebRTC.WebRTCSDPType.ANSWER, sdpmsg)
                promise = Gst.Promise.new()
                self.webrtc.emit('set-remote-description', answer, promise)
                print("[✓] WebRTC 握手完成！视频流传输中...")
            except Exception as e:
                print(f"[!] 设置远程描述失败: {e}")

        elif msg_type in ('ice', 'candidate'):
            candidate = data.get('candidate', '')
            print(f"[>] 收到 VP ICE 候选: {candidate[:80]}")
            try:
                if self.webrtc:
                    self.webrtc.emit('add-ice-candidate',
                                     data.get('sdpMLineIndex', 0),
                                     candidate)
                    # 检查 webrtcbin ICE 连接状态
                    ice_state = self.webrtc.get_property('ice-connection-state')
                    print(f"    ICE 连接状态: {ice_state}")
            except Exception as e:
                print(f"[!] 添加 ICE 失败: {e}")

        elif msg_type == 'pong':
            now = time.time()
            sender_time   = data.get('sender_time', 0)
            seq           = data.get('seq', 0)
            rtt_ms        = (now - sender_time) * 1000
            self.rtt_samples.append(rtt_ms)
            if len(self.rtt_samples) > 20:
                self.rtt_samples = self.rtt_samples[-20:]
            avg_rtt = sum(self.rtt_samples) / len(self.rtt_samples)
            print(f"[延迟 #{seq}] RTT={rtt_ms:.0f}ms "
                  f"(avg={avg_rtt:.0f} min={min(self.rtt_samples):.0f} max={max(self.rtt_samples):.0f})")

        elif msg_type == 'pose':
            self.pose_recv_count += 1
            pos  = data.get('p', [0, 0, 0])
            quat = data.get('q', [0, 0, 0, 1])
            ts   = data.get('t', 0)

            std_pitch, std_roll, std_yaw = MotorController.quat_to_euler(
                quat[0], quat[1], quat[2], quat[3])
            motor_pitch_deg = math.degrees(std_roll)
            motor_yaw_deg   = math.degrees(std_pitch)

            udp_payload = json.dumps({
                "pitch": round(motor_pitch_deg, 2),
                "yaw":   round(motor_yaw_deg, 2),
                "t":     ts
            })
            try:
                self.damo_sock.sendto(udp_payload.encode('utf-8'), self.damo_addr)
            except Exception:
                pass

            if self.pose_recv_count % POSE_PRINT_EVERY == 0:
                print(f"[Pose #{self.pose_recv_count}] "
                      f"PITCH(点头)={motor_pitch_deg:+7.2f}°  "
                      f"YAW(转头)={motor_yaw_deg:+7.2f}°")

            self.motor.on_pose(pos, quat, ts)


def start_glib_loop():
    glib_loop = GLib.MainLoop()
    try:
        glib_loop.run()
    except KeyboardInterrupt:
        glib_loop.quit()


def signal_handler(sig, frame):
    global running, sender
    print("\n[!] 收到停止信号...")
    running = False
    if sender is not None:
        if hasattr(sender, 'motor'):
            try:
                sender.motor.close()
            except:
                pass
        if hasattr(sender, 'pipe') and sender.pipe:
            try:
                sender.pipe.set_state(Gst.State.NULL)
                print("[✓] 管道已关闭")
            except:
                pass
    import os
    os._exit(0)


signal.signal(signal.SIGINT, signal_handler)


if __name__ == "__main__":
    print("=" * 50)
    print("  ZED Mini 双目 WebRTC 发送端 v7.3")
    print("  1344x376 side-by-side | 60fps | 8000kbps")
    print("  ★ 只响应 request_offer，不主动发 offer")
    print("=" * 50)
    print("")

    Gst.init(None)

    glib_thread = threading.Thread(target=start_glib_loop, daemon=True)
    glib_thread.start()
    print("[✓] GLib 主循环已启动\n")

    motor = create_motor_controller()
    motor.open()

    sender = ZedMiniSender(motor)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    try:
        loop.run_until_complete(sender.connect_signaling())
    except KeyboardInterrupt:
        print("\n[!] 退出...")
        motor.close()
        if sender.pipe:
            sender.pipe.set_state(Gst.State.NULL)
        sys.exit(0)
