#!/usr/bin/python3
"""
========================================
  ZED Mini aiortc WebRTC 发送端 v1.0
========================================
用 aiortc（纯 Python WebRTC）替代 GStreamer webrtcbin，
解决 GStreamer 1.20 的 DTLS/SRTP 兼容性问题。

架构：
  GStreamer: v4l2src → videoconvert → videoflip → appsink（输出原始 I420 帧）
  aiortc:   自定义 VideoTrack → H264 编码 → DTLS-SRTP → ICE → 发送到 VP

信令协议与 VP 端完全兼容，VP 代码无需修改。
"""

import asyncio
import json
import signal
import sys
import time
import math
import socket
import threading
import traceback
import fractions

import numpy as np
import websockets

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription,
    RTCIceCandidate,
    RTCConfiguration,
    RTCIceServer,
    MediaStreamTrack,
)
from av import VideoFrame

# ★ 修复1：强制使用 SRTP_AES128_CM_SHA1_80
# LiveKit WebRTC (VP端) 不支持 AEAD_AES_256_GCM
import aiortc.rtcdtlstransport as _dtls_mod
_dtls_mod.SRTP_PROFILES[:] = [_dtls_mod.SRTP_AES128_CM_SHA1_80]
print("[★] SRTP profile: AES128_CM_SHA1_80")

# ★ 修复2：排除 Clash TUN 虚拟网卡 (198.18.x.x) 的 ICE 候选
# 如果 VP 连到 198.18.0.1，DTLS 包会被 Clash 代理修改导致失败
import aioice.ice as _ice_mod
_original_get_host = _ice_mod.get_host_addresses
def _filtered_get_host(use_ipv4, use_ipv6):
    addrs = _original_get_host(use_ipv4, use_ipv6)
    filtered = [a for a in addrs if not a.startswith("198.18.") and not a.startswith("172.17.")]
    excluded = set(addrs) - set(filtered)
    if excluded:
        print(f"[★] ICE 排除地址: {excluded}")
    return filtered
_ice_mod.get_host_addresses = _filtered_get_host
print("[★] ICE 已排除 TUN/Docker 地址")

# ======================== 配置 ========================
SIGNALING_SERVER = "ws://39.102.113.104:8765"
TURN_SERVER_URL = "turn:39.102.113.104:3478"
TURN_USERNAME = "remote"
TURN_PASSWORD = "Wawjxyz3!"
STUN_SERVER = "stun:stun.l.google.com:19302"
PUBLIC_IP = "36.110.28.59"

ZED_DEVICE = "/dev/video0"
WIDTH = 1344
HEIGHT = 376
FPS = 60
# 测试：缩放到标准分辨率排除 VP 解码器兼容性问题
OUTPUT_WIDTH = 640
OUTPUT_HEIGHT = 480

POSE_PRINT_EVERY = 30
DAMO_UDP_HOST = "127.0.0.1"
DAMO_UDP_PORT = 9000

# ======================== 全局状态 ========================
running = True


# ======================== GStreamer 摄像头采集 ========================
class GStreamerCapture:
    """用 GStreamer 从 ZED Mini 采集原始 I420 帧"""

    def __init__(self, device=ZED_DEVICE, width=WIDTH, height=HEIGHT, fps=FPS):
        Gst.init(None)
        self.width = width
        self.height = height
        self.fps = fps
        self._latest_frame = None
        self._frame_event = threading.Event()
        self._lock = threading.Lock()

        # 管道：摄像头 → 色彩转换 → 翻转 → 缩放到输出分辨率 → appsink
        out_w = OUTPUT_WIDTH
        out_h = OUTPUT_HEIGHT
        self.width = out_w
        self.height = out_h
        pipeline_str = (
            f'v4l2src device={device} do-timestamp=true '
            f'! video/x-raw,format=YUY2,width={width},height={height},framerate={fps}/1 '
            '! videoconvert ! video/x-raw,format=I420 '
            '! videoflip method=rotate-180 '
            f'! videoscale ! video/x-raw,width={out_w},height={out_h} '
            '! appsink name=sink emit-signals=true max-buffers=2 drop=true sync=false'
        )
        print(f"[GStreamer] 原始: {width}x{height} → 输出: {out_w}x{out_h}")

        self.pipe = Gst.parse_launch(pipeline_str)
        self.sink = self.pipe.get_by_name('sink')
        self.sink.connect('new-sample', self._on_new_sample)

        print(f"[GStreamer] 管道: {device} → I420 {width}x{height}@{fps}fps → appsink")

    def _on_new_sample(self, sink):
        """GLib 线程回调：新帧到达"""
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        ok, map_info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK

        # I420 格式：Y (w*h) + U (w/2 * h/2) + V (w/2 * h/2)
        data = bytes(map_info.data)
        buf.unmap(map_info)

        # 构建 av.VideoFrame
        y_size = self.width * self.height
        u_size = (self.width // 2) * (self.height // 2)

        frame = VideoFrame(width=self.width, height=self.height, format='yuv420p')
        frame.planes[0].update(data[:y_size])
        frame.planes[1].update(data[y_size:y_size + u_size])
        frame.planes[2].update(data[y_size + u_size:y_size + 2 * u_size])

        with self._lock:
            self._latest_frame = frame
        self._frame_event.set()

        return Gst.FlowReturn.OK

    def get_frame(self, timeout=0.1):
        """阻塞等待下一帧（跨线程安全）"""
        self._frame_event.wait(timeout=timeout)
        self._frame_event.clear()
        with self._lock:
            f = self._latest_frame
            self._latest_frame = None
        return f

    def start(self):
        self.pipe.set_state(Gst.State.PLAYING)
        print("[GStreamer] 摄像头已启动")

    def stop(self):
        self.pipe.set_state(Gst.State.NULL)
        print("[GStreamer] 摄像头已停止")


# ======================== 自定义视频轨道 ========================
class ZedVideoTrack(MediaStreamTrack):
    """从 GStreamer appsink 读取帧，喂给 aiortc 编码和发送"""
    kind = "video"

    def __init__(self, capture: GStreamerCapture):
        super().__init__()
        self.capture = capture
        self._start_time = None
        self._frame_count = 0

    async def recv(self):
        """aiortc 调用此方法获取下一帧视频"""
        # 在 asyncio 线程中等待 GStreamer 线程产生的帧
        loop = asyncio.get_event_loop()
        frame = await loop.run_in_executor(None, self.capture.get_frame, 0.05)

        if frame is None:
            # 没有新帧，返回黑帧避免阻塞
            frame = VideoFrame(width=self.capture.width, height=self.capture.height, format='yuv420p')

        # 设置时间戳
        if self._start_time is None:
            self._start_time = time.time()
        self._frame_count += 1

        # pts 单位：1/90000 秒（RTP 时钟频率）
        elapsed = time.time() - self._start_time
        frame.pts = int(elapsed * 90000)
        frame.time_base = fractions.Fraction(1, 90000)

        # 每 300 帧打印一次状态
        if self._frame_count % 300 == 1:
            print(f"[VideoTrack] 帧 #{self._frame_count}: {frame.width}x{frame.height} "
                  f"format={frame.format.name} pts={frame.pts} elapsed={elapsed:.1f}s")

        return frame


# ======================== 四元数转欧拉角 ========================
def quat_to_euler(qx, qy, qz, qw):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return pitch, roll, yaw


# ======================== 主发送器 ========================
class AiortcSender:
    def __init__(self, capture: GStreamerCapture):
        self.capture = capture
        self.ws = None
        self.pc = None
        self.connected = False

        # 计数器
        self.ping_count = 0
        self.rtt_samples = []
        self.pose_recv_count = 0

        # 达妙 UDP 转发
        self.damo_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.damo_addr = (DAMO_UDP_HOST, DAMO_UDP_PORT)

        # 状态
        self._last_rebuild_time = 0

    def _make_rtc_config(self):
        """创建 WebRTC 配置（ICE 服务器）"""
        return RTCConfiguration(iceServers=[
            RTCIceServer(urls=[STUN_SERVER]),
            RTCIceServer(
                urls=[TURN_SERVER_URL, f"{TURN_SERVER_URL}?transport=tcp"],
                username=TURN_USERNAME,
                credential=TURN_PASSWORD,
            ),
        ])

    async def _create_pc_and_offer(self):
        """创建新的 PeerConnection 并生成 offer"""
        # 关闭旧连接
        if self.pc:
            await self.pc.close()
            self.pc = None

        print("\n[WebRTC] 创建新的 PeerConnection...")

        # 创建 PeerConnection
        self.pc = RTCPeerConnection(configuration=self._make_rtc_config())

        # 监听 ICE 连接状态
        @self.pc.on("iceconnectionstatechange")
        async def on_ice_state():
            state = self.pc.iceConnectionState
            print(f"[WebRTC] ICE 状态: {state}")
            if state == "failed":
                print("[WebRTC] ICE 失败，关闭连接")
                await self.pc.close()

        # 监听 ICE 候选
        @self.pc.on("icecandidate")
        async def on_ice_candidate(candidate):
            if candidate and self.ws:
                msg = {
                    "type": "ice",
                    "sdpMLineIndex": candidate.sdpMLineIndex,
                    "sdpMid": candidate.sdpMid or "video0",
                    "candidate": candidate.candidate,
                }
                await self._send_ws(msg)

        # 添加视频轨道
        video_track = ZedVideoTrack(self.capture)
        sender = self.pc.addTrack(video_track)

        # 强制 H264 Baseline (42e01f) — visionOS 有硬件解码器
        # VP8 在 visionOS 上可能没有解码器
        for transceiver in self.pc.getTransceivers():
            codecs = transceiver.receiver.getCapabilities("video").codecs
            h264 = [c for c in codecs
                    if "h264" in c.mimeType.lower()
                    and (c.parameters or {}).get("profile-level-id") == "42e01f"]
            rtx = [c for c in codecs if "rtx" in c.mimeType.lower()]
            if h264:
                transceiver.setCodecPreferences(h264 + rtx)
                print(f"[WebRTC] 编码: H264 42e01f ({len(h264)} + {len(rtx)} rtx)")
            else:
                print("[WebRTC] 警告: 没找到 H264 42e01f，使用默认")

        # 创建 offer
        offer = await self.pc.createOffer()
        await self.pc.setLocalDescription(offer)

        print(f"[WebRTC] Offer 已创建 ({len(offer.sdp)} chars)")
        return offer.sdp

    async def _handle_answer(self, sdp):
        """处理 VP 返回的 answer"""
        if not self.pc:
            print("[!] 没有 PeerConnection，忽略 answer")
            return
        answer = RTCSessionDescription(sdp=sdp, type="answer")
        await self.pc.setRemoteDescription(answer)
        print("[✓] Answer 已设置，WebRTC 握手完成！")

    async def _handle_ice_candidate(self, data):
        """处理 VP 发来的 ICE 候选"""
        if not self.pc:
            return
        candidate_str = data.get("candidate", "")
        sdp_mid = data.get("sdpMid", "")
        sdp_mline_index = data.get("sdpMLineIndex", 0)

        if not candidate_str:
            return

        try:
            candidate = RTCIceCandidate(
                component=1,
                foundation="",
                ip="",
                port=0,
                priority=0,
                protocol="udp",
                type="host",
                sdpMid=sdp_mid,
                sdpMLineIndex=sdp_mline_index,
            )
            # aiortc 的 addIceCandidate 接受原始 candidate 字符串
            # 需要通过 SDP 格式解析
            from aiortc.sdp import candidate_from_sdp
            candidate = candidate_from_sdp(candidate_str.split(":", 1)[-1] if ":" in candidate_str else candidate_str)
            candidate.sdpMid = sdp_mid
            candidate.sdpMLineIndex = sdp_mline_index
            await self.pc.addIceCandidate(candidate)
        except Exception as e:
            # aiortc 有时解析外部 ICE 候选会失败，可以忽略
            pass

    async def _send_ws(self, msg):
        """发送 WebSocket 消息"""
        if self.ws:
            try:
                await self.ws.send(json.dumps(msg))
                return True
            except Exception:
                return False
        return False

    async def _ping_loop(self):
        """定时 ping"""
        while running and self.ws:
            try:
                self.ping_count += 1
                await self._send_ws({
                    "type": "ping",
                    "sender_time": time.time(),
                    "seq": self.ping_count,
                })
            except Exception:
                pass
            await asyncio.sleep(3)

    async def _handle_message(self, data):
        """处理信令消息"""
        msg_type = data.get("type", "")

        if msg_type == "request_offer":
            # 防抖：15 秒内不重复
            now = time.time()
            if now - self._last_rebuild_time < 15:
                print(f"[*] 忽略 request_offer（{now - self._last_rebuild_time:.1f}s < 15s）")
                return
            self._last_rebuild_time = now

            print("\n" + "★" * 50)
            print("  收到 request_offer — 创建 offer")
            print("★" * 50 + "\n")

            sdp = await self._create_pc_and_offer()
            await self._send_ws({
                "type": "offer",
                "sdp": sdp,
                "sender_ip": PUBLIC_IP,
                "stereo": "merged",
                "width": WIDTH,
                "height": HEIGHT,
            })
            print("[✓] Offer 已发送到信令服务器")

        elif msg_type == "answer":
            print(f"[<] 收到 Answer ({len(data.get('sdp', ''))} chars)")
            await self._handle_answer(data["sdp"])

        elif msg_type in ("ice", "candidate"):
            await self._handle_ice_candidate(data)

        elif msg_type == "pong":
            now = time.time()
            rtt = (now - data.get("sender_time", 0)) * 1000
            self.rtt_samples.append(rtt)
            if len(self.rtt_samples) > 20:
                self.rtt_samples = self.rtt_samples[-20:]
            avg = sum(self.rtt_samples) / len(self.rtt_samples)
            print(f"[延迟 #{data.get('seq', 0)}] RTT={rtt:.0f}ms (avg={avg:.0f}ms)")

        elif msg_type == "pose":
            self.pose_recv_count += 1
            pos = data.get("p", [0, 0, 0])
            quat = data.get("q", [0, 0, 0, 1])
            ts = data.get("t", 0)

            pitch, roll, yaw = quat_to_euler(quat[0], quat[1], quat[2], quat[3])
            motor_pitch = math.degrees(roll)
            motor_yaw = math.degrees(pitch)

            # UDP 转发给达妙
            payload = json.dumps({"pitch": round(motor_pitch, 2), "yaw": round(motor_yaw, 2), "t": ts})
            try:
                self.damo_sock.sendto(payload.encode(), self.damo_addr)
            except Exception:
                pass

            if self.pose_recv_count % POSE_PRINT_EVERY == 0:
                print(f"[Pose #{self.pose_recv_count}] PITCH={motor_pitch:+.2f}° YAW={motor_yaw:+.2f}°")

    async def run(self):
        """主循环：连接信令服务器，处理消息"""
        global running
        retry = 0

        while running:
            try:
                async with websockets.connect(
                    SIGNALING_SERVER,
                    compression=None,
                    ping_interval=None,
                    close_timeout=5,
                ) as ws:
                    self.ws = ws
                    self.connected = True
                    self._last_rebuild_time = 0
                    retry += 1

                    print(f"\n[✓] 信令连接成功（第 {retry} 次）")

                    # 注册为 sender
                    await self._send_ws({
                        "type": "register",
                        "role": "sender",
                        "ip": PUBLIC_IP,
                        "name": "ZED-Mini-aiortc",
                        "stereo": "merged",
                        "resolution": f"{WIDTH}x{HEIGHT}",
                    })
                    print("[✓] 已注册为 sender\n")

                    # 启动 ping
                    ping_task = asyncio.ensure_future(self._ping_loop())

                    # 接收消息
                    async for message in ws:
                        if not running:
                            break
                        try:
                            data = json.loads(message)
                            await self._handle_message(data)
                        except json.JSONDecodeError:
                            pass
                        except Exception as e:
                            print(f"[!] 消息处理错误: {e}")
                            traceback.print_exc()

                    ping_task.cancel()

            except websockets.exceptions.ConnectionClosed:
                if running:
                    print("\n[!] 信令断开，3 秒后重连...")
            except Exception as e:
                if running:
                    print(f"[!] 连接异常: {e}")

            if running:
                self.ws = None
                self.connected = False
                if self.pc:
                    await self.pc.close()
                    self.pc = None
                await asyncio.sleep(3)


# ======================== GLib 主循环 ========================
def start_glib_loop():
    """GLib 主循环（GStreamer 需要）"""
    glib_loop = GLib.MainLoop()
    try:
        glib_loop.run()
    except KeyboardInterrupt:
        glib_loop.quit()


# ======================== 入口 ========================
def main():
    global running

    print("=" * 50)
    print("  ZED Mini aiortc WebRTC 发送端 v1.0")
    print(f"  {WIDTH}x{HEIGHT} side-by-side | {FPS}fps")
    print("  aiortc 处理 WebRTC (DTLS/SRTP/ICE)")
    print("  GStreamer 仅负责摄像头采集")
    print("=" * 50)
    print()

    # 启动 GLib 主循环（后台线程，GStreamer 需要）
    glib_thread = threading.Thread(target=start_glib_loop, daemon=True)
    glib_thread.start()
    print("[✓] GLib 主循环已启动")

    # 初始化摄像头
    capture = GStreamerCapture()
    capture.start()

    # 创建 sender
    sender = AiortcSender(capture)

    # 信号处理
    def on_signal(sig, frame):
        global running
        print("\n[!] 收到停止信号")
        running = False

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    # 运行 asyncio 主循环
    loop = asyncio.new_event_loop()
    try:
        loop.run_until_complete(sender.run())
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        capture.stop()
        if sender.pc:
            loop.run_until_complete(sender.pc.close())
        loop.close()
        print("[✓] 已退出")


if __name__ == "__main__":
    import logging
    # 开启 DTLS 调试日志
    logging.basicConfig(level=logging.DEBUG, format='%(name)s %(levelname)s %(message)s')
    logging.getLogger('aioice').setLevel(logging.WARNING)
    logging.getLogger('aiortc.rtcrtpsender').setLevel(logging.WARNING)
    logging.getLogger('aiortc.rtcrtpreceiver').setLevel(logging.WARNING)
    logging.getLogger('aiortc.codecs').setLevel(logging.WARNING)
    logging.getLogger('aiortc.rtcdtlstransport').setLevel(logging.DEBUG)
    logging.getLogger('aiortc.rtcicetransport').setLevel(logging.INFO)
    main()
