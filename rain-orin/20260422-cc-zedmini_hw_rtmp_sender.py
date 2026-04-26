#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
====================================================================
  ZED Mini 硬编 RTMP Sender v1.0 (Jetson NVENC)
====================================================================

架构:
  GStreamer in-process:
    v4l2src (YUY2 1344x376@100fps)
      → nvvidconv (Jetson VPU 硬件色彩转换, 零 CPU)
      → NVMM NV12
      → nvv4l2h264enc (Jetson NVENC 硬编 H264, 零 CPU)
      → h264parse → flvmux → rtmpsink
      → rtmp://127.0.0.1:1935/x/<stream_key>

  LiveKit Ingress (另一 docker 容器):
    接 RTMP → 转 WebRTC → 注入 LiveKit room (virtual participant)

  本进程 LiveKit Python SDK:
    ws://127.0.0.1:7880 订阅 data_channel → 收 VP 的 pose JSON
    → 解析四元数 → UDP 转发给达妙引擎 127.0.0.1:9000

为什么要两个 participant:
  - "zed-mini-sender-hw" (virtual, Ingress 创建) → 推视频 track
  - "zed-mini-sender-data" (本进程) → 订阅 data_channel

这样视频走真硬编路径不经 Python SDK 软编, CPU 降 ~140 percentage points.

环境变量:
  LIVEKIT_URL        默认 ws://127.0.0.1:7880
  RTMP_HOST          默认 127.0.0.1 (Ingress 返回 url 兜底用)
  H264_BITRATE       默认 8000000 (8 Mbps)
  CAM_FPS            默认 100

启动前确认:
  cd ~/Telep/livekit-local && docker compose ps
  ss -tlnp | grep -E "7880|1935"

用法:
  /usr/bin/python3 20260422-cc-zedmini_hw_rtmp_sender.py
  Ctrl+C 优雅退出 (删 ingress resource + 停 pipeline)

作者: Claude Code (2026-04-22)
"""
import asyncio
import json
import os
import signal
import sys
import time
import math
import socket
import threading
import traceback

# GStreamer Python binding — 必须先初始化 GStreamer 再 import 其他
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# LiveKit SDK
from livekit import rtc, api


# ==================== 配置 ====================
# 注意: 本脚本是历史 NVENC RTMP Ingress 试验, 已被 lk CLI 直接发布 (zed-hw-lk.service) 取代.
# 保留作参考, 不在生产路径上.
LIVEKIT_URL = os.environ.get("LIVEKIT_URL", "ws://127.0.0.1:7880")
# 凭证仅从 env 读取, 不再带默认值 (旧默认值已被公开,见 SECURITY.md)
LIVEKIT_API_KEY = os.environ.get("LIVEKIT_API_KEY")
LIVEKIT_API_SECRET = os.environ.get("LIVEKIT_API_SECRET")
if not (LIVEKIT_API_KEY and LIVEKIT_API_SECRET):
    raise RuntimeError(
        "LIVEKIT_API_KEY and LIVEKIT_API_SECRET env vars required "
        "(see ../.env.example and ../SECURITY.md)"
    )
LIVEKIT_ROOM = "teleop-room"

# 两个 identity:
LIVEKIT_DATA_IDENTITY = "zed-mini-sender-data"      # 本进程订阅 pose data
INGRESS_PARTICIPANT_IDENTITY = "zed-mini-sender-hw" # Ingress 创建, 推视频 track
INGRESS_NAME = "zed-hw-rtmp"

RTMP_HOST = os.environ.get("RTMP_HOST", "127.0.0.1")
RTMP_PORT = int(os.environ.get("RTMP_PORT", "1935"))

# ZED Mini WVGA 硬件档位: 100/60/30/15 fps. 100Hz 拉满.
ZED_DEVICE = "/dev/video0"
CAM_WIDTH = 1344
CAM_HEIGHT = 376
CAM_FPS = int(os.environ.get("CAM_FPS", "100"))

# 硬编参数 — Jetson NVENC 最低延迟配置
H264_BITRATE_BPS = int(os.environ.get("H264_BITRATE", "8000000"))  # 8 Mbps
H264_IDR_INTERVAL = 30   # 30 帧/IDR (100fps → 0.3s, 快速 recovery)
H264_PRESET_LEVEL = 1    # 1=UltraFastPreset, 4=FastPreset, 7=SlowPreset
H264_PROFILE = 2         # 0=Baseline 1=ConstrainedBaseline 2=Main 4=High. Main 对 VP 兼容性最好

# 达妙电机 UDP 转发 (本机环回, 达妙引擎进程监听这个)
DAMO_UDP_HOST = "127.0.0.1"
DAMO_UDP_PORT = 9000

# 打印频率控制
POSE_PRINT_EVERY = 30

# 全局运行标志
running = True


# ==================== 四元数 → 欧拉角 (沿用原 sender 逻辑) ====================
def quat_to_euler(qx, qy, qz, qw):
    """
    ZYX 内旋欧拉角 (航空惯例).
    返回 (pitch, roll, yaw) 单位弧度, 含万向锁钳位.
    """
    # Roll (X 轴旋转)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # Pitch (Y 轴旋转, 万向锁处理)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    # Yaw (Z 轴旋转)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return pitch, roll, yaw


# ==================== GStreamer 硬编 Pipeline ====================
class HwEncodePipeline:
    """
    Jetson NVENC 硬编管线. 全程 GPU/VPU, CPU 只做 mailbox.

    Pipeline 关键元素:
      - v4l2src: 从 /dev/video0 拉 YUY2 raw
      - nvvidconv flip-method=2: Jetson VPU 做 YUY2→NV12 + 旋转 180° (ZED Mini 倒置)
      - video/x-raw(memory:NVMM): NV12 数据存在 NVMM (GPU 侧) 内存, 不拷到 CPU
      - nvv4l2h264enc: V4L2 API 调 Jetson NVENC 硬件编码单元
          - bitrate: CBR 目标码率
          - insert-sps-pps=1: 每个 IDR 前重插 SPS/PPS, 解码端随时起播
          - iframeinterval: IDR 帧间隔
          - preset-level=1: UltraFastPreset (最低 encode 延迟)
          - maxperf-enable=1: 禁用 encoder 节流, MAXN 模式配合
          - control-rate=1: CBR
          - profile=2: Main profile (兼容 VP)
      - h264parse config-interval=1: 每秒插入 SPS/PPS (双重保险)
      - flvmux streamable=true: 实时流 FLV, 不缓 moov
      - rtmpsink: TCP 推 RTMP. "live=1" = 直播模式不 repeat
    """
    def __init__(self, rtmp_url: str):
        Gst.init(None)

        pipeline_desc = (
            f'v4l2src device={ZED_DEVICE} do-timestamp=true '
            f'! video/x-raw,format=YUY2,width={CAM_WIDTH},height={CAM_HEIGHT},framerate={CAM_FPS}/1 '
            f'! nvvidconv flip-method=2 '
            f'! video/x-raw(memory:NVMM),format=NV12 '
            f'! nvv4l2h264enc '
                f'bitrate={H264_BITRATE_BPS} '
                f'insert-sps-pps=1 '
                f'iframeinterval={H264_IDR_INTERVAL} '
                f'preset-level={H264_PRESET_LEVEL} '
                f'maxperf-enable=1 '
                f'control-rate=1 '
                f'profile={H264_PROFILE} '
            f'! h264parse config-interval=1 '
            f'! flvmux streamable=true '
            f'! rtmpsink location="{rtmp_url} live=1" sync=false async=false '
        )

        print("[GStreamer] ======== 硬编管线 ========")
        print(f"    设备:     {ZED_DEVICE} {CAM_WIDTH}x{CAM_HEIGHT}@{CAM_FPS}fps (YUY2)")
        print(f"    色彩转换: nvvidconv (VPU 硬件, 零 CPU) + 旋转180°")
        print(f"    编码器:   nvv4l2h264enc (NVENC 硬编)")
        print(f"    profile:  {'Baseline ConstrainedBaseline _ Main _ High'.split()[H264_PROFILE]} ")
        print(f"    码率:     {H264_BITRATE_BPS/1e6:.1f} Mbps CBR")
        print(f"    IDR 间隔: {H264_IDR_INTERVAL} 帧 ({H264_IDR_INTERVAL/CAM_FPS*1000:.0f}ms)")
        print(f"    preset:   level={H264_PRESET_LEVEL} (1=UltraFast)")
        print(f"    容器:     FLV (streamable)")
        print(f"    目标:     {rtmp_url}")
        print("[GStreamer] =========================")

        try:
            self.pipe = Gst.parse_launch(pipeline_desc)
        except GLib.Error as e:
            print(f"[GStreamer] 管线解析失败: {e}")
            raise

        bus = self.pipe.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

        self.started = False
        self.error = None

    def _on_bus_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"\n[GStreamer 错误] {err}")
            print(f"[GStreamer 调试] {debug}\n")
            self.error = err
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            print(f"[GStreamer 警告] {err}")
        elif t == Gst.MessageType.EOS:
            print("[GStreamer] 收到 EOS")

    def start(self):
        print("[GStreamer] 启动 pipeline → PLAYING...")
        ret = self.pipe.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("GStreamer pipeline 启动失败")
        # 等异步状态变化完成 (最多 3 秒)
        ret, state, pending = self.pipe.get_state(timeout=3 * Gst.SECOND)
        print(f"[GStreamer] 状态: {state.value_nick} (pending={pending.value_nick})")
        self.started = True

    def stop(self):
        if not self.started:
            return
        print("[GStreamer] 发送 EOS...")
        self.pipe.send_event(Gst.Event.new_eos())
        time.sleep(0.5)  # 让 EOS 传到 rtmpsink, 体面地关 TCP
        print("[GStreamer] NULL state...")
        self.pipe.set_state(Gst.State.NULL)
        self.started = False
        print("[GStreamer] pipeline 已停止")


# ==================== Ingress Resource 管理 ====================
class IngressManager:
    """
    通过 LiveKit API 创建 / 删除 RTMP Ingress resource.

    Ingress 服务侧会:
      1. 在 RTMP server 上分配一个 stream_key
      2. 在指定 room 中"预留"一个 participant identity
      3. 当 RTMP 流连上 stream_key 时, 开始 transcode → 作为 virtual participant 推视频 track

    生命周期: 进程退出时 delete, 避免残留 ingress 占 room slot.
    """
    def __init__(self):
        self.api_client = None
        self.info = None

    async def create(self):
        # LiveKit HTTP API 地址 (ws:// → http://)
        http_url = LIVEKIT_URL.replace("ws://", "http://").replace("wss://", "https://")
        # 端口保持 7880 (HTTP 和 WS 同端口, LiveKit server 一体)
        self.api_client = api.LiveKitAPI(
            url=http_url,
            api_key=LIVEKIT_API_KEY,
            api_secret=LIVEKIT_API_SECRET,
        )

        req = api.CreateIngressRequest(
            input_type=api.IngressInput.RTMP_INPUT,
            name=INGRESS_NAME,
            room_name=LIVEKIT_ROOM,
            participant_identity=INGRESS_PARTICIPANT_IDENTITY,
            participant_name="ZED Mini HW Encoder (Orin)",
            # bypass_transcoding=True,  # 如果 VP 客户端能直接解 H264 RTP, 可跳过 transcoding
                                         # 但 bypass 模式 ingress 对输入编码要求严格, 先走默认 transcode
        )

        print(f"[Ingress] 创建 RTMP ingress → room={LIVEKIT_ROOM}, identity={INGRESS_PARTICIPANT_IDENTITY}")
        self.info = await self.api_client.ingress.create_ingress(req)

        print(f"[Ingress] 创建成功:")
        print(f"    ingress_id: {self.info.ingress_id}")
        print(f"    url:        {self.info.url}")
        print(f"    stream_key: {self.info.stream_key}")
        return self.info

    def build_rtmp_url(self) -> str:
        """
        拼 GStreamer rtmpsink 要的完整 URL.

        Ingress 返回的 url 可能形如:
          rtmp://<host>:1935/x/
        拼: <url>/<stream_key>

        兜底: 如果 ingress 返回的 url 为空或 host 有问题 (如容器里的 IP),
        用 RTMP_HOST env 覆盖.
        """
        base = (self.info.url or "").rstrip("/")
        key = self.info.stream_key or ""

        if not base.startswith("rtmp://") and not base.startswith("rtmps://"):
            # ingress 没给完整 URL, 自己拼
            base = f"rtmp://{RTMP_HOST}:{RTMP_PORT}/live"
        else:
            # 如果 base host 不可达 (如 0.0.0.0 或容器 IP), 替换为 RTMP_HOST
            # 简单处理: 替换 host 部分
            import urllib.parse
            p = urllib.parse.urlparse(base)
            if p.hostname in (None, "0.0.0.0", "::", ""):
                base = base.replace(
                    p.netloc, f"{RTMP_HOST}:{p.port or RTMP_PORT}"
                )

        return f"{base}/{key}"

    async def delete(self):
        if not self.info or not self.api_client:
            if self.api_client:
                await self.api_client.aclose()
            return
        try:
            print(f"[Ingress] 删除 ingress: {self.info.ingress_id}")
            await self.api_client.ingress.delete_ingress(
                api.DeleteIngressRequest(ingress_id=self.info.ingress_id)
            )
            print(f"[Ingress] 删除 OK")
        except Exception as e:
            print(f"[Ingress] 删除失败: {e}")
        finally:
            await self.api_client.aclose()


# ==================== Data Channel Receiver ====================
class DataChannelReceiver:
    """
    本进程作为"纯 data channel 订阅者"连 LiveKit room.
    收 VP 的 pose → UDP 转发到达妙电机引擎.
    不推任何视频 (视频走 Ingress + GStreamer 硬编).
    """
    def __init__(self):
        self.room = None
        self.pose_recv_count = 0
        self.first_pose_time = None
        self.damo_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.damo_addr = (DAMO_UDP_HOST, DAMO_UDP_PORT)
        print(f"[Data] UDP 转发目标: {DAMO_UDP_HOST}:{DAMO_UDP_PORT}")

    def _generate_token(self) -> str:
        token = api.AccessToken(LIVEKIT_API_KEY, LIVEKIT_API_SECRET)
        token.with_identity(LIVEKIT_DATA_IDENTITY)
        token.with_grants(api.VideoGrants(
            room_join=True,
            room=LIVEKIT_ROOM,
            can_publish=False,          # 本进程不推视频
            can_publish_data=True,      # 允许 debug 发消息
            can_subscribe=True,         # 订阅其他参与者 (主要 VP 的 data)
        ))
        return token.to_jwt()

    def _on_data_received(self, data_packet: rtc.DataPacket):
        """VP → sender 的 pose 回调"""
        try:
            text = data_packet.data.decode('utf-8')
            msg = json.loads(text)
        except (json.JSONDecodeError, UnicodeDecodeError):
            return

        if msg.get("type") != "pose":
            return

        self.pose_recv_count += 1
        if self.first_pose_time is None:
            self.first_pose_time = time.time()
            print(f"[Data] 首个 pose 到达 ✓ (从 connect 起 {time.time() - self._connect_time:.2f}s)")

        quat = msg.get("q", [0, 0, 0, 1])
        ts = msg.get("t", 0)

        pitch, roll, yaw = quat_to_euler(quat[0], quat[1], quat[2], quat[3])

        # 物理含义 (沿用原 sender 映射):
        #   motor_pitch (点头) ← math.roll  (绕 X 轴)
        #   motor_yaw   (转头) ← math.pitch (绕 Y 轴)
        motor_pitch = math.degrees(roll)
        motor_yaw = math.degrees(pitch)

        # UDP 转发到达妙引擎
        payload = json.dumps({
            "pitch": round(motor_pitch, 2),
            "yaw": round(motor_yaw, 2),
            "t": ts,
        })
        try:
            self.damo_sock.sendto(payload.encode('utf-8'), self.damo_addr)
        except Exception:
            pass  # UDP 发送失败不影响主流程

        if self.pose_recv_count % POSE_PRINT_EVERY == 1:
            elapsed = time.time() - self.first_pose_time
            rate = self.pose_recv_count / elapsed if elapsed > 0 else 0
            print(f"[Pose] #{self.pose_recv_count} @ {rate:.1f}Hz → "
                  f"pitch={motor_pitch:+6.1f}° yaw={motor_yaw:+6.1f}°")

    async def connect(self):
        self.room = rtc.Room()
        self.room.on("data_received", self._on_data_received)

        # 可选: log 参与者加入/退出
        def _on_participant_connected(p: rtc.RemoteParticipant):
            print(f"[LiveKit] 参与者加入: {p.identity} (kind={p.kind})")
        def _on_participant_disconnected(p: rtc.RemoteParticipant):
            print(f"[LiveKit] 参与者离开: {p.identity}")
        self.room.on("participant_connected", _on_participant_connected)
        self.room.on("participant_disconnected", _on_participant_disconnected)

        jwt = self._generate_token()
        print(f"[LiveKit] 连接 {LIVEKIT_URL} (identity={LIVEKIT_DATA_IDENTITY})")
        self._connect_time = time.time()
        await self.room.connect(LIVEKIT_URL, jwt)
        print(f"[LiveKit] 已连接 room={self.room.name} sid={self.room.sid}")
        print(f"[LiveKit] 当前参与者: {[p.identity for p in self.room.remote_participants.values()]}")

    async def disconnect(self):
        if self.room:
            await self.room.disconnect()
            print("[LiveKit] data 订阅断开")


# ==================== Main ====================
async def main():
    global running

    def sig_handler(sig, frame):
        global running
        print(f"\n[!] 收到信号 {sig}, 优雅退出中...")
        running = False

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    print("=" * 68)
    print("  ZED Mini 硬编 RTMP Sender — Jetson NVENC + LiveKit Ingress")
    print(f"  LiveKit:  {LIVEKIT_URL}")
    print(f"  Room:     {LIVEKIT_ROOM}")
    print(f"  RTMP:     {RTMP_HOST}:{RTMP_PORT}")
    print(f"  视频 ID:  {INGRESS_PARTICIPANT_IDENTITY} (Ingress virtual)")
    print(f"  数据 ID:  {LIVEKIT_DATA_IDENTITY} (本进程)")
    print("=" * 68)

    # GLib mainloop — GStreamer 总线消息处理需要
    glib_loop = GLib.MainLoop()
    glib_thread = threading.Thread(target=glib_loop.run, daemon=True)
    glib_thread.start()
    print("[Main] GLib mainloop 后台线程已启动")

    ingress = IngressManager()
    pipeline = None
    receiver = DataChannelReceiver()

    try:
        # Step 1: 创建 Ingress resource (拿 rtmp url + stream key)
        info = await ingress.create()
        rtmp_url = ingress.build_rtmp_url()
        print(f"\n[Main] GStreamer 推流目标: {rtmp_url}\n")

        # Step 2: 启硬编 GStreamer pipeline
        pipeline = HwEncodePipeline(rtmp_url)
        pipeline.start()
        # 给 ingress 一些时间建立 RTMP 连接
        await asyncio.sleep(2)
        if pipeline.error:
            raise RuntimeError(f"pipeline 异常退出: {pipeline.error}")

        # Step 3: 连 LiveKit 订阅 data channel
        await receiver.connect()

        print(f"\n[Main] ✓ 全部就绪. VP 连 room={LIVEKIT_ROOM} 可:")
        print(f"  - 订阅视频 track (来自 {INGRESS_PARTICIPANT_IDENTITY})")
        print(f"  - 发送 pose data → 本进程 ({LIVEKIT_DATA_IDENTITY}) 收 → UDP 转达妙")
        print(f"\n[Main] Ctrl+C 优雅退出 (自动清理 ingress + pipeline)\n")

        # 主循环等 signal. 顺便定期检查 pipeline 健康.
        tick = 0
        while running:
            await asyncio.sleep(1)
            tick += 1
            if pipeline.error:
                print(f"[Main] pipeline 已错误退出, 主循环结束")
                break
            if tick % 30 == 0:
                # 每 30 秒心跳
                print(f"[Main] 心跳 @ {tick}s — pipeline=OK, pose_recv={receiver.pose_recv_count}")

    except Exception as e:
        print(f"\n[!!!] main 异常: {e}")
        traceback.print_exc()

    finally:
        print("\n[清理] 正在关闭...")
        if pipeline:
            try:
                pipeline.stop()
            except Exception as e:
                print(f"[清理] pipeline stop 异常: {e}")
        try:
            await receiver.disconnect()
        except Exception as e:
            print(f"[清理] receiver disconnect 异常: {e}")
        try:
            await ingress.delete()
        except Exception as e:
            print(f"[清理] ingress delete 异常: {e}")
        if glib_loop.is_running():
            glib_loop.quit()
        print("[清理] 完成")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
