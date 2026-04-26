#!/usr/bin/python3
"""
========================================================
  ZED Mini LiveKit WebRTC 发送端 v1.0
========================================================
用 LiveKit Python SDK 替代 GStreamer webrtcbin / aiortc 做 WebRTC 传输。
LiveKit 内部自动处理：H264/VP8 编码、DTLS-SRTP、ICE、NACK/PLI 等。
如果系统有 NVENC，LiveKit 会自动调用硬件编码器。

架构：
  GStreamer: v4l2src → videoconvert → videoflip → appsink（输出 RGBA 原始帧）
  LiveKit:   VideoSource.capture_frame(RGBA) → 自动编码 → DTLS-SRTP → ICE → 发送

Data Channel（VP → Linux）：
  VP 通过 publish_data 发送 pose JSON → Room "data_received" 事件
  → 解析四元数 → UDP 转发给达妙引擎（127.0.0.1:9000）

依赖：
  - livekit (pip install livekit)         — LiveKit Python SDK
  - livekit-api (pip install livekit-api) — Token 生成
  - PyGObject (系统包 python3-gi)         — GStreamer 绑定
  - numpy                                 — 帧数据操作
"""

import asyncio
import json
import signal
import sys
import os
import time
import math
import socket
import struct
import threading
import traceback

import numpy as np

# ==================== GStreamer 初始化 ====================
# 必须在其他 import 之前初始化 GStreamer，否则 GLib 主循环可能出问题
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# LiveKit SDK
from livekit import rtc, api

# ==================== 配置常量 ====================

# LiveKit 服务器配置
LIVEKIT_URL = os.environ.get("LIVEKIT_URL", "ws://39.102.113.104:7880")  # env 可覆盖
# 凭证仅从 env 读取, 不再带默认值 (旧默认值已被公开,见 SECURITY.md)
LIVEKIT_API_KEY = os.environ.get("LIVEKIT_API_KEY")
LIVEKIT_API_SECRET = os.environ.get("LIVEKIT_API_SECRET")
if not (LIVEKIT_API_KEY and LIVEKIT_API_SECRET):
    raise RuntimeError(
        "LIVEKIT_API_KEY and LIVEKIT_API_SECRET env vars required "
        "(see ../.env.example and ../SECURITY.md)"
    )
LIVEKIT_ROOM = "teleop-room"                    # 房间名称
LIVEKIT_IDENTITY = "zed-mini-sender"            # 本机身份标识

# ZED Mini 摄像头配置
ZED_DEVICE = "/dev/video0"      # ZED Mini V4L2 设备路径
CAM_WIDTH = 1344                # ZED Mini 原始宽度（左右眼拼接 side-by-side）
CAM_HEIGHT = 376                # ZED Mini 原始高度（WVGA 模式）
CAM_FPS = 100                   # 采集帧率 (ZED Mini WVGA 1344x376 硬件离散档: 100/60/30/15)
                                #  拉满 100Hz, 每帧排队延迟从 16.7ms 降到 10ms (-6.7ms)

# 达妙引擎 UDP 转发配置
DAMO_UDP_HOST = "127.0.0.1"    # 达妙引擎监听地址
DAMO_UDP_PORT = 9000           # 达妙引擎监听端口

# 打印频率控制
FRAME_PRINT_EVERY = 300         # 每 N 帧打印一次采集状态
POSE_PRINT_EVERY = 6            # 每 N 次 pose 打印一次 (30Hz pose → ~5Hz 打印)

# ==================== 全局状态 ====================
running = True                  # 全局运行标志，Ctrl+C 时置 False


# ==================== 四元数转欧拉角 ====================
def quat_to_euler(qx, qy, qz, qw):
    """
    将四元数 (qx, qy, qz, qw) 转换为欧拉角 (pitch, roll, yaw)。
    使用 ZYX 内旋顺序（航空惯例）。

    返回值：
        pitch (绕 Y 轴) — 点头
        roll  (绕 X 轴) — 侧倾
        yaw   (绕 Z 轴) — 转头
    单位：弧度
    """
    # Roll (X 轴旋转)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (Y 轴旋转) — 需要处理万向锁边界
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # 万向锁，钳位到 ±90°
    else:
        pitch = math.asin(sinp)

    # Yaw (Z 轴旋转)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return pitch, roll, yaw


# ==================== GStreamer 摄像头采集器 ====================
class GStreamerCapture:
    """
    用 GStreamer 从 ZED Mini 采集 RGBA 原始帧。

    管道流程：
        v4l2src (YUY2) → videoconvert (RGBA) → videoflip (旋转180°) → appsink

    appsink 通过 emit-signals 回调将每一帧送到 Python 层，
    存入 _latest_frame 供 LiveKit 推帧线程读取。
    """

    def __init__(self, device=ZED_DEVICE, width=CAM_WIDTH, height=CAM_HEIGHT, fps=CAM_FPS):
        """
        初始化 GStreamer 采集管道。

        参数：
            device: V4L2 设备路径
            width:  采集宽度
            height: 采集高度
            fps:    采集帧率
        """
        # 初始化 GStreamer（安全调用，多次 init 不会出错）
        Gst.init(None)

        self.width = width
        self.height = height
        self.fps = fps

        # 帧缓冲区（线程安全）
        self._latest_frame = None       # 最新一帧的 RGBA bytes 数据
        self._frame_event = threading.Event()   # 新帧到达事件（用于跨线程同步）
        self._lock = threading.Lock()           # 保护 _latest_frame 的互斥锁
        self._frame_count = 0                   # 总帧计数

        # 构建 GStreamer 管道描述字符串
        # 关键点：
        #   1. v4l2src 采集 YUY2 原始数据（ZED Mini 原生格式）
        #   2. videoconvert 转换为 RGBA（LiveKit VideoSource 要求 RGBA 输入）
        #   3. videoflip 旋转 180°（ZED Mini 物理安装是倒置的）
        #   4. appsink 输出帧到 Python 回调
        #      - emit-signals=true: 每帧触发 new-sample 信号
        #      - max-buffers=2: 最多缓存 2 帧，防止内存暴涨
        #      - drop=true: 缓冲满时丢弃旧帧（保证低延迟）
        #      - sync=false: 不做时钟同步（实时推流不需要）
        pipeline_str = (
            f'v4l2src device={device} do-timestamp=true '
            f'! video/x-raw,format=YUY2,width={width},height={height},framerate={fps}/1 '
            '! videoconvert ! video/x-raw,format=RGBA '
            '! videoflip method=rotate-180 '
            '! appsink name=sink emit-signals=true max-buffers=2 drop=true sync=false'
        )

        # 解析并构建管道
        self.pipe = Gst.parse_launch(pipeline_str)

        # 获取 appsink 元素并连接回调
        self.sink = self.pipe.get_by_name('sink')
        self.sink.connect('new-sample', self._on_new_sample)

        # 监听管道总线消息（错误/警告）
        bus = self.pipe.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

        print(f"[GStreamer] 管道已创建:")
        print(f"    设备: {device}")
        print(f"    格式: YUY2 → RGBA, {width}x{height}@{fps}fps")
        print(f"    旋转: 180° (ZED Mini 倒置修正)")

    def _on_bus_message(self, bus, message):
        """GStreamer 总线消息回调 — 打印错误和警告"""
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"\n[GStreamer 错误] {err}")
            print(f"[GStreamer 调试] {debug}\n")
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            print(f"[GStreamer 警告] {err}")

    def _on_new_sample(self, sink):
        """
        GLib 线程回调：appsink 每收到一帧都会触发此函数。
        从 GstBuffer 中提取 RGBA 像素数据，存入 _latest_frame。
        """
        # 拉取 sample（包含 buffer + caps 信息）
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        # 将 GstBuffer 映射到可读内存
        ok, map_info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK

        # 复制帧数据（RGBA 格式，每像素 4 字节）
        # 总大小 = width * height * 4
        data = bytes(map_info.data)
        buf.unmap(map_info)

        # 线程安全地更新最新帧
        with self._lock:
            self._latest_frame = data
            self._frame_count += 1
            count = self._frame_count

        # 通知等待线程有新帧
        self._frame_event.set()

        # 定期打印采集状态
        if count % FRAME_PRINT_EVERY == 1:
            expected_size = self.width * self.height * 4  # RGBA = 4 bytes/pixel
            print(f"[GStreamer] 帧 #{count}: {len(data)} bytes "
                  f"(期望 {expected_size}), {self.width}x{self.height} RGBA")

        return Gst.FlowReturn.OK

    def get_frame(self, timeout=0.05):
        """
        阻塞等待下一帧（跨线程安全）。

        参数：
            timeout: 最长等待时间（秒）
        返回：
            RGBA bytes 数据，或 None（超时）
        """
        self._frame_event.wait(timeout=timeout)
        self._frame_event.clear()
        with self._lock:
            frame = self._latest_frame
            self._latest_frame = None
        return frame

    @property
    def frame_count(self):
        """当前已采集的帧数"""
        with self._lock:
            return self._frame_count

    def start(self):
        """启动 GStreamer 管道（开始采集）"""
        self.pipe.set_state(Gst.State.PLAYING)
        print("[GStreamer] 摄像头已启动，等待帧数据...\n")

    def stop(self):
        """停止 GStreamer 管道"""
        self.pipe.set_state(Gst.State.NULL)
        print("[GStreamer] 摄像头已停止")


# ==================== LiveKit 发送器 ====================
class LiveKitSender:
    """
    LiveKit WebRTC 发送端。

    职责：
        1. 连接 LiveKit Server（自动生成 JWT Token）
        2. 创建 VideoSource + LocalVideoTrack 并发布
        3. 从 GStreamer 采集器获取 RGBA 帧 → VideoSource.capture_frame()
        4. 监听 Room "data_received" 事件接收 VP 回传的 pose 数据
        5. 将 pose 数据 UDP 转发给达妙引擎
    """

    def __init__(self, capture: GStreamerCapture):
        """
        初始化 LiveKit 发送器。

        参数：
            capture: GStreamer 采集器实例
        """
        self.capture = capture
        self.room = None                    # LiveKit Room 实例
        self.video_source = None            # LiveKit VideoSource
        self.video_track = None             # LiveKit LocalVideoTrack
        self.published = False              # 是否已成功发布轨道

        # Pose 数据统计
        self.pose_recv_count = 0            # 收到的 pose 总数

        # 达妙 UDP 转发 socket
        self.damo_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.damo_addr = (DAMO_UDP_HOST, DAMO_UDP_PORT)

        # 推帧统计
        self._push_count = 0               # 已推送到 LiveKit 的帧数
        self._push_start_time = None        # 推帧开始时间（用于计算 FPS）

    def _generate_token(self):
        """
        生成 LiveKit JWT Token。

        Token 包含：
            - identity: 发送端身份标识
            - grants: 房间加入权限 + 发布权限

        返回：
            JWT 字符串
        """
        token = api.AccessToken(LIVEKIT_API_KEY, LIVEKIT_API_SECRET)
        token.with_identity(LIVEKIT_IDENTITY)
        token.with_grants(api.VideoGrants(
            room_join=True,             # 允许加入房间
            room=LIVEKIT_ROOM,          # 指定房间名
            can_publish=True,           # 允许发布轨道
            can_publish_data=True,      # 允许发布数据（Data Channel）
            can_subscribe=True,         # 允许订阅其他参与者的轨道
        ))
        jwt = token.to_jwt()
        print(f"[LiveKit] Token 已生成 (identity={LIVEKIT_IDENTITY}, room={LIVEKIT_ROOM})")
        return jwt

    def _on_data_received(self, data_packet: rtc.DataPacket):
        """
        LiveKit Data Channel 接收回调。

        VP 端通过 publish_data 发送 pose JSON：
            {"type":"pose", "t":<timestamp>, "p":[x,y,z], "q":[qx,qy,qz,qw]}

        处理流程：
            1. 解析 JSON
            2. 四元数 → 欧拉角
            3. UDP 转发给达妙引擎
            4. 定期打印 pose 信息
        """
        try:
            # data_packet.data 是 bytes
            text = data_packet.data.decode('utf-8')
            msg = json.loads(text)
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            # 非 JSON 数据，忽略
            return

        msg_type = msg.get("type", "")

        if msg_type == "pose":
            self.pose_recv_count += 1

            # 提取位置和四元数
            pos = msg.get("p", [0, 0, 0])
            quat = msg.get("q", [0, 0, 0, 1])
            ts = msg.get("t", 0)

            # 四元数转欧拉角
            pitch, roll, yaw = quat_to_euler(quat[0], quat[1], quat[2], quat[3])

            # 达妙引擎使用的角度约定：
            #   motor_pitch = roll（物理安装映射）
            #   motor_yaw   = pitch（物理安装映射）
            motor_pitch = math.degrees(roll)
            motor_yaw = math.degrees(pitch)

            # UDP 转发给达妙引擎
            payload = json.dumps({
                "pitch": round(motor_pitch, 2),
                "yaw": round(motor_yaw, 2),
                "t": ts
            })
            try:
                self.damo_sock.sendto(payload.encode('utf-8'), self.damo_addr)
            except Exception:
                pass  # UDP 发送失败不影响主流程

            # 定期打印 pose (5Hz). ★ 全部用物理含义命名,不再混入数学命名,避免歧义
            #
            # 物理 PITCH 点头  ← 数学 roll(绕X 轴)
            # 物理 YAW   转头  ← 数学 pitch(绕Y 轴)
            # 物理 ROLL  侧倾  ← 数学 yaw(绕Z 轴)
            #
            # (visionOS device frame: +X 右, +Y 上, -Z 前)
            #
            # 发到 UDP 9000 的字段名也是物理含义 (pitch=点头, yaw=转头)
            if self.pose_recv_count % POSE_PRINT_EVERY == 0:
                participant_name = ""
                if data_packet.participant:
                    participant_name = f" from={data_packet.participant.identity}"
                phys_roll_deg = math.degrees(yaw)   # 数学 yaw(绕Z) = 物理侧倾
                print(f"[Pose #{self.pose_recv_count}]{participant_name}")
                print(f"   PITCH 点头(抬/低头)  = {motor_pitch:+7.2f}°")
                print(f"   YAW   转头(左/右转)  = {motor_yaw:+7.2f}°")
                print(f"   ROLL  侧倾(耳贴肩)   = {phys_roll_deg:+7.2f}°")
                print(f"   位置 pos = [{pos[0]:+.2f}, {pos[1]:+.2f}, {pos[2]:+.2f}]")

    async def _push_frames_loop(self):
        """
        推帧循环：不断从 GStreamer 获取 RGBA 帧，推送到 LiveKit VideoSource。

        LiveKit SDK 内部会自动：
            - 将 RGBA 帧编码为 H264/VP8（有 NVENC 时用硬件编码）
            - 封装为 RTP 包
            - 通过 DTLS-SRTP 加密
            - 通过 ICE 传输到接收端

        此函数在 asyncio 事件循环中运行，通过 run_in_executor 调用
        GStreamer 的阻塞 get_frame() 方法，避免阻塞事件循环。
        """
        loop = asyncio.get_event_loop()
        self._push_start_time = time.time()
        self._push_count = 0

        print("[LiveKit] 开始推帧循环...")

        while running:
            try:
                # 在线程池中等待 GStreamer 产生新帧（避免阻塞 asyncio）
                frame_data = await loop.run_in_executor(None, self.capture.get_frame, 0.05)

                if frame_data is None:
                    # 超时没有新帧，继续等待
                    continue

                # 验证帧数据大小
                expected_size = self.capture.width * self.capture.height * 4  # RGBA = 4 bytes/pixel
                if len(frame_data) != expected_size:
                    # 帧大小不匹配，跳过（可能是管道还没稳定）
                    if self._push_count == 0:
                        print(f"[LiveKit] 帧大小不匹配: {len(frame_data)} != {expected_size}, 跳过")
                    continue

                # 构建 LiveKit VideoFrame（RGBA 格式）
                frame = rtc.VideoFrame(
                    width=self.capture.width,
                    height=self.capture.height,
                    type=rtc.VideoBufferType.RGBA,
                    data=frame_data,
                )

                # 推送帧到 VideoSource → LiveKit 自动编码并发送
                self.video_source.capture_frame(frame)

                self._push_count += 1

                # 定期打印推帧状态
                if self._push_count % FRAME_PRINT_EVERY == 1:
                    elapsed = time.time() - self._push_start_time
                    fps = self._push_count / elapsed if elapsed > 0 else 0
                    print(f"[LiveKit] 推帧 #{self._push_count}: "
                          f"{self.capture.width}x{self.capture.height} RGBA, "
                          f"平均 {fps:.1f} fps, "
                          f"运行 {elapsed:.1f}s")

            except Exception as e:
                if running:
                    print(f"[LiveKit] 推帧异常: {e}")
                    traceback.print_exc()
                    # 短暂等待后重试，避免错误风暴
                    await asyncio.sleep(0.1)

        print(f"[LiveKit] 推帧循环结束，共推送 {self._push_count} 帧")

    async def connect_and_publish(self):
        """
        连接 LiveKit 房间并发布视频轨道。

        流程：
            1. 生成 JWT Token
            2. 创建 Room 实例并注册事件回调
            3. 连接到 LiveKit Server
            4. 创建 VideoSource + LocalVideoTrack
            5. 发布轨道到房间
            6. 启动推帧循环
        """
        # 生成 Token
        jwt = self._generate_token()

        # 创建 Room 实例
        self.room = rtc.Room()

        # ---- 注册 Room 事件回调 ----

        # 连接状态变化
        @self.room.on("connection_state_changed")
        def on_connection_state(state):
            print(f"[LiveKit] 连接状态: {state}")

        # 参与者加入
        @self.room.on("participant_connected")
        def on_participant_connected(participant: rtc.RemoteParticipant):
            print(f"[LiveKit] 参与者加入: {participant.identity} (sid={participant.sid})")

        # 参与者离开
        @self.room.on("participant_disconnected")
        def on_participant_disconnected(participant: rtc.RemoteParticipant):
            print(f"[LiveKit] 参与者离开: {participant.identity}")

        # 轨道被订阅（VP 端订阅了我们的视频）
        @self.room.on("track_subscribed")
        def on_track_subscribed(track, publication, participant):
            print(f"[LiveKit] 轨道被订阅: {track.kind} by {participant.identity}")

        # Data Channel 数据接收（VP 回传 pose）
        @self.room.on("data_received")
        def on_data_received(data_packet: rtc.DataPacket):
            self._on_data_received(data_packet)

        # 断开连接
        @self.room.on("disconnected")
        def on_disconnected(reason):
            print(f"[LiveKit] 断开连接: {reason}")

        # ---- 连接到 LiveKit Server ----
        print(f"\n[LiveKit] 正在连接: {LIVEKIT_URL}")
        print(f"[LiveKit] 房间: {LIVEKIT_ROOM}, 身份: {LIVEKIT_IDENTITY}")

        await self.room.connect(LIVEKIT_URL, jwt)

        print(f"[LiveKit] 连接成功!")
        print(f"[LiveKit] 房间名: {self.room.name}")
        # room.sid 是 async property，返回 coroutine，需要 await
        room_sid = await self.room.sid
        print(f"[LiveKit] 房间 SID: {room_sid}")
        print(f"[LiveKit] 本地参与者: {self.room.local_participant.identity}")

        # ---- 创建 VideoSource 和 VideoTrack ----
        # VideoSource 是帧数据的入口点
        # LiveKit 根据 width/height 自动配置编码器分辨率
        self.video_source = rtc.VideoSource(
            width=self.capture.width,
            height=self.capture.height,
        )

        # 创建本地视频轨道，关联到 VideoSource
        self.video_track = rtc.LocalVideoTrack.create_video_track(
            'zed-mini-stereo',      # 轨道名称（VP 端可见）
            self.video_source,
        )

        print(f"[LiveKit] VideoSource 已创建: {self.capture.width}x{self.capture.height}")
        print(f"[LiveKit] VideoTrack 已创建: {self.video_track.name}")

        # ---- 发布轨道 ----
        # 配置发布选项
        publish_options = rtc.TrackPublishOptions()
        publish_options.video_codec = rtc.VideoCodec.H264   # 指定 H264 编码（visionOS 有硬件解码器）
        publish_options.source = rtc.TrackSource.SOURCE_CAMERA  # 标记为摄像头源

        # ★ 显式配置 VideoEncoding —— 不设的话 SDK 默认 max_framerate=30,
        #   会把我们 100 fps 的推帧砍到 30 fps (这就是 VP 端只看到 30 fps 的根因!)
        #   max_bitrate 给足 15 Mbps 让 NVENC 不会因为带宽限制丢帧。
        #   注意: VideoEncoding 是 protobuf message, 不能直接 = 赋值,
        #   必须逐字段写。
        publish_options.video_encoding.max_framerate = CAM_FPS
        publish_options.video_encoding.max_bitrate   = 15_000_000

        publication = await self.room.local_participant.publish_track(
            self.video_track,
            publish_options,
        )

        self.published = True
        print(f"[LiveKit] 轨道已发布!")
        print(f"[LiveKit] Publication SID: {publication.sid}")
        print(f"[LiveKit] 编码: H264")
        print()

        # ---- 启动推帧循环 ----
        await self._push_frames_loop()

    async def run(self):
        """
        主运行循环（带自动重连）。

        如果连接断开或出错，会在 5 秒后自动重连。
        """
        retry = 0

        while running:
            try:
                retry += 1
                print(f"\n{'=' * 50}")
                print(f"  第 {retry} 次连接尝试")
                print(f"{'=' * 50}\n")

                await self.connect_and_publish()

            except Exception as e:
                if running:
                    print(f"\n[LiveKit] 连接/推流异常: {e}")
                    traceback.print_exc()
                    print(f"[LiveKit] 5 秒后重连...\n")

            # 清理旧连接
            if self.room:
                try:
                    await self.room.disconnect()
                except Exception:
                    pass
                self.room = None
                self.video_source = None
                self.video_track = None
                self.published = False

            if running:
                await asyncio.sleep(5)

    def cleanup(self):
        """清理资源"""
        try:
            self.damo_sock.close()
        except Exception:
            pass
        print("[LiveKit] 资源已清理")


# ==================== GLib 主循环（后台线程）====================
def start_glib_loop():
    """
    在后台线程运行 GLib 主循环。
    GStreamer 的信号回调（如 appsink 的 new-sample）需要 GLib 主循环驱动。
    """
    glib_loop = GLib.MainLoop()
    try:
        glib_loop.run()
    except Exception:
        pass


# ==================== 入口函数 ====================
def main():
    global running

    # 打印启动横幅
    print("=" * 60)
    print("  ZED Mini LiveKit WebRTC 发送端 v1.0")
    print(f"  分辨率: {CAM_WIDTH}x{CAM_HEIGHT} side-by-side | {CAM_FPS}fps")
    print(f"  LiveKit: {LIVEKIT_URL}")
    print(f"  房间: {LIVEKIT_ROOM} | 身份: {LIVEKIT_IDENTITY}")
    print("  LiveKit 自动处理: H264编码 / DTLS-SRTP / ICE / NACK")
    print("  GStreamer 仅负责摄像头采集 (YUY2 → RGBA)")
    print("=" * 60)
    print()

    # ---- 启动 GLib 主循环（后台线程）----
    # GStreamer 的 appsink 信号回调需要 GLib 主循环
    glib_thread = threading.Thread(target=start_glib_loop, daemon=True)
    glib_thread.start()
    print("[启动] GLib 主循环已启动（后台线程）")

    # ---- 初始化 GStreamer 摄像头采集 ----
    capture = GStreamerCapture()
    capture.start()

    # ---- 创建 LiveKit 发送器 ----
    sender = LiveKitSender(capture)
    print(f"[启动] 达妙 UDP 转发: {DAMO_UDP_HOST}:{DAMO_UDP_PORT}")
    print()

    # ---- 信号处理：Ctrl+C 优雅退出 ----
    def on_signal(sig, frame):
        global running
        if not running:
            # 第二次 Ctrl+C 强制退出
            print("\n[!] 强制退出")
            import os
            os._exit(1)
        print("\n[!] 收到停止信号，正在优雅退出...")
        running = False

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    # ---- 运行 asyncio 主循环 ----
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    try:
        loop.run_until_complete(sender.run())
    except KeyboardInterrupt:
        pass
    finally:
        # 清理所有资源
        running = False
        print("\n[清理] 正在关闭...")

        # 断开 LiveKit 连接
        if sender.room:
            try:
                loop.run_until_complete(sender.room.disconnect())
                print("[清理] LiveKit 已断开")
            except Exception:
                pass

        # 停止 GStreamer 管道
        capture.stop()

        # 清理发送器资源
        sender.cleanup()

        # 关闭事件循环
        loop.close()

        print("[清理] 全部完成，已退出")


if __name__ == "__main__":
    main()
