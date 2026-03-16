#!/usr/bin/env python3
"""
========================================
  WebRTC 发送端 - 混合方案
========================================

方案: 用 Python 处理 WebSocket 信令，用 gst-launch-1.0 处理视频流

优势:
  - 不需要 gi 模块
  - 可以使用任何 Python 3 (包括 miniconda3)

作者: Claude Code
日期: 2025-03-16
"""

import asyncio
import websockets
import json
import subprocess
import threading
import sys
import signal

# ============================================
# 配置区
# ============================================
SIGNALING_SERVER = "ws://39.102.113.104:8765"
TURN_SERVER = "turn://remote:Wawjxyz3!@39.102.113.104:3478"

# 本机公网 IP (用于向接收端宣告)
PUBLIC_IP = "36.110.28.59"

# 视频配置
LEFT_CAMERA = "/dev/video4"
WIDTH = 1280
HEIGHT = 720
FPS = 30
BITRATE = 4000

# ============================================
# GStreamer 管道 (使用子进程)
# ============================================
PIPELINE_CMD = [
    "gst-launch-1.0", "-v",
    "v4l2src", f"device={LEFT_CAMERA}",
    "!", "video/x-raw", f"format=YUY2,width={WIDTH},height={HEIGHT},framerate={FPS}/1",
    "!", "videoconvert",
    "!", "video/x-raw", "format=I420",
    "!", "nvh264enc", f"bitrate={BITRATE}", "preset=5", "rc-mode=cbr", "zerolatency=true",
    "!", "rtph264pay", "config-interval=1",
    "!", "udpsink", f"host=100.81.46.44", "port=5004", "sync=false"
]

# ============================================
# WebRTC 发送器类
# ============================================
class WebRTCSender:
    def __init__(self):
        self.ws = None
        self.gst_process = None
        self.running = True

    async def connect_signaling(self):
        """连接信令服务器"""
        print(f"[*] 正在连接信令服务器: {SIGNALING_SERVER}")
        print(f"[*] TURN 服务器: {TURN_SERVER}")
        print(f"[*] 公网 IP: {PUBLIC_IP}")
        print("")

        try:
            async with websockets.connect(SIGNALING_SERVER) as websocket:
                self.ws = websocket
                print("[+] 信令服务器连接成功！")
                print("")
                print("[*] 等待接收端连接...")

                # 发送注册消息
                await self.send_register()

                # 持续监听消息
                async for message in websocket:
                    await self.handle_message(json.loads(message))

        except Exception as e:
            print(f"[!] 连接失败: {e}")
            import traceback
            traceback.print_exc()

    async def send_register(self):
        """发送注册消息"""
        msg = {
            "type": "register",
            "role": "sender",
            "ip": PUBLIC_IP,
            "camera": "realsense-d435-stereo",
            "config": {
                "width": WIDTH,
                "height": HEIGHT,
                "fps": FPS,
                "bitrate": BITRATE
            }
        }
        await self.ws.send(json.dumps(msg))
        print(f"[>] 已发送注册消息")

    async def handle_message(self, data):
        """处理接收到的消息"""
        if 'sdp' in data:
            print(f"[<] 收到 SDP {data['sdp']['type']}")
            # SDP 处理逻辑...

        elif 'ice' in data:
            print(f"[<] 收到 ICE 候选")
            # ICE 候选处理逻辑...

        elif data.get('type') == 'start':
            print("[+] 收到启动信号，开始推流...")
            self.start_stream()

    def start_stream(self):
        """启动 GStreamer 推流"""
        if self.gst_process and self.gst_process.poll() is None:
            print("[*] 推流已在运行")
            return

        print(f"[*] 启动 GStreamer 推流...")
        print(f"    相机: {LEFT_CAMERA}")
        print(f"    分辨率: {WIDTH}x{HEIGHT} @ {FPS}fps")

        # 启动 gst-launch 子进程
        self.gst_process = subprocess.Popen(
            PIPELINE_CMD,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        # 启动日志输出线程
        threading.Thread(target=self.log_output, daemon=True).start()

        print("[+] 推流已启动")
        print("")
        print("在 Mac 上运行接收端:")
        print("  gst-launch-1.0 udpsrc port=5004 ! \\")
        print("    application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! \\")
        print("    rtpjitterbuffer latency=0 ! \\")
        print("    rtph264depay ! h264parse ! avdec_h264 ! \\")
        print("    videoconvert ! autovideosink sync=false")

    def log_output(self):
        """输出 GStreamer 日志"""
        if self.gst_process:
            for line in iter(self.gst_process.stderr.readline, b''):
                if not self.running:
                    break
                line = line.decode('utf-8', errors='ignore').strip()
                # 只输出重要信息
                if any(keyword in line for keyword in ['ERROR', 'WARNING', 'pipeline', 'PREROLLED']):
                    print(f"[GST] {line}")

    def stop(self):
        """停止推流"""
        self.running = False
        if self.gst_process:
            self.gst_process.terminate()
            self.gst_process.wait()
        print("[*] 推流已停止")


# ============================================
# 主程序
# ============================================
def signal_handler(sig, frame):
    """处理 Ctrl+C"""
    print("\n[!] 收到停止信号...")
    sender.stop()
    sys.exit(0)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)

    # 强制刷新输出
    sys.stdout = sys.__stdout__

    print("=" * 50, flush=True)
    print("  RealSense 双目 WebRTC 发送端", flush=True)
    print("=" * 50, flush=True)
    print("", flush=True)

    sender = WebRTCSender()

    try:
        asyncio.run(sender.connect_signaling())
    except KeyboardInterrupt:
        sender.stop()
