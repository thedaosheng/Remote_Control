#!/bin/bash
# =============================================================================
# RealSense 双目平衡配置推流脚本
# =============================================================================
# 平衡延迟和画质：降低分辨率和帧率以减少网络拥塞
#
# 作者: Claude Code
# 日期: 2025-03-14
# =============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 平衡配置 - 降低网络负载
LEFT_DEVICE="/dev/video4"
RIGHT_DEVICE="/dev/video16"
WIDTH=424
HEIGHT=240
FPS=30                         # 降低到 30fps
BITRATE=1500                   # 1.5Mbps
TARGET_IP="100.81.46.44"
LEFT_PORT=5004
RIGHT_PORT=5006

echo -e "${BLUE}========== 双目平衡配置推流 ==========${NC}"
echo ""
echo -e "${GREEN}[INFO]${NC} 配置：${WIDTH}x${HEIGHT} @ ${FPS}fps, 1.5Mbps"
echo -e "${GREEN}[INFO]${NC} 适合网络抖动较大的环境"
echo ""

if [ ! -e "$LEFT_DEVICE" ] || [ ! -e "$RIGHT_DEVICE" ]; then
    echo -e "${RED}[ERROR]${NC} 相机设备不存在"
    exit 1
fi

trap cleanup INT TERM
cleanup() {
    echo ""
    kill $LEFT_PID $RIGHT_PID 2>/dev/null
    exit 0
}

# 使用 low-latency-hp 预设
echo -e "${BLUE}[CAM]${NC} 启动双目推流..."
gst-launch-1.0 \
    v4l2src device=$LEFT_DEVICE do-timestamp=true ! \
    video/x-raw,format=YUY2,width=$WIDTH,height=$HEIGHT,framerate=$FPS/1 ! \
    videoconvert ! video/x-raw,format=I420 ! \
    nvh264enc bitrate=$BITRATE preset=5 rc-mode=cbr zerolatency=true ! \
    rtph264pay config-interval=1 ! \
    udpsink host=$TARGET_IP port=$LEFT_PORT sync=false > /dev/null 2>&1 &

LEFT_PID=$!

gst-launch-1.0 \
    v4l2src device=$RIGHT_DEVICE do-timestamp=true ! \
    video/x-raw,format=YUY2,width=$WIDTH,height=$HEIGHT,framerate=$FPS/1 ! \
    videoconvert ! video/x-raw,format=I420 ! \
    nvh264enc bitrate=$BITRATE preset=5 rc-mode=cbr zerolatency=true ! \
    rtph264pay config-interval=1 ! \
    udpsink host=$TARGET_IP port=$RIGHT_PORT sync=false > /dev/null 2>&1 &

RIGHT_PID=$!

sleep 3
if ! kill -0 $LEFT_PID 2>/dev/null || ! kill -0 $RIGHT_PID 2>/dev/null; then
    echo -e "${RED}[ERROR]${NC} 推流启动失败"
    exit 1
fi

echo -e "${GREEN}[INFO]${NC} 推流运行中"
echo -e "${BLUE}[CAM]${NC} 左: ${TARGET_IP}:${LEFT_PORT}"
echo -e "${BLUE}[CAM]${NC} 右: ${TARGET_IP}:${RIGHT_PORT}"
echo -e "${GREEN}[INFO]${NC} 按 Ctrl+C 停止"

wait
