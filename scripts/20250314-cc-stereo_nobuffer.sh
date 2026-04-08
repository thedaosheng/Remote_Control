#!/bin/bash
# =============================================================================
# RealSense 双目零缓冲推流脚本
# =============================================================================
# 优化参数以最小化发送端缓冲，降低端到端延迟
#
# 作者: Claude Code
# 日期: 2025-03-14
# =============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

LEFT_DEVICE="/dev/video4"
RIGHT_DEVICE="/dev/video16"
WIDTH=1280
HEIGHT=720
FPS=30
BITRATE=4000
TARGET_IP="$1"

if [ -z "$TARGET_IP" ]; then
    TARGET_IP="100.81.46.44"
fi

echo -e "${BLUE}========== 零缓冲低延迟推流 ==========${NC}"
echo ""
echo -e "${GREEN}[INFO]${NC} 配置：${WIDTH}x${HEIGHT} @ ${FPS}fps"
echo -e "${GREEN}[INFO]${NC} 目标：${TARGET_IP}"
echo ""

trap cleanup INT TERM
cleanup() {
    kill $LEFT_PID $RIGHT_PID 2>/dev/null
    exit 0
}

# 关键优化：
# 1. max-lateness=0 - 不允许延迟
# 2. sync=false - 不同步时钟
# 3. qos=false - 关闭 QoS（不丢弃旧帧）
# 4. ts-offset=0 - 无时间戳偏移

echo -e "${BLUE}[CAM]${NC} 启动左相机..."
gst-launch-1.0 -v \
    v4l2src device=$LEFT_DEVICE do-timestamp=true ! \
    video/x-raw,format=YUY2,width=$WIDTH,height=$HEIGHT,framerate=$FPS/1 ! \
    videoconvert ! video/x-raw,format=I420 ! \
    nvh264enc preset=5 zerolatency=true bitrate=$BITRATE ! \
    rtph264pay config-interval=1 pt=96 ! \
    udpsink host=$TARGET_IP port=5004 sync=false max-lateness=0 qos=false \
    > /tmp/left_nobuf.log 2>&1 &

LEFT_PID=$!

echo -e "${BLUE}[CAM]${NC} 启动右相机..."
gst-launch-1.0 -v \
    v4l2src device=$RIGHT_DEVICE do-timestamp=true ! \
    video/x-raw,format=YUY2,width=$WIDTH,height=$HEIGHT,framerate=$FPS/1 ! \
    videoconvert ! video/x-raw,format=I420 ! \
    nvh264enc preset=5 zerolatency=true bitrate=$BITRATE ! \
    rtph264pay config-interval=1 pt=96 ! \
    udpsink host=$TARGET_IP port=5006 sync=false max-lateness=0 qos=false \
    > /tmp/right_nobuf.log 2>&1 &

RIGHT_PID=$!

sleep 3
if ! kill -0 $LEFT_PID 2>/dev/null || ! kill -0 $RIGHT_PID 2>/dev/null; then
    echo -e "${RED}[ERROR]${NC} 推流启动失败"
    exit 1
fi

echo -e "${GREEN}[INFO]${NC} ========== 零缓冲推流运行中 =========="
echo -e "${BLUE}[CAM]${NC} 左: ${TARGET_IP}:5004"
echo -e "${BLUE}[CAM]${NC} 右: ${TARGET_IP}:5006"
echo ""
echo -e "${GREEN}[INFO]${NC} 按 Ctrl+C 停止"

wait
