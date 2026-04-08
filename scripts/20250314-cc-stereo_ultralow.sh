#!/bin/bash
# =============================================================================
# RealSense 双目超低延迟推流脚本（无编码版本）
# =============================================================================
# 直接推原始 YUV 数据，完全消除编解码延迟
# 带宽要求：640x480@60fps YUV ≈ 18Mbps/路
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

# 配置
LEFT_DEVICE="/dev/video4"
RIGHT_DEVICE="/dev/video16"
WIDTH=640
HEIGHT=480
FPS=60
TARGET_IP="100.81.46.44"
LEFT_PORT=5004
RIGHT_PORT=5006

echo -e "${BLUE}========== 超低延迟推流（无编码）==========${NC}"
echo ""
echo -e "${YELLOW}[WARN]${NC} 此脚本直接推原始 YUV，无编解码延迟"
echo -e "${YELLOW}[WARN]${NC} 带宽需求：约 18Mbps/路 (共 36Mbps)"
echo ""

# 检查设备
if [ ! -e "$LEFT_DEVICE" ] || [ ! -e "$RIGHT_DEVICE" ]; then
    echo -e "${RED}[ERROR]${NC} 相机设备不存在"
    exit 1
fi

echo -e "${GREEN}[INFO]${NC} 配置：${WIDTH}x${HEIGHT} @ ${FPS}fps (原始 YUYV)"
echo ""

trap cleanup INT TERM

cleanup() {
    echo ""
    echo -e "${YELLOW}[INFO]${NC} 停止推流..."
    kill $LEFT_PID $RIGHT_PID 2>/dev/null
    exit 0
}

# 启动左相机 - 直接推 YUYV，无编码
echo -e "${BLUE}[CAM]${NC} 启动左相机..."
gst-launch-1.0 \
    v4l2src device=$LEFT_DEVICE do-timestamp=true ! \
    video/x-raw,format=YUY2,width=$WIDTH,height=$HEIGHT,framerate=$FPS/1 ! \
    udpsink host=$TARGET_IP port=$LEFT_PORT sync=false > /dev/null 2>&1 &

LEFT_PID=$!

# 启动右相机
echo -e "${BLUE}[CAM]${NC} 启动右相机..."
gst-launch-1.0 \
    v4l2src device=$RIGHT_DEVICE do-timestamp=true ! \
    video/x-raw,format=YUY2,width=$WIDTH,height=$HEIGHT,framerate=$FPS/1 ! \
    udpsink host=$TARGET_IP port=$RIGHT_PORT sync=false > /dev/null 2>&1 &

RIGHT_PID=$!

sleep 3

if ! kill -0 $LEFT_PID 2>/dev/null || ! kill -0 $RIGHT_PID 2>/dev/null; then
    echo -e "${RED}[ERROR]${NC} 推流启动失败"
    exit 1
fi

echo -e "${GREEN}[INFO]${NC} ========== 超低延迟推流运行中 =========="
echo -e "${BLUE}[CAM]${NC} 左相机: $LEFT_DEVICE -> ${TARGET_IP}:${LEFT_PORT}"
echo -e "${BLUE}[CAM]${NC} 右相机: $RIGHT_DEVICE -> ${TARGET_IP}:${RIGHT_PORT}"
echo ""
echo -e "${GREEN}[INFO]${NC} 预估延迟：仅网络传输 (~5-40ms)"
echo -e "${GREEN}[INFO]${NC} 按 Ctrl+C 停止"

wait
