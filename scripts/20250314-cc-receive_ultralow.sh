#!/bin/bash
# =============================================================================
# 超低延迟接收端脚本（Mac）
# =============================================================================
# 关键优化：
# 1. 减小 UDP 缓冲区
# 2. 禁用抖动缓冲或设置极小值
# 3. sync=false 直接显示
# 4. max-lateness=-1 不丢弃帧
#
# 作者: Claude Code
# 日期: 2025-03-14
# =============================================================================

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========== 超低延迟接收端 ==========${NC}"
echo ""

# 用户可能需要安装: brew install gstreamer

trap cleanup INT TERM

cleanup() {
    echo ""
    echo "Stopping..."
    kill $LEFT_PID $RIGHT_PID $MERGE_PID 2>/dev/null
    wait
    exit 0
}

# 关键优化参数：
# 1. latency=0 - RTP 抖动缓冲延迟设为 0
# 2. buffer-size - UDP 缓冲区大小
# 3. sync=false - 不同步时钟
# 4. max-lateness=-1 - 不因延迟丢弃帧

echo -e "${GREEN}[INFO]${NC} 启动左相机接收..."
gst-launch-1.0 -v \
    udpsrc port=5004 buffer-size=512000 ! \
    "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96" ! \
    rtpjitterbuffer latency=0 drop-on-latency=false ! \
    rtph264depay ! \
    h264parse ! \
    avdec_h264 ! \
    videoconvert ! \
    videoscale ! \
    "video/x-raw,width=1280,height=720" ! \
    autovideosink sync=false max-lateness=-1 \
    > /tmp/left_recv.log 2>&1 &

LEFT_PID=$!

echo -e "${GREEN}[INFO]${NC} 启动右相机接收..."
gst-launch-1.0 -v \
    udpsrc port=5006 buffer-size=512000 ! \
    "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96" ! \
    rtpjitterbuffer latency=0 drop-on-latency=false ! \
    rtph264depay ! \
    h264parse ! \
    avdec_h264 ! \
    videoconvert ! \
    videoscale ! \
    "video/x-raw,width=1280,height=720" ! \
    autovideosink sync=false max-lateness=-1 \
    > /tmp/right_recv.log 2>&1 &

RIGHT_PID=$!

sleep 2
echo -e "${GREEN}[INFO]${NC} 接收端运行中"
echo -e "${GREEN}[INFO]${NC} 按 Ctrl+C 停止"

wait
