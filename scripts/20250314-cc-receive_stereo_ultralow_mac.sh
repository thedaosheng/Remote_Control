#!/bin/bash
#
# ============================================
# 超低延迟双目接收端脚本 (Mac)
# ============================================
#
# 关键优化：
#   - rtpjitterbuffer latency=0  # 禁用抖动缓冲（最重要！）
#   - sync=false                 # 不同步时钟，立即显示
#
# 使用方法: ./receive_stereo_ultralow.sh
#
# 作者: Claude Code
# 日期: 2025-03-14
#

set -e

# ============================================
# 配置参数
# ============================================
LEFT_PORT=5004
RIGHT_PORT=5006
WIDTH=1280
HEIGHT=720

# ============================================
# 颜色定义
# ============================================
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

# ============================================
# 函数定义
# ============================================
cleanup() {
    echo ""
    echo -e "${GREEN}[INFO]${NC} 停止接收..."
    kill $LEFT_PID $RIGHT_PID 2>/dev/null
    wait
    exit 0
}

trap cleanup SIGINT SIGTERM

# ============================================
# 启动接收
# ============================================
echo -e "${BLUE}=========================================${NC}"
echo -e "${BLUE}     超低延迟双目接收端${NC}"
echo -e "${BLUE}=========================================${NC}"
echo ""
echo "左相机: UDP $LEFT_PORT"
echo "右相机: UDP $RIGHT_PORT"
echo "分辨率: ${WIDTH}x${HEIGHT} (左右独立窗口)"
echo ""
echo "按 Ctrl+C 停止"
echo -e "${BLUE}=========================================${NC}"
echo ""

# ============================================
# 左相机接收
# ============================================
echo -e "${GREEN}[INFO]${NC} 启动左相机接收..."

gst-launch-1.0 -v \
    udpsrc port=$LEFT_PORT ! \
    "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264" ! \
    rtpjitterbuffer latency=0 ! \
    rtph264depay ! \
    h264parse ! \
    avdec_h264 ! \
    videoconvert ! \
    autovideosink sync=false \
    > /tmp/left_recv.log 2>&1 &

LEFT_PID=$!

# ============================================
# 右相机接收
# ============================================
echo -e "${GREEN}[INFO]${NC} 启动右相机接收..."

gst-launch-1.0 -v \
    udpsrc port=$RIGHT_PORT ! \
    "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264" ! \
    rtpjitterbuffer latency=0 ! \
    rtph264depay ! \
    h264parse ! \
    avdec_h264 ! \
    videoconvert ! \
    autovideosink sync=false \
    > /tmp/right_recv.log 2>&1 &

RIGHT_PID=$!

sleep 2

echo -e "${GREEN}[INFO]${NC} ========== 接收端运行中 =========="
echo ""
echo -e "${GREEN}[INFO]${NC} 预估延迟: ~50ms (采集5ms + 编码5ms + 网络30ms + 解码10ms)"
echo ""
echo -e "${GREEN}[INFO]${NC} 左相机窗口应该已打开"
echo -e "${GREEN}[INFO]${NC} 右相机窗口应该已打开"
echo ""
echo -e "${GREEN}[INFO]${NC} 按 Ctrl+C 停止"

# 等待进程
wait $LEFT_PID $RIGHT_PID
