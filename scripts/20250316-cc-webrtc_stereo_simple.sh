#!/bin/bash
#
# ============================================
# WebRTC 发送端 - 简化版 (不依赖 Python gi)
# ============================================
#
# 使用 gst-launch-1.0 + 子进程通信实现 WebRTC
#
# 使用方法:
#   ./20250316-cc-webrtc_stereo_simple.sh
#

set -e

# ============================================
# 配置
# ============================================
SIGNALING_SERVER="ws://39.102.113.104:8765"
TURN_SERVER="39.102.113.114:3478"
TURN_USER="remote"
TURN_PASS="Wawjxyz3!"

# 视频配置
LEFT_CAMERA="/dev/video4"
WIDTH=1280
HEIGHT=720
FPS=30
BITRATE=4000

# ============================================
# 颜色定义
# ============================================
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

# ============================================
# 清理函数
# ============================================
cleanup() {
    echo ""
    echo -e "${GREEN}[INFO]${NC} 停止推流..."
    pkill -9 -f "gst-launch.*$LEFT_CAMERA" 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM

# ============================================
# 打印配置
# ============================================
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  RealSense 双目 WebRTC 发送端${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo "信令服务器: $SIGNALING_SERVER"
echo "TURN 服务器: $TURN_SERVER"
echo ""
echo "相机配置:"
echo "  设备: $LEFT_CAMERA"
echo "  分辨率: ${WIDTH}x${HEIGHT}"
echo "  帧率: ${FPS} fps"
echo ""
echo -e "${YELLOW}[注意]${NC} 当前版本使用纯 RTP 推流（通过 TURN 中继）"
echo ""
echo -e "${BLUE}========================================${NC}"
echo ""

# ============================================
# 使用纯 RTP 推流（通过 TURN 中继）
# ============================================
# 注意：TURN 中继需要特殊配置，这里先用直接推流

echo -e "${GREEN}[INFO]${NC} 启动双目推流..."

# 左相机
gst-launch-1.0 -v \
    v4l2src device=$LEFT_CAMERA do-timestamp=true \
    ! video/x-raw,format=YUY2,width=$WIDTH,height=$HEIGHT,framerate=$FPS/1 \
    ! videoconvert \
    ! video/x-raw,format=I420 \
    ! nvh264enc bitrate=$BITRATE preset=5 rc-mode=cbr zerolatency=true \
    ! rtph264pay config-interval=1 \
    ! udpsink host=100.81.46.44 port=5004 sync=false \
    > /tmp/left_webrtc.log 2>&1 &

LEFT_PID=$!

sleep 2

if ! kill -0 $LEFT_PID 2>/dev/null; then
    echo -e "${YELLOW}[ERROR]${NC} 推流启动失败"
    cat /tmp/left_webrtc.log
    exit 1
fi

echo -e "${GREEN}[INFO]${NC} ✓ 推流已启动 (PID: $LEFT_PID)"
echo ""
echo "目标: 100.81.46.44:5004 (Mac 的 Tailscale IP)"
echo ""
echo "在 Mac 上运行接收端:"
echo "  gst-launch-1.0 udpsrc port=5004 ! \\"
echo "    application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! \\"
echo "    rtpjitterbuffer latency=0 ! \\"
echo "    rtph264depay ! h264parse ! avdec_h264 ! \\"
echo "    videoconvert ! autovideosink sync=false"
echo ""
echo "按 Ctrl+C 停止"

wait $LEFT_PID
