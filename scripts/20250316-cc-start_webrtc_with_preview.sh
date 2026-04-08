#!/bin/bash
#
# RealSense 双目 WebRTC 推流（带本地预览）
#

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

cleanup() {
    echo ""
    echo -e "${YELLOW}[!] 停止推流...${NC}"
    pkill -9 -f "webrtc_sender_hybrid.py" 2>/dev/null || true
    pkill -9 -f "gst-launch.*video4" 2>/dev/null || true
    pkill -9 -f "gst-launch.*video16" 2>/dev/null || true
    echo -e "${GREEN}[✓] 已停止${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

clear
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  RealSense 双目 WebRTC 推流${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${GREEN}[*] 云端服务器: 39.102.113.104:8765${NC}"
echo -e "${GREEN}[*] 本地预览: 开启${NC}"
echo ""
echo "等待 Mac 接收端连接..."
echo "按 Ctrl+C 停止"
echo -e "${BLUE}========================================${NC}"
echo ""

# 启动本地预览窗口（左相机）
gst-launch-1.0 \
    v4l2src device=/dev/video4 do-timestamp=true \
    ! video/x-raw,format=YUY2,width=1280,height=720,framerate=30/1 \
    ! videoconvert \
    ! autovideosink sync=false \
    > /tmp/preview.log 2>&1 &

PREVIEW_PID=$!
sleep 2

# 启动 WebRTC 发送端
cd ~/teleop/scripts
python3 20250316-cc-webrtc_sender_hybrid.py 2>&1 | tee -a /tmp/webrtc_sender.log

# 清理
cleanup
