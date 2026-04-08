#!/bin/bash
#
# RealSense 双目 WebRTC 推流（双预览窗口 + 云端连接）
#
export TERM=xterm-256color

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

cleanup() {
    echo ""
    echo -e "${YELLOW}[!] 停止推流...${NC}"
    pkill -9 -f "gst-launch.*video4" 2>/dev/null
    pkill -9 -f "gst-launch.*video10" 2>/dev/null
    pkill -f "gst-launch.*preview" 2>/dev/null
    pkill -f "webrtc_signaling_client" 2>/dev/null
    sleep 1
    echo -e "${GREEN}[✓] 已停止${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

clear
echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║   RealSense 双目 WebRTC 推流           ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════╝${NC}"
echo ""

# 检查设备
if [ ! -e "/dev/video4" ]; then
    echo -e "${RED}[错误] 左相机 /dev/video4 不存在!${NC}"
    read -p "按回车退出..."
    exit 1
fi

if [ ! -e "/dev/video10" ]; then
    echo -e "${RED}[错误] 右相机 /dev/video10 不存在!${NC}"
    read -p "按回车退出..."
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 启动左相机：预览 + 推流（使用 tee 分流）
echo -e "${GREEN}[1/3] 启动左相机（预览 + 推流）...${NC}"
gst-launch-1.0 \
    v4l2src device=/dev/video4 do-timestamp=true \
    ! video/x-raw,format=YUY2,width=1280,height=720,framerate=30/1 \
    ! videoconvert \
    ! tee name=t \
    t. ! queue \
    ! autovideosink sync=false \
    t. ! queue \
    ! video/x-raw,format=I420 \
    ! nvh264enc bitrate=4000 preset=5 rc-mode=cbr zerolatency=true \
    ! rtph264pay config-interval=1 \
    ! udpsink host=39.102.113.104 port=51004 sync=false \
    > /tmp/left_full.log 2>&1 &
LEFT_PID=$!

# 启动右相机：预览 + 推流（使用 tee 分流）
echo -e "${GREEN}[2/3] 启动右相机（预览 + 推流）...${NC}"
gst-launch-1.0 \
    v4l2src device=/dev/video10 do-timestamp=true \
    ! video/x-raw,format=YUY2,width=1280,height=720,framerate=30/1 \
    ! videoconvert \
    ! tee name=t \
    t. ! queue \
    ! autovideosink sync=false \
    t. ! queue \
    ! video/x-raw,format=I420 \
    ! nvh264enc bitrate=4000 preset=5 rc-mode=cbr zerolatency=true \
    ! rtph264pay config-interval=1 \
    ! udpsink host=39.102.113.104 port=51006 sync=false \
    > /tmp/right_full.log 2>&1 &
RIGHT_PID=$!

sleep 2
echo -e "  ${GREEN}✓ 双目预览已打开${NC}"
echo -e "  ${GREEN}✓ 双目推流已启动${NC}"

# 启动信令客户端
echo -e "${GREEN}[3/3] 连接云端...${NC}"
/usr/bin/python3 "$SCRIPT_DIR/20250316-cc-webrtc_signaling_client.py" 2>&1 &
SIGNAL_PID=$!

sleep 3

echo ""
echo -e "${GREEN}====================${NC}"
echo -e "${GREEN}   推流运行中${NC}"
echo -e "${GREEN}====================${NC}"
echo ""
echo "📺 本地预览: 两个窗口（左相机 + 右相机）"
echo "☁️  云端推流: 39.102.113.104:51004 (左) / 51006 (右)"
echo ""
echo -e "${YELLOW}按 Ctrl+C 停止推流${NC}"
echo ""

wait
