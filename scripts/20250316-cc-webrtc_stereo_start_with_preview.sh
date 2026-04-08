#!/bin/bash
#
# RealSense 双目 WebRTC 推流（带本地预览窗口）
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
    pkill -9 -f "gst-launch.*preview" 2>/dev/null
    pkill -9 -f "webrtc_signaling_client" 2>/dev/null
    sleep 1
    echo -e "${GREEN}[✓] 已停止${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

clear
echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║   RealSense 双目 WebRTC 推流 (带预览)   ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════╝${NC}"
echo ""

# 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 启动本地预览窗口（左右分屏）
echo -e "${GREEN}[*] 启动本地预览窗口...${NC}"
gst-launch-1.0 \
    v4l2src device=/dev/video4 do-timestamp=true \
    ! video/x-raw,format=YUY2,width=640,height=360,framerate=30/1 \
    ! videoconvert \
    ! queue \
    ! videomixer name=mixer sink_0::xpos=0 sink_0::ypos=0 \
    v4l2src device=/dev/video10 do-timestamp=true \
    ! video/x-raw,format=YUY2,width=640,height=360,framerate=30/1 \
    ! videoconvert \
    ! queue \
    ! mixer. \
    mixer. \
    ! videoconvert \
    ! autovideosink sync=false \
    > /tmp/preview.log 2>&1 &
PREVIEW_PID=$!
sleep 2

if ps -p $PREVIEW_PID >/dev/null 2>&1; then
    echo -e "  ${GREEN}✓ 预览窗口已打开（左右分屏）${NC}"
else
    echo -e "  ${YELLOW}⚠ 预览窗口启动失败，继续推流...${NC}"
fi

# 启动推流
echo -e "${GREEN}[*] 启动双目推流...${NC}"
gst-launch-1.0 \
    v4l2src device=/dev/video4 do-timestamp=true \
    ! video/x-raw,format=YUY2,width=1280,height=720,framerate=30/1 \
    ! videoconvert \
    ! video/x-raw,format=I420 \
    ! nvh264enc bitrate=4000 preset=5 rc-mode=cbr zerolatency=true \
    ! rtph264pay config-interval=1 \
    ! udpsink host=39.102.113.104 port=51004 sync=false \
    > /tmp/left_stream.log 2>&1 &
LEFT_PID=$!

gst-launch-1.0 \
    v4l2src device=/dev/video10 do-timestamp=true \
    ! video/x-raw,format=YUY2,width=1280,height=720,framerate=30/1 \
    ! videoconvert \
    ! video/x-raw,format=I420 \
    ! nvh264enc bitrate=4000 preset=5 rc-mode=cbr zerolatency=true \
    ! rtph264pay config-interval=1 \
    ! udpsink host=39.102.113.104 port=51006 sync=false \
    > /tmp/right_stream.log 2>&1 &
RIGHT_PID=$!

sleep 2
echo -e "  ${GREEN}✓ 双目推流已启动${NC}"

# 启动信令客户端
echo -e "${GREEN}[*] 连接云端信令服务器...${NC}"
python3 "$SCRIPT_DIR/20250316-cc-webrtc_signaling_client.py" >/dev/null 2>&1 &
SIGNAL_PID=$!

sleep 2
echo ""

echo -e "${GREEN}====================${NC}"
echo -e "${GREEN}   推流运行中${NC}"
echo -e "${GREEN}====================${NC}"
echo ""
echo "本地: 桌面预览窗口 (左右分屏)"
echo "云端: 39.102.113.104:51004 (左)"
echo "      39.102.113.104:51006 (右)"
echo ""
echo -e "${YELLOW}按 Ctrl+C 停止推流${NC}"
echo ""

wait
