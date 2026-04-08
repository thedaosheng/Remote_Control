#!/bin/bash
#
# 停止 WebRTC 推流
#

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

echo -e "${RED}========================================${NC}"
echo -e "${RED}  停止 WebRTC 推流${NC}"
echo -e "${RED}========================================${NC}"
echo ""

# 停止所有相关进程
pkill -9 -f "webrtc_sender_hybrid.py" 2>/dev/null && echo "✓ 停止发送端"
pkill -9 -f "gst-launch.*video4" 2>/dev/null && echo "✓ 停止左相机"
pkill -9 -f "gst-launch.*video16" 2>/dev/null && echo "✓ 停止右相机"
pkill -9 -f "webrtc_test" 2>/dev/null && echo "✓ 停止测试进程"

echo ""
echo -e "${GREEN}[✓] 所有推流已停止${NC}"
