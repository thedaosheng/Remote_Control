#!/bin/bash
#
# 停止 WebRTC 推流
#
export TERM=xterm-256color

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

clear
echo -e "${RED}╔════════════════════════════════════════╗${NC}"
echo -e "${RED}║     停止 WebRTC 推流                  ║${NC}"
echo -e "${RED}╚════════════════════════════════════════╝${NC}"
echo ""

pkill -9 -f "gst-launch.*video4" 2>/dev/null && echo "✓ 停止左相机推流"
pkill -9 -f "gst-launch.*video10" 2>/dev/null && echo "✓ 停止右相机推流"
pkill -9 -f "python3.*websockets" 2>/dev/null && echo "✓ 停止信令客户端"

echo ""
echo -e "${GREEN}[✓] 所有推流已停止${NC}"
echo ""
echo "按回车键关闭窗口..."
read
