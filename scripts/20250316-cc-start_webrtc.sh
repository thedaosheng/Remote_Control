#!/bin/bash
#
# ============================================
#   RealSense 双目 WebRTC 推流启动脚本
# ============================================
#
# 功能: 一键启动 WebRTC 发送端，连接云端信令服务器
#
# 使用方法: ./20250316-cc-start_webrtc.sh
#
# 作者: Claude Code
# 日期: 2025-03-16
#

set -e

# ============================================
# 配置
# ============================================
SCRIPT_DIR="$HOME/teleop/scripts"
SENDER_SCRIPT="$SCRIPT_DIR/20250316-cc-webrtc_sender_hybrid.py"
LOG_FILE="/tmp/webrtc_sender.log"

# ============================================
# 颜色
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
    echo -e "${YELLOW}[!] 停止推流...${NC}"
    pkill -9 -f "webrtc_sender_hybrid.py" 2>/dev/null || true
    pkill -9 -f "gst-launch.*video4" 2>/dev/null || true
    echo -e "${GREEN}[✓] 已停止${NC}"
    exit 0
}

# 注册 Ctrl+C
trap cleanup SIGINT SIGTERM

# ============================================
# 启动
# ============================================
clear
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  RealSense 双目 WebRTC 推流${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${GREEN}[*] 云端服务器: 39.102.113.104:8765${NC}"
echo -e "${GREEN}[*] TURN 服务器: 39.102.113.104:3478${NC}"
echo ""
echo "等待 Mac 接收端连接..."
echo "按 Ctrl+C 停止"
echo -e "${BLUE}========================================${NC}"
echo ""

# 启动发送端
cd "$SCRIPT_DIR"
python3 "$SENDER_SCRIPT" 2>&1 | tee -a "$LOG_FILE"

# 如果到这里说明异常退出
cleanup
