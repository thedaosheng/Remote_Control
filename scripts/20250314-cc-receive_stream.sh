#!/bin/bash
#
# RTP 流接收端脚本
# 功能: 接收 RTP 推流并显示/保存
#
# 使用方法: ./20250314-cc-receive_stream.sh [端口] [显示/保存]
# 示例:
#   ./20250314-cc-receive_stream.sh 5004        # 接收 5004 端口并显示
#   ./20250314-cc-receive_stream.sh 5004 save   # 接收并保存到文件
#
# 作者: Claude Code
# 日期: 2025-03-14
#

# ============================================
# 配置参数
# ============================================
PORT=${1:-5004}              # 默认监听端口 5004
MODE=${2:-"display"}         # 默认模式: 显示

# 监听地址 (0.0.0.0 表示监听所有网卡)
LISTEN_ADDR="0.0.0.0"
LISTEN_PORT=$PORT

# 输出文件 (保存模式用)
OUTPUT_FILE="received_stream_${PORT}.mkv"

# ============================================
# 颜色定义
# ============================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# ============================================
# 解码器选择
# ============================================
DECODER=""
DECODER_NAME=""

# 优先级 1: NVDEC (NVIDIA GPU 硬件解码)
if gst-inspect-1.0 nvh264dec &> /dev/null; then
    DECODER="nvh264dec"
    DECODER_NAME="NVDEC (NVIDIA GPU)"
# 优先级 2: VAAPI (通用硬件解码)
elif gst-inspect-1.0 vaapih264dec &> /dev/null; then
    DECODER="vaapih264dec"
    DECODER_NAME="VAAPI (硬件加速)"
# 优先级 3: 软件解码
elif gst-inspect-1.0 avdec_h264 &> /dev/null; then
    DECODER="avdec_h264"
    DECODER_NAME="软件解码 (libav)"
else
    print_error "没有可用的 H.264 解码器!"
    exit 1
fi

# ============================================
# 打印配置
# ============================================
print_info "========== RTP 接收配置 =========="
echo "  监听地址:   ${LISTEN_ADDR}:${LISTEN_PORT}"
echo "  解码器:     $DECODER_NAME"
if [ "$MODE" = "save" ]; then
    echo "  模式:       保存到文件"
    echo "  输出文件:   $OUTPUT_FILE"
else
    echo "  模式:       实时显示"
fi
echo ""

# ============================================
# 启动接收
# ============================================
print_info "开始接收 RTP 流..."
print_info "按 Ctrl+C 停止"
echo ""

if [ "$MODE" = "save" ]; then
    # 保存模式: 接收并保存到文件
    print_info "正在保存到: $OUTPUT_FILE"
    gst-launch-1.0 -v \
        udpsrc port=$LISTEN_PORT \
        ! application/x-rtp,media=video,encoding-name=H264 \
        ! rtph264depay \
        ! $DECODER \
        ! videoconvert \
        ! matroskamux \
        ! filesink location=$OUTPUT_FILE
else
    # 显示模式: 接收并显示 (需要 ffmpeg 或其他显示工具)
    # 方案1: 使用 autovideosink (自动选择视频输出)
    print_info "正在显示视频..."
    gst-launch-1.0 -v \
        udpsrc port=$LISTEN_PORT \
        ! application/x-rtp,media=video,encoding-name=H264 \
        ! rtph264depay \
        ! $DECODER \
        ! videoconvert \
        ! autovideosink sync=false
fi
