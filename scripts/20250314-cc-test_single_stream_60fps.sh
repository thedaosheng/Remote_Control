#!/bin/bash
#
# RealSense 单相机 60FPS RTP 推流脚本 (高帧率版本)
# 功能: 从 RealSense 相机采集 60fps 彩色视频，编码后通过 RTP 推流
#
# 使用方法: ./20250314-cc-test_single_stream_60fps.sh <目标IP地址>
# 示例: ./20250314-cc-test_single_stream_60fps.sh 100.81.46.44
#
# 注意: 60fps 模式需要使用 640x480 分辨率，这是 RealSense D435 在 60fps 下的最高分辨率
#
# 作者: Claude Code
# 日期: 2025-03-14
#

set -e  # 遇到错误立即退出

# ============================================
# 配置参数 (60FPS 版本)
# ============================================
# 视频设备节点 (RealSense D435 彩色流)
VIDEO_DEVICE="/dev/video4"

# 视频参数 - 60fps 配置
# RealSense D435 在 60fps 下的最佳分辨率选项:
# - 960x540 (推荐，最接近 720p)
# - 848x480
# - 640x480
WIDTH=960
HEIGHT=540
FPS=60

# 编码参数 (60fps 需要更高码率)
# 注意: nvh264enc 的 bitrate 单位是 kbit/sec，不是 bits/sec
BITRATE=6000  # 6Mbps (6000 kbit/sec = 6Mbps)

# RTP 目标地址 (从命令行参数获取)
DEST_IP=${1:-"100.81.46.44"}
DEST_PORT=5005  # 使用 5005 端口避免与 30fps 版本冲突

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

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_60fps() {
    echo -e "${BLUE}[60FPS]${NC} $1"
}

# 清理函数
cleanup() {
    print_info "正在停止推流..."
    jobs -p | xargs -r kill 2>/dev/null || true
    exit 0
}

trap cleanup SIGINT SIGTERM

# ============================================
# 依赖检查
# ============================================
print_60fps "========== 60FPS 高帧率推流 =========="
print_info "========== 检查依赖 =========="

if [ ! -e "$VIDEO_DEVICE" ]; then
    print_error "视频设备 $VIDEO_DEVICE 不存在!"
    exit 1
fi

# 验证设备是否支持 60fps
print_info "验证 $VIDEO_DEVICE 是否支持 ${WIDTH}x${HEIGHT} @ ${FPS}fps..."
if v4l2-ctl -d $VIDEO_DEVICE -V 2>&1 | grep -q "${WIDTH}x${HEIGHT}.*${FPS}"; then
    print_info "设备支持 60fps 模式 ✓"
else
    print_warn "无法确认设备支持 60fps，尝试继续..."
fi

if ! command -v gst-launch-1.0 &> /dev/null; then
    print_error "gst-launch-1.0 未安装!"
    exit 1
fi

# ============================================
# 编码器选择
# ============================================
print_info "========== 选择编码器 =========="

ENCODER=""
ENCODER_NAME=""

# 优先级 1: NVENC (NVIDIA GPU 硬件编码) - 最适合 60fps
if gst-inspect-1.0 nvh264enc &> /dev/null; then
    ENCODER="nvh264enc bitrate=$BITRATE preset=1 rc-mode=cbr zerolatency=true"
    ENCODER_NAME="NVENC (NVIDIA GPU)"
    print_info "使用 NVENC 硬件编码 ✓ (推荐用于 60fps)"
# 优先级 2: VAAPI (通用硬件编码)
elif gst-inspect-1.0 vaapih264enc &> /dev/null; then
    ENCODER="vaapih264enc bitrate=$BITRATE"
    ENCODER_NAME="VAAPI (硬件加速)"
    print_info "使用 VAAPI 硬件编码 ✓"
# 优先级 3: x264 (CPU 软件编码) - 60fps 下可能性能不足
elif gst-inspect-1.0 x264enc &> /dev/null; then
    ENCODER="x264enc bitrate=6000 speed-preset=superfast tune=zerolatency"
    ENCODER_NAME="x264 (CPU 编码)"
    print_warn "使用 x264 软件编码 - 60fps 可能性能不足!"
else
    print_error "没有可用的 H.264 编码器!"
    exit 1
fi

# ============================================
# 打印配置
# ============================================
print_info "========== 推流配置 =========="
echo "  视频设备:   $VIDEO_DEVICE"
echo "  分辨率:     ${WIDTH}x${HEIGHT} (60fps 最佳分辨率)"
echo "  帧率:       ${FPS} fps"
echo "  编码器:     $ENCODER_NAME"
echo "  码率:       $((BITRATE / 1000000))Mbps"
echo "  目标地址:   ${DEST_IP}:${DEST_PORT}"
echo ""

# ============================================
# 启动推流
# ============================================
print_60fps "========== 开始 60FPS 推流 =========="
print_info "按 Ctrl+C 停止推流"
echo ""

# GStreamer 管道 (60fps 优化)
gst-launch-1.0 -v \
    v4l2src device=$VIDEO_DEVICE \
    ! video/x-raw,format=YUY2,width=$WIDTH,height=$HEIGHT,framerate=$FPS/1 \
    ! videoconvert \
    ! $ENCODER \
    ! rtph264pay config-interval=1 \
    ! udpsink host=$DEST_IP port=$DEST_PORT sync=false

cleanup
