#!/bin/bash
#
# RealSense 单相机 RTP 推流脚本
# 功能: 从 RealSense 相机采集彩色视频，编码后通过 RTP 推流到目标地址
#
# 使用方法: ./20250314-cc-test_single_stream.sh <目标IP地址>
# 示例: ./20250314-cc-test_single_stream.sh 100.81.46.44
#
# 作者: Claude Code
# 日期: 2025-03-14
#

set -e  # 遇到错误立即退出

# ============================================
# 配置参数
# ============================================
# 视频设备节点 (RealSense D435 彩色流)
VIDEO_DEVICE="/dev/video4"

# 视频参数
WIDTH=1280
HEIGHT=720
FPS=30

# 编码参数
BITRATE=4000000  # 4Mbps

# RTP 目标地址 (从命令行参数获取)
DEST_IP=${1:-"100.81.46.44"}
DEST_PORT=5004

# ============================================
# 颜色定义 (用于终端输出)
# ============================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# ============================================
# 函数定义
# ============================================

# 打印带颜色的信息
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

# 清理函数: 捕获 Ctrl+C 信号后退出
cleanup() {
    print_info "正在停止推流..."
    # 清理可能残留的后台进程
    jobs -p | xargs -r kill 2>/dev/null || true
    exit 0
}

# 注册信号处理
trap cleanup SIGINT SIGTERM

# ============================================
# 依赖检查
# ============================================
print_info "========== 检查依赖 =========="

# 检查视频设备
if [ ! -e "$VIDEO_DEVICE" ]; then
    print_error "视频设备 $VIDEO_DEVICE 不存在!"
    print_info "请检查 RealSense 相机是否已连接"
    exit 1
fi
print_info "视频设备: $VIDEO_DEVICE ✓"

# 检查 v4l2-ctl
if ! command -v v4l2-ctl &> /dev/null; then
    print_warn "v4l2-ctl 未安装，无法验证设备格式"
fi

# 检查 GStreamer
if ! command -v gst-launch-1.0 &> /dev/null; then
    print_error "gst-launch-1.0 未安装!"
    print_info "请运行: sudo apt-get install gstreamer1.0-tools"
    exit 1
fi
print_info "GStreamer: $(gst-launch-1.0 --version | head -1) ✓"

# ============================================
# 编码器选择
# ============================================
print_info "========== 选择编码器 =========="

ENCODER=""
ENCODER_NAME=""

# 优先级 1: NVENC (NVIDIA GPU 硬件编码)
if gst-inspect-1.0 nvh264enc &> /dev/null; then
    ENCODER="nvh264enc bitrate=$BITRATE preset=1 rc-mode=cbr zerolatency=true"
    ENCODER_NAME="NVENC (NVIDIA GPU)"
    print_info "使用 NVENC 硬件编码 ✓"
# 优先级 2: VAAPI (通用硬件编码)
elif gst-inspect-1.0 vaapih264enc &> /dev/null; then
    ENCODER="vaapih264enc bitrate=$BITRATE"
    ENCODER_NAME="VAAPI (硬件加速)"
    print_info "使用 VAAPI 硬件编码 ✓"
# 优先级 3: x264 (CPU 软件编码)
elif gst-inspect-1.0 x264enc &> /dev/null; then
    ENCODER="x264enc bitrate=4000 speed-preset=ultrafast tune=zerolatency"
    ENCODER_NAME="x264 (CPU 编码)"
    print_info "使用 x264 软件编码 (较慢) ✓"
# 优先级 4: libav (ffmpeg 编码器)
elif gst-inspect-1.0 avenc_h264_omx &> /dev/null; then
    ENCODER="avenc_h264_omx bitrate=$BITRATE"
    ENCODER_NAME="OMX H264"
    print_info "使用 OMX H264 编码 ✓"
else
    print_error "没有可用的 H.264 编码器!"
    print_info "请安装 GStreamer 插件:"
    print_info "  sudo apt-get install gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav"
    exit 1
fi

# ============================================
# 打印配置信息
# ============================================
print_info "========== 推流配置 =========="
echo "  视频设备:   $VIDEO_DEVICE"
echo "  分辨率:     ${WIDTH}x${HEIGHT}"
echo "  帧率:       ${FPS} fps"
echo "  编码器:     $ENCODER_NAME"
echo "  目标地址:   ${DEST_IP}:${DEST_PORT}"
echo "  协议:       RTP/UDP"
echo ""

# ============================================
# 启动推流
# ============================================
print_info "========== 开始推流 =========="
print_info "按 Ctrl+C 停止推流"
print_info "等待视频流启动..."

# GStreamer 管道说明:
# v4l2src        - 从 Video4Linux 设备采集视频
# video/x-raw    - 指定视频格式 (YUYV, 宽度, 高度, 帧率)
# videoconvert   - 转换色彩空间 (YUYV -> I420)
# $ENCODER       - H.264 编码
# rtph264pay     - RTP 打包
# udpsink        - UDP 发送

gst-launch-1.0 -v \
    v4l2src device=$VIDEO_DEVICE \
    ! video/x-raw,format=YUY2,width=$WIDTH,height=$HEIGHT,framerate=$FPS/1 \
    ! videoconvert \
    ! $ENCODER \
    ! rtph264pay config-interval=1 \
    ! udpsink host=$DEST_IP port=$DEST_PORT sync=false

# 管道结束后执行清理
cleanup
