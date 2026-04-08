#!/bin/bash
#
# RealSense 双目 RTP 推流脚本
# 功能: 同时从两个 RealSense 相机采集视频，独立编码后通过 RTP 推流
#       左相机 -> UDP 5004, 右相机 -> UDP 5006
#
# 使用方法: ./20250314-cc-start_stereo_stream.sh [目标IP地址]
# 示例: ./20250314-cc-start_stereo_stream.sh 100.81.46.44
#
# 作者: Claude Code
# 日期: 2025-03-14
#

set -e

# ============================================
# 配置参数
# ============================================
# 左相机视频设备 (第一个 RealSense D435 的彩色流)
LEFT_CAMERA="/dev/video4"
# 右相机视频设备 (第三个 RealSense D435 的彩色流)
RIGHT_CAMERA="/dev/video16"

# 视频参数
WIDTH=1280
HEIGHT=720
FPS=30

# 编码参数
# 注意: nvh264enc 的 bitrate 单位是 kbit/sec，不是 bits/sec
BITRATE=4000  # 4Mbps 每路 (4000 kbit/sec = 4Mbps)

# RTP 目标地址
DEST_IP=${1:-"100.81.46.44"}
LEFT_PORT=5004   # 左相机端口
RIGHT_PORT=5006  # 右相机端口

# ============================================
# 颜色定义
# ============================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ============================================
# 函数定义
# ============================================

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_camera() {
    echo -e "${BLUE}[CAM]${NC} $1"
}

# 清理函数: 停止所有后台推流进程
cleanup() {
    echo ""
    print_info "========== 停止推流 =========="
    print_info "正在停止所有推流进程..."

    # 杀死所有后台的 gst-launch 进程
    if [ -n "$LEFT_PID" ]; then
        kill $LEFT_PID 2>/dev/null || true
        print_camera "左相机进程已停止"
    fi

    if [ -n "$RIGHT_PID" ]; then
        kill $RIGHT_PID 2>/dev/null || true
        print_camera "右相机进程已停止"
    fi

    # 等待进程结束
    wait $LEFT_PID 2>/dev/null || true
    wait $RIGHT_PID 2>/dev/null || true

    print_info "所有推流已停止"
    exit 0
}

# 注册信号处理 (Ctrl+C)
trap cleanup SIGINT SIGTERM

# ============================================
# 依赖检查
# ============================================
print_info "========== 检查依赖 =========="

# 检查视频设备
if [ ! -e "$LEFT_CAMERA" ]; then
    print_error "左相机设备 $LEFT_CAMERA 不存在!"
    exit 1
fi
print_camera "左相机: $LEFT_CAMERA ✓"

if [ ! -e "$RIGHT_CAMERA" ]; then
    print_error "右相机设备 $RIGHT_CAMERA 不存在!"
    exit 1
fi
print_camera "右相机: $RIGHT_CAMERA ✓"

# 检查 GStreamer
if ! command -v gst-launch-1.0 &> /dev/null; then
    print_error "gst-launch-1.0 未安装!"
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
# preset=5: low-latency-hp (低延迟高性能) - 专为实时应用设计
if gst-inspect-1.0 nvh264enc &> /dev/null; then
    ENCODER="nvh264enc bitrate=$BITRATE preset=5 rc-mode=cbr zerolatency=true"
    ENCODER_NAME="NVENC (NVIDIA GPU - Low Latency HP)"
    print_info "使用 NVENC 硬件编码 (低延迟模式) ✓"
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
echo "  编码器:     $ENCODER_NAME"
echo "  分辨率:     ${WIDTH}x${HEIGHT} @ ${FPS}fps"
echo "  目标地址:   $DEST_IP"
echo ""
print_camera "左相机: $LEFT_CAMERA -> ${DEST_IP}:${LEFT_PORT}"
print_camera "右相机: $RIGHT_CAMERA -> ${DEST_IP}:${RIGHT_PORT}"
echo ""

# ============================================
# 启动双目推流
# ============================================
print_info "========== 启动双目推流 =========="
print_info "按 Ctrl+C 同时停止两路推流"
print_info "等待视频流启动..."
echo ""

# 左相机推流命令 (后台运行)
LEFT_CMD="gst-launch-1.0 -v \
    v4l2src device=$LEFT_CAMERA \
    ! video/x-raw,format=YUY2,width=$WIDTH,height=$HEIGHT,framerate=$FPS/1 \
    ! videoconvert \
    ! $ENCODER \
    ! rtph264pay config-interval=1 \
    ! udpsink host=$DEST_IP port=$LEFT_PORT sync=false"

# 右相机推流命令 (后台运行)
RIGHT_CMD="gst-launch-1.0 -v \
    v4l2src device=$RIGHT_CAMERA \
    ! video/x-raw,format=YUY2,width=$WIDTH,height=$HEIGHT,framerate=$FPS/1 \
    ! videoconvert \
    ! $ENCODER \
    ! rtph264pay config-interval=1 \
    ! udpsink host=$DEST_IP port=$RIGHT_PORT sync=false"

# 启动左相机推流 (后台，输出重定向到日志文件)
print_camera "启动左相机推流..."
eval "$LEFT_CMD" > /tmp/left_camera.log 2>&1 &
LEFT_PID=$!

# 启动右相机推流 (后台，输出重定向到日志文件)
print_camera "启动右相机推流..."
eval "$RIGHT_CMD" > /tmp/right_camera.log 2>&1 &
RIGHT_PID=$!

# 短暂等待，让进程启动
sleep 2

# 检查进程是否还在运行
if ! kill -0 $LEFT_PID 2>/dev/null; then
    print_error "左相机推流启动失败!"
    print_info "查看日志: cat /tmp/left_camera.log"
    cleanup
    exit 1
fi

if ! kill -0 $RIGHT_PID 2>/dev/null; then
    print_error "右相机推流启动失败!"
    print_info "查看日志: cat /tmp/right_camera.log"
    cleanup
    exit 1
fi

print_info "========== 双目推流运行中 =========="
print_camera "左相机 (PID: $LEFT_PID) -> ${DEST_IP}:${LEFT_PORT}"
print_camera "右相机 (PID: $RIGHT_PID) -> ${DEST_IP}:${RIGHT_PORT}"
echo ""
print_info "日志文件:"
print_info "  左相机: /tmp/left_camera.log"
print_info "  右相机: /tmp/right_camera.log"
echo ""
print_info "按 Ctrl+C 停止推流"

# 等待任意一个进程结束 (或被 Ctrl+C 中断)
wait $LEFT_PID $RIGHT_PID

# 如果到这里说明有进程异常退出
print_warn "检测到推流进程异常退出"
cleanup
