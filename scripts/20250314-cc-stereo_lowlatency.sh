#!/bin/bash
# =============================================================================
# RealSense 双目彩色低延迟推流脚本
# =============================================================================
# 降低分辨率、使用 YUV 4:2:2 (16-bit) 格式、提高帧率以获得最低延迟
#
# 作者: Claude Code
# 日期: 2025-03-14
# =============================================================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 默认配置 - 低延迟优化
LEFT_DEVICE="/dev/video4"       # 左彩色相机 (YUYV 16-bit)
RIGHT_DEVICE="/dev/video16"     # 右彩色相机 (YUYV 16-bit)
WIDTH=640
HEIGHT=480
FPS=60                         # 640x480 支持 60fps
BITRATE=2000                   # 2Mbps (降低码率减少编码延迟)
TARGET_IP="100.81.46.44"
LEFT_PORT=5004
RIGHT_PORT=5006

# 日志文件
LEFT_LOG="/tmp/left_lowlatency.log"
RIGHT_LOG="/tmp/right_lowlatency.log"

# =============================================================================
# 脚本开始
# =============================================================================

echo -e "${BLUE}========== 双目彩色低延迟推流 ==========${NC}"
echo ""

# 检查参数
if [ -n "$1" ]; then
    TARGET_IP="$1"
fi

# -----------------------------------------------------------------------------
# 检查依赖
# -----------------------------------------------------------------------------
echo -e "${GREEN}[INFO]${NC} ========== 检查依赖 =========="

# 检查设备
if [ ! -e "$LEFT_DEVICE" ]; then
    echo -e "${RED}[ERROR]${NC} 左相机设备不存在: $LEFT_DEVICE"
    exit 1
fi
echo -e "${BLUE}[CAM]${NC} 左彩色相机: $LEFT_DEVICE ✓"

if [ ! -e "$RIGHT_DEVICE" ]; then
    echo -e "${RED}[ERROR]${NC} 右相机设备不存在: $RIGHT_DEVICE"
    exit 1
fi
echo -e "${BLUE}[CAM]${NC} 右彩色相机: $RIGHT_DEVICE ✓"

# 检查 GStreamer
if ! command -v gst-launch-1.0 &> /dev/null; then
    echo -e "${RED}[ERROR]${NC} 未找到 gst-launch-1.0"
    exit 1
fi
GST_VERSION=$(gst-launch-1.0 --version | head -1)
echo -e "${GREEN}[INFO]${NC} GStreamer: $GST_VERSION ✓"

# -----------------------------------------------------------------------------
# 选择编码器
# -----------------------------------------------------------------------------
echo -e "${GREEN}[INFO]${NC} ========== 选择编码器 =========="

ENCODER="nvh264enc"
ENCODER_NAME="NVENC (NVIDIA GPU)"

# 检查 NVENC 可用性
if ! gst-inspect-1.0 nvh264enc &> /dev/null; then
    ENCODER="x264enc"
    ENCODER_NAME="x264 (CPU 编码)"
    echo -e "${YELLOW}[WARN]${NC} NVENC 不可用，使用 CPU 编码"
else
    echo -e "${GREEN}[INFO]${NC} 使用 NVENC 硬件编码 ✓"
fi

# -----------------------------------------------------------------------------
# 显示配置
# -----------------------------------------------------------------------------
echo -e "${GREEN}[INFO]${NC} ========== 推流配置 =========="
echo "  编码器:     $ENCODER_NAME"
echo "  格式:       I420 (YUV 4:2:0 - 12-bit 等效)"
echo "  分辨率:     ${WIDTH}x${HEIGHT} @ ${FPS}fps"
echo "  码率:       ${BITRATE}kbps"
echo "  目标地址:   $TARGET_IP"
echo ""
echo -e "${BLUE}[CAM]${NC} 左相机: $LEFT_DEVICE -> ${TARGET_IP}:${LEFT_PORT}"
echo -e "${BLUE}[CAM]${NC} 右相机: $RIGHT_DEVICE -> ${TARGET_IP}:${RIGHT_PORT}"
echo ""

# -----------------------------------------------------------------------------
# 构建编码命令
# -----------------------------------------------------------------------------
if [ "$ENCODER" = "nvh264enc" ]; then
    # NVENC 编码参数 - 使用专用低延迟预设
    # preset=5: low-latency-hp (低延迟高性能) - 专门为低延迟设计
    # rc-mode=cbr: 恒定码率
    # zerolatency=true: 零延迟模式
    ENC_PARAMS="nvh264enc bitrate=$BITRATE preset=5 rc-mode=cbr zerolatency=true"
else
    # x264 编码参数 - 最低延迟配置
    ENC_PARAMS="x264enc bitrate=$BITRATE speed-preset=superfast tune=zerolatency threads=4"
fi

# -----------------------------------------------------------------------------
# 启动推流
# -----------------------------------------------------------------------------
echo -e "${GREEN}[INFO]${NC} ========== 启动双目低延迟推流 =========="
echo -e "${GREEN}[INFO]${NC} 按 Ctrl+C 同时停止两路推流"
echo -e "${GREEN}[INFO]${NC} 等待视频流启动..."
echo ""

# 陷阱处理 - Ctrl+C 时停止所有进程
trap cleanup INT TERM

cleanup() {
    echo ""
    echo -e "${YELLOW}[INFO]${NC} 正在停止推流..."
    kill $LEFT_PID $RIGHT_PID 2>/dev/null
    wait $LEFT_PID $RIGHT_PID 2>/dev/null
    echo -e "${GREEN}[INFO]${NC} 推流已停止"
    exit 0
}

# 启动左相机推流
# YUYV (4:2:2 16-bit) -> I420 (4:2:0 12-bit等效) -> NVENC编码
# 降低色彩采样率可减少带宽和编码延迟
echo -e "${BLUE}[CAM]${NC} 启动左相机推流..."
gst-launch-1.0 -v \
    v4l2src device=$LEFT_DEVICE do-timestamp=true ! \
    video/x-raw,format=YUY2,width=$WIDTH,height=$HEIGHT,framerate=$FPS/1 ! \
    videoconvert ! video/x-raw,format=I420 ! \
    $ENC_PARAMS ! \
    rtph264pay config-interval=1 ! \
    udpsink host=$TARGET_IP port=$LEFT_PORT sync=false \
    > $LEFT_LOG 2>&1 &

LEFT_PID=$!

# 启动右相机推流
echo -e "${BLUE}[CAM]${NC} 启动右相机推流..."
gst-launch-1.0 -v \
    v4l2src device=$RIGHT_DEVICE do-timestamp=true ! \
    video/x-raw,format=YUY2,width=$WIDTH,height=$HEIGHT,framerate=$FPS/1 ! \
    videoconvert ! video/x-raw,format=I420 ! \
    $ENC_PARAMS ! \
    rtph264pay config-interval=1 ! \
    udpsink host=$TARGET_IP port=$RIGHT_PORT sync=false \
    > $RIGHT_LOG 2>&1 &

RIGHT_PID=$!

# 等待进程启动
sleep 3

# 检查进程状态
if ! kill -0 $LEFT_PID 2>/dev/null; then
    echo -e "${RED}[ERROR]${NC} 左相机推流启动失败，查看日志: $LEFT_LOG"
    kill $RIGHT_PID 2>/dev/null
    exit 1
fi

if ! kill -0 $RIGHT_PID 2>/dev/null; then
    echo -e "${RED}[ERROR]${NC} 右相机推流启动失败，查看日志: $RIGHT_LOG"
    kill $LEFT_PID 2>/dev/null
    exit 1
fi

echo ""
echo -e "${GREEN}[INFO]${NC} ========== 双目低延迟推流运行中 =========="
echo -e "${BLUE}[CAM]${NC} 左相机 (PID: $LEFT_PID) -> ${TARGET_IP}:${LEFT_PORT}"
echo -e "${BLUE}[CAM]${NC} 右相机 (PID: $RIGHT_PID) -> ${TARGET_IP}:${RIGHT_PORT}"
echo ""
echo -e "${GREEN}[INFO]${NC} 日志文件:"
echo -e "${GREEN}[INFO]${NC}   左相机: $LEFT_LOG"
echo -e "${GREEN}[INFO]${NC}   右相机: $RIGHT_LOG"
echo ""
echo -e "${GREEN}[INFO]${NC} 按 Ctrl+C 停止推流"

# 等待进程
wait $LEFT_PID $RIGHT_PID
