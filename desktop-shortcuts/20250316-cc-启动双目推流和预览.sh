#!/bin/bash
# ============================================
#  RealSense 双目推流 + 预览
#  ============================================
#  功能：同时推流到云端并在本地显示预览
#  作者：Claude Code
#  日期：2025-03-16
# ============================================

echo "=========================================="
echo "   RealSense 双目推流 + 预览"
echo "=========================================="
echo ""

# 检查相机设备
if [ ! -e "/dev/video4" ] || [ ! -e "/dev/video10" ]; then
    echo "[错误] 相机设备不存在！"
    read -p "按任意键退出..."
    exit 1
fi

# 检查是否有其他进程占用
if lsof /dev/video4 >/dev/null 2>&1 || lsof /dev/video10 >/dev/null 2>&1; then
    echo "[警告] 检测到相机被占用，正在停止旧进程..."
    pkill -9 -f "webrtc.*client" 2>/dev/null
    pkill -9 -f "gst-launch.*video4" 2>/dev/null
    pkill -9 -f "gst-launch.*video10" 2>/dev/null
    sleep 1
fi

echo "[*] 启动双目推流和预览..."
echo ""

# 设置显示环境（确保显示在本地显示器上）
export DISPLAY=:0

# 启动左相机：推流 + 预览（使用 tee 分流）
gst-launch-1.0 \
    v4l2src device=/dev/video4 do-timestamp=true \
    ! video/x-raw,format=YUY2,width=1280,height=720,framerate=30/1 \
    ! videoconvert \
    ! tee name=t \
    t. ! queue ! autovideosink sync=false \
    t. ! queue \
    ! video/x-raw,format=I420 \
    ! nvh264enc preset=5 zerolatency=true bitrate=4000 rc-mode=cbr \
    ! rtph264pay config-interval=1 \
    ! application/x-rtp,media=video,encoding-name=H264,payload=96 \
    ! udpsink host=39.102.113.104 port=51004 sync=false \
    > /tmp/left_full.log 2>&1 &

LEFT_PID=$!
echo "[✓] 左相机: 推流(51004) + 预览 (PID: $LEFT_PID)"

sleep 0.5

# 启动右相机：推流 + 预览
gst-launch-1.0 \
    v4l2src device=/dev/video10 do-timestamp=true \
    ! video/x-raw,format=YUY2,width=1280,height=720,framerate=30/1 \
    ! videoconvert \
    ! tee name=t \
    t. ! queue ! autovideosink sync=false \
    t. ! queue \
    ! video/x-raw,format=I420 \
    ! nvh264enc preset=5 zerolatency=true bitrate=4000 rc-mode=cbr \
    ! rtph264pay config-interval=1 \
    ! application/x-rtp,media=video,encoding-name=H264,payload=96 \
    ! udpsink host=39.102.113.104 port=51006 sync=false \
    > /tmp/right_full.log 2>&1 &

RIGHT_PID=$!
echo "[✓] 右相机: 推流(51006) + 预览 (PID: $RIGHT_PID)"

echo ""
echo "[*] 运行中... 按 Ctrl+C 停止"
echo ""

# 等待用户中断
trap "echo ''; echo '[*] 正在停止...'; kill $LEFT_PID $RIGHT_PID 2>/dev/null; echo '[✓] 已停止'; exit 0" INT TERM

while true; do
    sleep 1
    if ! kill -0 $LEFT_PID 2>/dev/null; then
        echo "[!] 左相机进程已停止"
        break
    fi
    if ! kill -0 $RIGHT_PID 2>/dev/null; then
        echo "[!] 右相机进程已停止"
        break
    fi
done

echo "[✓] 所有进程已停止"
