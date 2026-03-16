#!/bin/bash
# ============================================
#  RealSense 双目相机预览
#  ============================================
#  功能：显示左右两个相机的实时预览窗口
#  作者：Claude Code
#  日期：2025-03-16
# ============================================

echo "=========================================="
echo "   RealSense 双目相机预览"
echo "=========================================="
echo ""

# 检查相机设备是否存在
if [ ! -e "/dev/video4" ]; then
    echo "[错误] 左相机设备 /dev/video4 不存在！"
    read -p "按任意键退出..."
    exit 1
fi

if [ ! -e "/dev/video10" ]; then
    echo "[错误] 右相机设备 /dev/video10 不存在！"
    read -p "按任意键退出..."
    exit 1
fi

echo "[*] 检测到双目相机设备"
echo "    左相机: /dev/video4"
echo "    右相机: /dev/video10"
echo ""

# 检查是否有其他进程占用相机
LEFT_BUSY=$(lsof /dev/video4 2>/dev/null | grep -v COMMAND | wc -l)
RIGHT_BUSY=$(lsof /dev/video10 2>/dev/null | grep -v COMMAND | wc -l)

if [ "$LEFT_BUSY" -gt 0 ]; then
    echo "[警告] 左相机 /dev/video4 正在被其他进程占用！"
    echo "         可能的进程:"
    lsof /dev/video4 2>/dev/null | grep -v COMMAND
    echo ""
fi

if [ "$RIGHT_BUSY" -gt 0 ]; then
    echo "[警告] 右相机 /dev/video10 正在被其他进程占用！"
    echo "         可能的进程:"
    lsof /dev/video10 2>/dev/null | grep -v COMMAND
    echo ""
fi

echo "[*] 启动双目预览..."
echo "    - 左相机预览窗口将显示 RGB 图像"
echo "    - 右相机预览窗口将显示 RGB 图像"
echo ""
echo "[提示] 按 Ctrl+C 关闭预览"
echo ""
sleep 1

# 启动左相机预览（后台）
gst-launch-1.0 -v \
    v4l2src device=/dev/video4 do-timestamp=true \
    ! video/x-raw,format=YUY2,width=1280,height=720,framerate=30/1 \
    ! videoconvert \
    ! autovideosink sync=false \
    > /tmp/left_preview.log 2>&1 &

LEFT_PID=$!
echo "[✓] 左相机预览已启动 (PID: $LEFT_PID)"

# 等待一下再启动右相机
sleep 0.5

# 启动右相机预览（后台）
gst-launch-1.0 -v \
    v4l2src device=/dev/video10 do-timestamp=true \
    ! video/x-raw,format=YUY2,width=1280,height=720,framerate=30/1 \
    ! videoconvert \
    ! autovideosink sync=false \
    > /tmp/right_preview.log 2>&1 &

RIGHT_PID=$!
echo "[✓] 右相机预览已启动 (PID: $RIGHT_PID)"

echo ""
echo "[*] 预览运行中... 按 Ctrl+C 停止"
echo ""

# 等待用户中断
trap "echo ''; echo '[*] 正在关闭预览...'; kill $LEFT_PID $RIGHT_PID 2>/dev/null; echo '[✓] 预览已关闭'; exit 0" INT TERM

# 持续运行直到被中断
while true; do
    sleep 1

    # 检查进程是否还在运行
    if ! kill -0 $LEFT_PID 2>/dev/null; then
        echo "[!] 左相机预览进程已停止"
        kill $RIGHT_PID 2>/dev/null
        break
    fi

    if ! kill -0 $RIGHT_PID 2>/dev/null; then
        echo "[!] 右相机预览进程已停止"
        kill $LEFT_PID 2>/dev/null
        break
    fi
done

echo "[✓] 预览已关闭"
