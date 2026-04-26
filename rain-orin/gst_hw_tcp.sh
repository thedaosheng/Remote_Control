#!/bin/bash
# Orin GStreamer hw-encode → TCP server on :5004
# 等待 lk CLI 作为 client 连上,开始推 H264 Annex-B

set -e
PORT="${GST_TCP_PORT:-5004}"

echo "[gst] 启动硬编管线 (等待 lk CLI 连 127.0.0.1:${PORT})..."

# 注意: tcpserversink 会 block 直到 client 连上
exec gst-launch-1.0 -v \
  v4l2src device=/dev/video0 do-timestamp=true \
  ! video/x-raw,format=YUY2,width=1344,height=376,framerate=60/1 \
  ! nvvidconv flip-method=2 \
  ! 'video/x-raw(memory:NVMM),format=NV12' \
  ! nvv4l2h264enc \
      bitrate=2000000 \
      insert-sps-pps=1 \
      iframeinterval=1 idrinterval=30 slice-header-spacing=0 \
      preset-level=4 \
      maxperf-enable=1 \
      control-rate=1 \
      profile=2 \
  ! h264parse config-interval=1 \
  ! video/x-h264,stream-format=byte-stream,alignment=au \
  ! queue leaky=downstream max-size-buffers=10 max-size-time=0 max-size-bytes=0 \
  ! tcpserversink host=127.0.0.1 port=${PORT} sync=false
