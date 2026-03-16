# RealSense 双目推流完整配置文档

## 概述

- **用途**: RealSense D435 双目相机 RTP 推流
- **编码**: H.264 (NVENC 硬件加速)
- **传输**: RTP over UDP
- **目标延迟**: ~100ms

---

## 1. 视频源配置

### 左相机
```
设备: /dev/video4
格式: YUYV (YUV 4:2:2, 16-bit)
分辨率: 1280x720
帧率: 30 fps
像素格式: V4L2_PIX_FMT_YUYV
```

### 右相机
```
设备: /dev/video16
格式: YUYV (YUV 4:2:2, 16-bit)
分辨率: 1280x720
帧率: 30 fps
像素格式: V4L2_PIX_FMT_YUYV
```

---

## 2. 编码器配置 (NVENC H.264)

```
编码器: nvh264enc
Preset: 5 (low-latency-hp)
RC Mode: cbr (固定码率)
Zero Latency: true
Bitrate: 4000 kbps (4 Mbps)
Profile: High
Level: 4.0
```

### NVENC Preset 说明
| Preset | 名称 | 延迟 | 用途 |
|--------|------|------|------|
| 0 | default | 2-3帧 | 质量 |
| 1 | fast | 1-2帧 | 平衡 |
| 5 | **ll-hp** | **1帧** | **低延迟高性能** |
| 6 | ll | 1帧 | 低延迟 |
| 7 | llhq | 1-2帧 | 低延迟高质量 |

---

## 3. RTP 配置

```
Payload Type: 96 (动态)
Clock Rate: 90000 Hz
Encoding: H.264
Packetization Mode: 1 (单一 NALU 单元)
Config Interval: 1 (每秒发送 SPS/PPS)
```

### RTP Header 扩展
- 无扩展头 (标准 RTP)

---

## 4. 网络配置

### 发送端 (Linux)
```
IP: 192.168.8.176
网卡: enp130s0 (Realtek 2.5GbE)
MTU: 1500
带宽: 1 Gbps (实际连接)
```

### 接收端
```
IP: 100.81.46.44
左相机端口: 5004/udp
右相机端口: 5006/udp
```

### 带宽需求
```
单路视频: ~4 Mbps
双路视频: ~8 Mbps
RTP 开销: ~10%
总计: ~9 Mbps
```

---

## 5. GStreamer 管道

### 发送端 (左相机)
```bash
gst-launch-1.0 -v \
    v4l2src device=/dev/video4 do-timestamp=true \
    ! video/x-raw,format=YUY2,width=1280,height=720,framerate=30/1 \
    ! videoconvert \
    ! video/x-raw,format=I420 \
    ! nvh264enc bitrate=4000 preset=5 rc-mode=cbr zerolatency=true \
    ! rtph264pay config-interval=1 \
    ! udpsink host=100.81.46.44 port=5004 sync=false
```

### 接收端 (超低延迟)
```bash
gst-launch-1.0 -v \
    udpsrc port=5004 \
    ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 \
    ! rtpjitterbuffer latency=0 \
    ! rtph264depay \
    ! h264parse \
    ! avdec_h264 \
    ! videoconvert \
    ! autovideosink sync=false
```

---

## 6. 延迟分解

| 阶段 | 延迟 | 优化 |
|------|------|------|
| 相机采集 | 5ms | V4L2 do-timestamp |
| 格式转换 | 3ms | YUYV→I420 |
| NVENC 编码 | 33ms | preset=5 |
| RTP 打包 | 1ms | config-interval=1 |
| 网络传输 | 30ms | 本地 LAN |
| **抖动缓冲** | **0ms** | **latency=0** |
| 解码 | 20ms | avdec_h264 |
| 显示 | 5ms | sync=false |
| **总计** | **~100ms** | |

---

## 7. SDP 示例

### 左相机 SDP
```
v=0
o=- 0 0 IN IP4 192.168.8.176
s=Left Camera RealSense
c=IN IP4 100.81.46.44
t=0 0
m=video 5004 RTP/AVP 96
a=rtpmap:96 H264/90000
a=fmtp:96 packetization-mode=1;profile-level-id=64001f
```

### 右相机 SDP
```
v=0
o=- 0 0 IN IP4 192.168.8.176
s=Right Camera RealSense
c=IN IP4 100.81.46.44
t=0 0
m=video 5006 RTP/AVP 96
a=rtpmap:96 H264/90000
a=fmtp:96 packetization-mode=1;profile-level-id=64001f
```

---

## 8. WebRTC 兼容性

### 编解码器
- H.264 Constrained Baseline Profile (Level 4.0)
- RFC 6184 RTP Payload Format for H.264 Video

### 候选配对 (ICE Candidates)
```
发送端候选:
  - host: 192.168.8.176:5004 (左)
  - host: 192.168.8.176:5006 (右)
  - srflx (STUN反射地址，需配置STUN服务器)
  - relay (TURN中继，如需NAT穿透)

接收端候选:
  - host: 100.81.46.44
```

### STUN/TURN 服务器建议
- STUN: stun.l.google.com:19302 (Google 公共 STUN)
- TURN: 需自行部署 (推荐 coturn)

---

## 9. 脚本位置

### Linux 发送端
```bash
~/teleop/scripts/20250314-cc-start_stereo_stream.sh
```

### Mac 接收端
```bash
~/teleop/scripts/20250314-cc-receive_stereo_ultralow_mac.sh
```

---

## 10. 故障排查

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| 高延迟 | 抖动缓冲累积 | 添加 rtpjitterbuffer latency=0 |
| 花屏 | 网络丢包 | 检查网络质量，增加码率 |
| 卡顿 | 解码性能不足 | 使用硬件解码器 |
| 无画面 | 防火墙阻挡 | 开放 UDP 5004/5006 |

---

## 11. 关键优化参数总结

```bash
# 发送端
nvh264enc preset=5 zerolatency=true  # 低延迟编码
udpsink sync=false                    # 立即发送，不等待时钟

# 接收端
rtpjitterbuffer latency=0             # 禁用抖动缓冲 (最重要!)
autovideosink sync=false              # 立即显示，不同步时钟
```

---

生成时间: 2025-03-14
作者: Claude Code
