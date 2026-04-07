# Remote Control — Vision Pro 远程图传遥操作系统

跨公网双向 WebRTC 立体视频传输 + 头部姿态回传系统。

```
┌─────────────────┐    ┌────────────────┐    ┌──────────────────┐
│  Linux 工控机   │    │  阿里云服务器  │    │  Apple Vision Pro│
│  (设备 A 端)    │◄──►│  LiveKit Server│◄──►│  (控制端)         │
│                 │    │                │    │                  │
│  ZED Mini 双目  │    │  39.102.113.104│    │  visionOS 26     │
│  GStreamer 采集 │    │  端口: 7880    │    │  LiveKit Swift   │
│  LiveKit Python │    │  TURN: 3478    │    │  SDK 2.12.1      │
│  NVENC H264     │    │                │    │                  │
└─────────────────┘    └────────────────┘    └──────────────────┘
       推流                  SFU 中继              接收 + 渲染
```

## 当前状态（经过验证）

| 测试项 | 结果 |
|--------|------|
| Linux → LiveKit Server 推流 | ✅ 1344x376 @60fps，NVENC H264 |
| 云端 SFU 中继 | ✅ UDP 直连，srflx 候选 |
| LiveKit → Python receiver 解码 | ✅ 100% 帧解码成功 |
| Vision Pro 接收（窗口模式） | ✅ 双眼可见视频 |
| Vision Pro 接收（沉浸模式） | ⚠️ 视频帧到达但 CompositorServices 渲染黑屏（待修复） |
| 网络延迟 | ✅ Linux → 云端 ~4ms，端到端 < 100ms |

## 目录结构

- `device-a-linux/` — Linux 工控机推流端代码（被控制端，拿摄像头的那台）
- `cloud/` — 阿里云 LiveKit Server 配置和部署说明
- `controller/` — Apple Vision Pro 接收端代码（控制端，戴头显的那个）

## 核心方案演进

之前尝试过两种自建 WebRTC 方案，**都失败了**：

1. **GStreamer webrtcbin 自建信令**：
   - GStreamer 1.20.3 的 DTLS（OpenSSL）和 LiveKit WebRTC（VP 端，BoringSSL）SRTP 协商不兼容
   - 表现：ICE connected，但 `Failed to unprotect RTP packet` 失败
   - 浏览器接收能工作，但 VP native app 不行

2. **aiortc 自建信令**：
   - aiortc 1.14.0 的双 ICE component 模式不稳定
   - 本地回环测试都失败（Connection(0) 和 Connection(1) 时序问题）
   - SRTP_AEAD_AES_256_GCM 默认协商，VP 端不支持

**最终方案：LiveKit Server 作为媒体中继 SFU**
- Linux 端用 LiveKit Python SDK 1.1.5（封装好的 WebRTC，自动用 NVENC 硬编码）
- VP 端用 LiveKit Swift SDK 2.12.1（官方支持 visionOS 1.0+，含 26.x）
- 两端都跟 LiveKit Server 对话，不直接对话 → 兼容性问题彻底消失
- 加密协商由 LiveKit 自己处理，DTLS/SRTP/ICE 全部由 SDK 封装

## 快速启动

### 1. 启动 LiveKit Server（云端，已部署）
```bash
ssh root@39.102.113.104
livekit-server --config /etc/livekit.yaml --node-ip 39.102.113.104
```
详见 [`cloud/README.md`](cloud/README.md)

### 2. 启动 Linux 推流端
```bash
# 必须用系统 Python（conda 没 gi 模块）
/usr/bin/python3 device-a-linux/zedmini_livekit_sender.py
```
详见 [`device-a-linux/README.md`](device-a-linux/README.md)

### 3. Vision Pro 接收端
在 Mac 上用 Xcode 26.4 打开 `controller/visionos-native/RemoteControl.xcodeproj`，target 选 Vision Pro 真机 → Run。
详见 [`controller/README.md`](controller/README.md)

## 网络要求

阿里云安全组入方向规则：
- TCP 7880 (LiveKit HTTP/WebSocket，必须)
- TCP 7881 (LiveKit RTC TCP fallback)
- UDP 50000-60000 (LiveKit RTC UDP，**视频数据走这里**)

如果 Linux 端运行 Clash Verge 等代理软件，需要在 TUN 配置中排除 LiveKit 服务器 IP 和局域网，否则 WebRTC UDP 流量会被劫持：

```yaml
# ~/.config/clash-verge-rev/Merge.yaml
tun:
  enable: true
  stack: mixed
  auto-route: true
  inet4-route-exclude-address:
    - 39.102.113.104/32
    - 192.168.0.0/16
    - 10.0.0.0/8
    - 172.16.0.0/12
```

## 关键参数

| 参数 | 值 |
|------|---|
| LiveKit 服务器 | `ws://39.102.113.104:7880` |
| API Key | `teleop_key` |
| API Secret | `teleop_secret_key_2026` |
| 房间名 | `teleop-room` |
| Sender 身份 | `zed-mini-sender` |
| Receiver 身份 | `vision-pro-receiver` |
| 视频分辨率 | 1344×376 SBS（每眼 672×376）|
| 帧率 | 60 fps |
| 编码 | H264 @ NVENC |
| 端到端延迟 | ~50-100ms |
