# Remote_Control

遥操的 AI 代码库

## RealSense 双目 WebRTC 远程图传

Linux 端 RealSense 双目相机通过 WebRTC 推流到远程接收端（MacBook）。

### 快速开始

```bash
# 启动 WebRTC 推流
/usr/bin/python3 /home/rhz/teleop/scripts/20250316-cc-webrtc_signaling_client.py
```

### 文件说明

- `scripts/20250316-cc-webrtc_signaling_client.py` - WebRTC 发送端（主程序）
- `scripts/20250316-cc-webrtc_stereo_client.py` - 双目 WebRTC 发送端
- `scripts/20250316-cc-webrtc_signaling_server.py` - 云端信令服务器

### 桌面快捷方式

见 `desktop-shortcuts/` 目录：
- `双目预览.desktop` - 预览双目相机
- `双目推流+预览.desktop` - 推流同时预览

### 配置

- 信令服务器: ws://39.102.113.104:8765
- TURN 服务器: turn://remote:Wawjxyz3%21@39.102.113.104:3478
- 左相机: /dev/video4
- 右相机: /dev/video10

**详细日报**: 见 `desktop-shortcuts/20250316-日报-WebRTC公网图传.md`
