# Device A — Linux 工控机（被控制端）

包含两大模块：
1. **视频推流** — ZED Mini 立体相机 → LiveKit → Apple Vision Pro
2. **ROS2 遥操作框架** — 双臂机器人完整控制链路（Airbot Play G2 × 2 + 舵轮底盘 + 云台）

---

## 目录结构

```
device-a-linux/
├── README.md                          ← 本文件
├── zedmini_livekit_sender.py          ← LiveKit 推流（生产版）
├── generate_livekit_tokens.py         ← 生成 LiveKit JWT token
├── ROS2_SETUP.md                      ← ROS2 环境搭建文档
└── ros2_ws/                           ← ROS2 工作空间
```

---

## 视频推流（LiveKit 方案）

### 工作原理

```
ZED Mini (/dev/video0)
  ↓ GStreamer v4l2src + videoconvert + videoflip
  ↓ 1344x376 RGBA @60fps
LiveKit VideoSource (Python SDK)
  ↓ 自动 H264 编码（NVENC 硬件加速）
LiveKit Room (cloud)
  ↓ SFU 中继到所有订阅者
Apple Vision Pro (LiveKit Swift SDK)
```

### 依赖

- ZED Mini 相机（USB，应该出现在 `/dev/video0`）
- NVIDIA GPU（用于 NVENC 硬编码，自动启用）
- 系统 Python 3.10+（`/usr/bin/python3`，**不能用 conda**，因为缺 `gi` 模块）
- GStreamer 1.20+ 及插件：
  ```bash
  sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base \
                   gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
                   gstreamer1.0-plugins-ugly python3-gi
  ```
- Python 包：
  ```bash
  /usr/bin/pip3 install livekit livekit-api numpy
  ```

### 启动

```bash
# 必须用系统 Python
/usr/bin/python3 -u zedmini_livekit_sender.py
```

### 验证运行正常的输出

```
[GStreamer] 摄像头已启动
[LiveKit] 房间 SID: RM_xxx
[LiveKit] VideoSource 已创建: 1344x376
[LiveKit] VideoTrack 已创建: zed-mini-stereo
[GStreamer] 帧 #301: 2021376 bytes (期望 2021376), 1344x376 RGBA
[LiveKit] 推帧 #301: 1344x376 RGBA, 平均 60.0 fps, 运行 5.0s
```

帧率应稳定在 60fps，启动时会自动检测到 `Nvidia Encoder is supported`。

### 配置项（脚本顶部）

```python
LIVEKIT_URL = "ws://39.102.113.104:7880"
API_KEY = "teleop_key"
API_SECRET = "teleop_secret_key_2026"
ROOM_NAME = "teleop-room"
IDENTITY = "zed-mini-sender"
ZED_DEVICE = "/dev/video0"
WIDTH = 1344
HEIGHT = 376
FPS = 60
```

### Pose 回传

VP 端通过 LiveKit Data Channel 发送头部姿态（30Hz JSON）：
```json
{"type":"pose","t":1775404983.88,"p":[x,y,z],"q":[qx,qy,qz,qw]}
```

Sender 接收后转欧拉角，UDP 转发给达妙引擎（127.0.0.1:9000）：
```json
{"pitch": +12.34, "yaw": -5.67, "t": 1775404983.88}
```

### Token 生成

`generate_livekit_tokens.py` 用来生成长期有效（30 天）的 JWT token，写入到 sender 脚本和 VP 端代码中：
```bash
/usr/bin/python3 generate_livekit_tokens.py
```

---

## ROS2 遥操作框架

详细见 [ROS2_SETUP.md](ROS2_SETUP.md)。

### 快速启动

```bash
cd ros2_ws
export PATH="/usr/bin:$PATH"  # 绕开 miniconda
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

ros2 launch airbot_bringup system.launch.py mode:=mock
```
