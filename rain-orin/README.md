# Device A — Linux 工控机端

> 这是 Remote Control 系统的 Linux 端（"设备 A"），跑在装着 ZED Mini 摄像头和达妙电机的工控机上。
> 它做两件事：
> 1. **推视频流**：ZED Mini 双目 → GStreamer → LiveKit Server（云端中继）→ Vision Pro
> 2. **收头部位姿 + 控电机**：Vision Pro → LiveKit Data Channel → sender → UDP → 达妙电机控制脚本 → 电机

---

## 目录

- [硬件清单](#硬件清单)
- [系统要求](#系统要求)
- [一键部署](#一键部署)
- [手动部署（详细步骤）](#手动部署详细步骤)
- [数据流图](#数据流图)
- [启动顺序](#启动顺序)
- [电机校准流程（首次安装必做）](#电机校准流程首次安装必做)
- [VP→电机 控制档位](#vp电机-控制档位)
- [文件清单](#文件清单)
- [故障排查](#故障排查)
- [性能数据 & 调参参考](#性能数据--调参参考)

---

## 硬件清单

| 项 | 数量 | 备注 |
|----|------|------|
| **ZED Mini** 双目摄像头 | 1 | USB 3.0，提供 1344×376 SBS @ 100Hz |
| **达妙 DM-J4310-2EC** 电机 | 2 | CAN 控制，已配 ID 0x01 + 0x02 |
| **HDSC USB→CAN** 适配器 | 1 | 注意：**HDSC** 牌子，CDC 虚拟串口模式（不是 SocketCAN） |
| **24V 直流电源** | 1 套 | 通过 XT30 给两个电机串联供电 |
| CAN 线 + XT30 线 | 若干 | 电机间串联用 |
| NVIDIA GPU 工控机 | 1 | 用 NVENC 硬编 H264，跑得动 1344×376@100 |

⚠️ **常见误解**：USB→CAN 适配器有两种模式
- ✅ **HDSC CDC 虚拟串口**（pyserial 打开 `/dev/ttyACM*`）← 达妙官方推荐，本系统用这个
- ❌ **SocketCAN/gs_usb**（python-can 走 `can0` 接口）← 比如 DISCOVER Robotics 那种，**不兼容达妙官方驱动**

如果你的 USB-CAN 是 SocketCAN 类型的，需要重新写一套基于 python-can 的达妙驱动（DM_CAN.py 用的是 pyserial），不在本仓库范围内。

---

## 系统要求

| 项 | 推荐 | 必需 |
|----|------|------|
| OS | Ubuntu 22.04 LTS | Linux + Python 3.10 |
| Python | `/usr/bin/python3 == 3.10` | **必须是系统 Python，不能用 conda** |
| GPU | NVIDIA + driver 535+ | 可选（无 GPU 用软件编码会卡） |
| GStreamer | 1.20+ | python3-gi + python3-gst-1.0 |
| 网络 | 上行 ≥ 15 Mbps | 推视频到云端用 |

⚠️ **conda 污染**：如果你的 `which python3` 指向 conda，**所有脚本都要显式用 `/usr/bin/python3`**。原因：
- sender 用 GStreamer 的 Python binding (`gi.repository.Gst`)
- `python3-gi` 和 `python3-gst-1.0` 是 apt 装的 deb 包，**只对系统 Python 生效**
- conda 的 python 里 import gi 会失败

解决：要么 `alias python3=/usr/bin/python3`，要么所有命令显式写 `/usr/bin/python3`。

---

## 一键部署

```bash
git clone https://github.com/thedaosheng/Remote_Control.git
cd Remote_Control/device-a-linux
chmod +x setup_linux.sh
./setup_linux.sh
```

`setup_linux.sh` 会做：
1. 检查 Ubuntu 版本和必备 deb 包（提示你装缺的）
2. 检查系统 Python 是否被 conda 污染
3. `pip install --user -r requirements.txt`（livekit / pyserial / numpy）
4. 检测 USB 设备（ZED Mini / 达妙 USB-CAN / dialout 组）
5. 验证 NVENC 可用
6. （可选）安装 systemd user service

之后跑校准脚本 + 启动 sender，详见下面的[启动顺序](#启动顺序)和[电机校准流程](#电机校准流程首次安装必做)。

---

## 手动部署（详细步骤）

如果一键脚本失败，按下面手动来。

### 1. 装系统 deb 包

```bash
sudo apt update
sudo apt install -y \
    python3 python3-pip \
    python3-gi python3-gst-1.0 \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    v4l-utils can-utils
```

如果有 NVIDIA GPU，还要装：
```bash
sudo apt install -y libnvidia-encode-535  # 或对应版本
```

### 2. 装 Python 依赖（用系统 Python！）

```bash
/usr/bin/python3 -m pip install --user --upgrade pip
/usr/bin/python3 -m pip install --user -r requirements.txt
```

验证：
```bash
/usr/bin/python3 -c "import livekit; import serial; import numpy; print('OK')"
/usr/bin/python3 -c "import gi; gi.require_version('Gst','1.0'); from gi.repository import Gst; Gst.init(None); print(Gst.version())"
```

### 3. 把用户加到 dialout 组（打开 /dev/ttyACM* 需要）

```bash
sudo usermod -aG dialout $USER
# 重新登录或者 newgrp dialout
```

### 4. 检测 USB 设备

```bash
# ZED Mini
ls /dev/video*
v4l2-ctl --device=/dev/video0 --list-formats-ext

# 达妙 USB-CAN（找 HDSC）
ls /dev/serial/by-id/
# 应该能看到类似 usb-HDSC_CDC_Device_*
```

记下 HDSC 那个 by-id 链接对应的 `/dev/ttyACMx` 路径。本系统所有脚本默认是 `/dev/ttyACM0`，如果你的不是，启动时加 `--serial /dev/ttyACMx`。

### 5. （可选）systemd user service

```bash
mkdir -p ~/.config/systemd/user
sed "s|__SCRIPT_DIR__|$(pwd)|g" systemd/lk-sender.service > ~/.config/systemd/user/lk-sender.service
systemctl --user daemon-reload
systemctl --user start lk-sender
systemctl --user enable lk-sender   # 开机自启
loginctl enable-linger $USER         # 不登录也跑
```

---

## 数据流图

```
ZED Mini (1344x376@100 YUY2)
   │ v4l2src
   ▼
GStreamer pipeline (videoconvert RGBA + videoflip 180°)
   │ appsink → 推帧线程
   ▼
zedmini_livekit_sender.py
   │  rtc.VideoSource.capture_frame(RGBA) @ 100Hz
   │  publish_options.video_encoding.max_framerate = 100   ★ 必须设, 默认 30
   │  publish_options.video_encoding.max_bitrate = 15M
   ▼
LiveKit Server (阿里云 39.102.113.104:7880)
   ↓                                        ↑
   ↓ 视频流 H264 RTP                     pose Data Channel
   ↓                                        ↑
Apple Vision Pro (visionOS 26 RemoteControl App)
   • CompositorServices Metal 立体渲染
   • ARKit WorldTrackingProvider 头部追踪 30Hz → LiveKit Data Channel publish
   ↑
   │ pose JSON {pitch, yaw, p, q, t}
   ▼
zedmini_livekit_sender.py 内部 _on_data_received 回调
   • quat → euler (绕X 绕Y 绕Z)
   • 物理映射 swap: motor_pitch ← roll(绕X), motor_yaw ← pitch(绕Y)
   • UDP 转发 → 127.0.0.1:9000  {"pitch":deg, "yaw":deg, "t":...}
   ▼
dm_motor_vp_control.py
   • UDP 监听线程: 收 pose, EMA 平滑 vel
   • 200Hz 主控制循环: target_pos + dq_target → MIT 控制
   • 速度前馈 (dq_target = 平滑后的头部速度)
   • 逐帧位置限速 + 硬限幅
   ▼
DM_Control_Python (DM_CAN.py)
   • pyserial @ 921600 → HDSC USB-CAN 适配器
   • DM 电机协议 (0x01 / 0x02 ID)
   ▼
达妙 DM-J4310-2EC × 2
   • CAN_ID 0x01 = Yaw   (左右转头)
   • CAN_ID 0x02 = Pitch (俯仰点头)
```

---

## 启动顺序

第一次（硬件首次安装/校准过零点之后）和日常都按这个顺序：

```bash
# 1. 启动 sender (推视频 + 接收 pose)
#    方式 A: systemd 后台跑
systemctl --user start lk-sender
journalctl --user -u lk-sender -f   # 看日志
#
#    方式 B: 前台跑 (调试时方便)
/usr/bin/python3 device-a-linux/zedmini_livekit_sender.py

# 2. 在 VP 上启动 RemoteControl App, Connect, 进入 Immersive Space
#    (零点 = VP 进入 Immersive 那一瞬间的头部姿态)

# 3. 启动 VP→电机控制脚本 (等 VP 已经在 Immersive 之后再启动比较稳)
/usr/bin/python3 device-a-linux/dm_motor/scripts/dm_motor_vp_control.py --preset L4
```

退出顺序：反过来。先 Ctrl+C 电机控制脚本（会安全归零失能），再退出 VP，最后停 sender。

---

## 电机校准流程（首次安装必做）

电机硬件首次安装、或者拆装过之后，按下面顺序做一次。日常使用不用重做（零点写 Flash 永久保存）。

### 第 1 步 — 永久零点

把电机用手摆到你想要的"零位"姿态，跑：
```bash
/usr/bin/python3 device-a-linux/dm_motor/scripts/dm_motor_zero_calibration.py
```

脚本会：enable → set_zero_position(0xFE) → save_motor_param(0xAA) → disable → enable → 验证（应该读到 ±0.01°）。

### 第 2 步 — 测物理可达范围（互动）

零点之后，电机的运动范围是有限的（机械限位 + 防过冲），但你不知道两侧极限角度是多少。跑：
```bash
/usr/bin/python3 device-a-linux/dm_motor/scripts/dm_motor_axis_verify.py --skip-zero
```

脚本会：
1. 失能电机 → 你用手把电机掰到一侧极限 → 按回车记录
2. 再掰到另一侧 → 按回车记录
3. 然后单独让 Yaw 电机 ±15° 摆动 → 你看是物理上"左右转头"还是别的
4. 再单独让 Pitch 电机 ±15° → 你看是"点头"还是别的

测出来的极值会写到 `device-a-linux/dm_motor/scripts/dm_motor_calibration.json`。

⚠️ **dm_motor_calibration.json 是机器特定的**，不会进 git（在 .gitignore 里）。每台机器都要重新跑校准。

### 第 3 步 — 半幅摆动（可选，验证极性）

```bash
/usr/bin/python3 device-a-linux/dm_motor/scripts/dm_motor_half_amp_sweep.py
```

让电机在工作区中心 ± 半幅范围内做正弦摆动 15 秒，先 Yaw 再 Pitch。看物理动作和方向是否符合预期。如果方向反了，下一步 vp_control 启动时加 `--sign-yaw -1` 或 `--sign-pitch -1`。

---

## VP→电机 控制档位

跟手感分 5 档（详见 `dm_motor/PRESETS.md`）：

```bash
# 一键选档
/usr/bin/python3 dm_motor/scripts/dm_motor_vp_control.py --preset L4

# 也可以在 preset 之上单独覆盖
/usr/bin/python3 dm_motor/scripts/dm_motor_vp_control.py --preset L4 --kp 28
```

| 档位 | KP | KD | rate | ff_gain | ff_ema | 适用 |
|------|----|----|------|---------|--------|------|
| **L1 安全** | 8 | 1.2 | 100 | off | - | 第一次跑 / 演示 |
| **L2 温和** | 15 | 1.2 | 200 | 0.5 | 0.3 | 日常调试 |
| **L3 默认** ⭐ | 20 | 1.0 | 300 | 1.0 | 0.5 | 推荐起步 |
| **L4 跟手** | 25 | 1.0 | 500 | 1.2 | 0.7 | 想要明显跟手 |
| **L5 激进** | 30 | 0.8 | 600 | 1.5 | 0.8 | 极限测试（可能震动） |

各旋钮含义：
- **`--kp`** = MIT 模式位置刚度，越大跟踪误差越小，太大震
- **`--kd`** = 速度阻尼，抑制振荡，太大粘滞
- **`--rate-limit`** = 软件层逐帧位置变化上限（°/s）
- **`--ff-gain`** = 速度前馈增益（dq_target 倍数），1.0 = 1:1 跟随
- **`--ff-ema`** = 速度前馈的 EMA 平滑系数，越大越跟手越容易抖

故障切回 L1：
```bash
/usr/bin/python3 dm_motor/scripts/dm_motor_vp_control.py --preset L1
```

---

## 文件清单

```
device-a-linux/
├── README.md                          ← 本文件
├── setup_linux.sh                     ← 一键部署脚本
├── requirements.txt                   ← Python 依赖
├── zedmini_livekit_sender.py          ← sender 主程序 (推视频 + 接 VP pose)
├── generate_livekit_tokens.py         ← LiveKit JWT token 生成 (基本不用动)
├── systemd/
│   └── lk-sender.service              ← systemd user unit
├── dm_motor/                          ← 达妙电机相关
│   ├── PRESETS.md                     ← 档位说明 + 调参手册
│   ├── DM_Control_Python/             ← 达妙官方 Python 驱动 (vendor 进来的)
│   │   ├── DM_CAN.py
│   │   ├── DM_Motor_Test.py
│   │   ├── README.md                  ← 官方 README
│   │   ├── README_zh.md
│   │   ├── LICENSE                    ← Apache 2.0
│   │   └── requirements.txt
│   ├── scripts/
│   │   ├── dm_motor_zero_calibration.py    ← 永久零点 (首次安装跑一次)
│   │   ├── dm_motor_axis_verify.py         ← 互动测物理范围 + 验证轴对应
│   │   ├── dm_motor_half_amp_sweep.py      ← 半幅摆动验证 (可选)
│   │   └── dm_motor_vp_control.py          ← VP → 电机 实时控制 (含 5 档 preset)
│   └── calibration.example.json       ← 校准数据示例 (.gitignore 真实校准)
└── ros2_ws/                            ← 双臂机器人 ROS2 (本期不动, 之前的)
```

---

## 故障排查

### 问题 1: `import gi` 报错 / sender 起不来

```
ModuleNotFoundError: No module named 'gi'
```
原因：用了 conda 的 python，conda 里没装 GStreamer python binding。
修复：所有命令显式用 `/usr/bin/python3`，或者
```bash
alias python3=/usr/bin/python3
```

### 问题 2: VP 端只收到 30 fps（视频很卡）

原因：sender 没设 `publish_options.video_encoding.max_framerate`，LiveKit Python SDK 默认 30。
修复：本仓库的 sender 已经设了 100，如果你的版本没设，加这两行：
```python
publish_options.video_encoding.max_framerate = 100
publish_options.video_encoding.max_bitrate   = 15_000_000
```
注意 `VideoEncoding` 是 protobuf message，不能直接 `=` 赋值，必须**逐字段写**。

### 问题 3: 达妙串口打不开 / `read_motor_param` 返回 None

可能原因和检查顺序：
1. **串口路径错** → `ls /dev/serial/by-id/` 确认 HDSC 链接到的 ttyACM 编号，启动时加 `--serial /dev/ttyACMx`
2. **dialout 组没加** → `id` 看看，没有就 `sudo usermod -aG dialout $USER` 重新登录
3. **CAN_H/CAN_L 接反** → 物理检查接线
4. **24V 没接** → 检查电源
5. **CAN 波特率** → 达妙固件默认 1Mbps，HDSC 模块也是 1Mbps，不用动
6. **电机 ID 不是 0x01/0x02** → 用达妙官方 Windows 上位机重新配 CAN ID

### 问题 4: 我的 USB-CAN 不是 HDSC

如果你接的是 DISCOVER Robotics、CANable、之类的 SocketCAN 适配器，本仓库的脚本**不兼容**。因为 `DM_Control_Python/DM_CAN.py` 用的是 pyserial（HDSC 的 CDC 虚拟串口模式），SocketCAN 设备 pyserial 打不开（会报 `Inappropriate ioctl for device`）。

要支持 SocketCAN 需要重写一套基于 python-can 的达妙驱动，不在本仓库范围。最简单的解决：换一个 HDSC USB-CAN 适配器（达妙官方推荐的那种）。

### 问题 5: 电机轴对应反了（你抬头电机却左转）

跑 `dm_motor_axis_verify.py` 重新确认 0x01 / 0x02 哪个是 Yaw 哪个是 Pitch。如果代码里假设错了，改 `dm_motor_vp_control.py` 里 motor 创建的两行：
```python
m_yaw   = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)  # ← swap 这两行
m_pitch = Motor(DM_Motor_Type.DM4310, 0x02, 0x12)
```

### 问题 6: 电机方向反了（抬头变低头）

启动 vp_control 时加极性翻转：
```bash
/usr/bin/python3 dm_motor/scripts/dm_motor_vp_control.py --preset L4 --sign-pitch -1
# 或 --sign-yaw -1
```

### 问题 7: 电机震动 / 啸叫

按这个顺序排查：
1. 切回 L1 安全档：`--preset L1`
2. 如果 L1 也震 → 物理结构有问题（线束/松动/共振）
3. 如果只在 L4/L5 震 → KD 加 0.5，或者 ff-gain 减半
4. 如果只在快速转头时震 → ff-ema 减 0.2

### 问题 8: 控制脚本启动后电机突然跳

不应该出现，脚本有 1.5s 软淡入 + cosine ramp。如果出现：
1. 检查 fade 是否正常打印（`[RECV f0.30]` 这种）
2. 看 rate-limit 是不是被设得太高
3. 看 VP 端 LiveKitContentView 进入 Immersive 之前的姿态是否正常

### 问题 9: VP 进 Immersive Space 后零点偏

零点是进 Immersive 的那一帧记录的。如果你进的时候头不在自然朝前姿态，整个相对偏移就会偏。
解决：退出 Immersive → Disconnect → Connect → 自然朝前 → 再进 Immersive。

### 问题 10: sender 跑到一半视频卡住

可能 LiveKit 重连了（网络抖动）。看日志：
```bash
journalctl --user -u lk-sender -f
```
sender 内部有自动重连，正常情况几秒后恢复。如果一直不恢复，检查云端 LiveKit Server 是否还在跑。

---

## 性能数据 & 调参参考

### sender 推帧
- 1344×376@100 fps YUY2 → RGBA → NVENC H264
- 实测稳态 99.8 fps（CPU < 5%, GPU < 10%）
- 上行带宽 ~10 Mbps

### VP 端接收
- 视频：~90 fps（用户实测）
- pose：30 Hz（VP 端 `LiveKitConfig.poseFrequency` 写死的，可改）

### 端到端延迟（motion-to-photon 估算）
| 环节 | 延迟 |
|------|------|
| VP 头部追踪 | ~5 ms |
| VP poseTimer 30Hz 采样 | 平均 17 ms / 最坏 33 ms |
| LiveKit 网络传输 | 30-50 ms（云端中继） |
| sender UDP 转发 | <1 ms |
| 控制脚本 EMA 平滑 (α=0.5) | ~33 ms |
| 200 Hz 控制循环 | ~5 ms |
| 电机 CAN + 物理响应 | 5-10 ms |
| **总计** | **~95-150 ms** |

如果想再降：把 VP 端的 `LiveKitConfig.poseFrequency` 从 30 提到 60 或 90，能省 10-20 ms。这是 VP 端 Swift 代码 `controller/visionos-native/RemoteControl/RemoteControl/LiveKit/LiveKitManager.swift` 里的常量。

### 跟踪精度（实测）
| Preset | 跟踪误差 | 备注 |
|--------|---------|------|
| L1 | 5-7° | 跟得最差，但绝对不震 |
| L3 | 1-2° | 默认值 |
| L4 | <1° | 跟手感强 |
| L5 | <1° | 跟手感最强但有时震 |

---

## 联系方式

- 仓库：https://github.com/thedaosheng/Remote_Control
- 问题：在 GitHub Issues 提
- VP 端 Swift 代码：`controller/visionos-native/`
- 云端 LiveKit Server 配置：`cloud/`
