# rain-orin/ — AGX Orin (Rain high-level node)

> Last updated: 2026-04-26. See top-level [`ARCHITECTURE.md`](../ARCHITECTURE.md) for system context.

## Role

The AGX Orin is the **Rain-side high-level compute node**: ROS 2 host for video pipeline, head-pose data channel, AIRBOT safety services, and cloud↔ROS bridges. It is the on-robot brain.

## Hardware

- Jetson AGX Orin Devkit, P3701-0005, L4T R36.4.3, JetPack 6.2.1
- Hostname `bit-orin`, sudo user `bit`
- Two NICs: `eno1` (1 GbE, primary, Rain LAN) + `wlP1p1s0` (WiFi, fallback)
- USB-CAN dongles for AIRBOT (slcan via `slcand`; NOT mttcan — see [agent-bus `feedback_jetson_kernel_modules`](../SECURITY.md) for kernel-module rebuild note)
- ZED Mini stereo camera

## Network

- Primary: `eno1 192.168.8.6/24` (Rain LAN island, 千兆交换机 + Huawei CPE 5S 5G uplink)
- Fallback: `wlP1p1s0 192.168.0.103/24` (家里 WiFi, 当 Rain 段不可用时)
- Tailscale: `100.75.76.30` (`thedaosheng@` tailnet)

## Bring-up from zero

```bash
# 1. Clone repo
git clone https://github.com/thedaosheng/Remote_Control.git ~/Telep
cd ~/Telep

# 2. Install ROS 2 Humble + Cyclone DDS
sudo apt update
sudo apt install -y \
    ros-humble-ros-base \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions

# 3. Set env in ~/.bashrc
cat >> ~/.bashrc <<'EOF'
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI=file:///home/bit/cyclonedds.xml
EOF
source ~/.bashrc

# 4. Copy Cyclone XML to home (pins eno1 multicast)
cp rain-orin/setup/cyclonedds-eno1.xml ~/cyclonedds.xml

# 5. LiveKit credentials — two paths:
#  (a) Interactive use: copy top-level .env.example → .env, fill values, source it
cp .env.example .env
# edit .env, fill in LIVEKIT_API_KEY + LIVEKIT_API_SECRET (from /etc/livekit.yaml on Aliyun)
set -a && source .env && set +a
#
#  (b) systemd services: write same key/value pairs into root-owned /etc/livekit.env
sudo tee /etc/livekit.env > /dev/null <<EOF
LIVEKIT_API_KEY=...
LIVEKIT_API_SECRET=...
EOF
sudo chmod 600 /etc/livekit.env
sudo chown root:root /etc/livekit.env

# 6. Install Python deps (system Python, NOT conda)
/usr/bin/python3 -m pip install --user --upgrade pip
/usr/bin/python3 -m pip install --user -r rain-orin/requirements.txt
sudo apt install -y python3-gi python3-gst-1.0 \
    gstreamer1.0-tools gstreamer1.0-plugins-{base,good,bad,ugly}

# 7. Build ROS 2 workspace
cd ~/Telep/rain-orin/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 8. Install third-party SDKs (NOT in repo, license-restricted)
#    - 求之科技 AIRBOT SDK: deb + Python wheel (request from upstream)
#      sudo dpkg -i ~/Downloads/airbot-arm_*.deb
#      pip install --user ~/Downloads/arm_sdk-*.whl
#    - 达妙 DM-CAN lib: Apache-2.0 from upstream cmjang/DM_Control_Python
#      (an MIT copy is already vendored under rain-orin/dm_motor/DM_Control_Python/
#       since pre-PR-1; no separate install needed)

# 9. SocketCAN for AIRBOT (slcan/gs_usb kernel modules — must be rebuilt
#    from upstream v5.15 source against shipped L4T -headers if missing;
#    see agent-bus/CONTEXT/devices.md)
sudo systemctl enable slcan@0 slcan@1
sudo systemctl enable airbot-arm@2-50051 airbot-arm@3-50052

# 10. Enable Orin systemd USER services (after deploying unit files)
loginctl enable-linger bit
mkdir -p ~/.config/systemd/user
cp ~/Telep/rain-orin/systemd/*.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now \
    zed-hw-gst zed-hw-lk \
    teleop-lk-data-bridge \
    teleop-safety teleop-dm-motor

# 11. Verify
journalctl --user -u zed-hw-gst -n 50            # video pipeline
ros2 topic list | grep vp                         # /vp/head_pose visible
ros2 service list | grep arms                     # 14 safety services
# From CangJie or another Cyclone client on 192.168.8.0/24:
#   ros2 topic echo /vp/head_pose
```

## Files in this directory

```
rain-orin/
├── README.md                                ← 本文件
├── ROS2_SETUP.md                            ← Humble setup notes (legacy, partly超越)
├── EXPERIMENTAL_VALIDATION_2026-04-08.md    ← 早期实验日志 (历史)
├── requirements.txt                         ← Python deps
├── setup_linux.sh                           ← initial setup helper (legacy; review before run)
├── gst_hw_tcp.sh                            ← GStreamer NVENC pipeline (camera → TCP 5004)
│
├── zedmini_livekit_sender.py                ← LiveKit publisher (大部分被 lk CLI / zed-hw-lk.service 替代)
├── generate_livekit_tokens.py               ← JWT token 生成 (env-driven now)
│
├── 20260422-cc-pose_data_receiver.py        ← VP pose receiver test
├── 20260422-cc-zedmini_hw_rtmp_sender.py    ← NVENC RTMP Ingress 试验 (历史, 已被 lk CLI 替代)
├── 20260424-cc-keyboard_teleop_node.py      ← WASD/QE → /cmd_vel (调试用)
├── 20260424-cc-teleop_safety_node.py        ← 14 std_srvs/Trigger AIRBOT 安全服务 + bus-wide /arms/emergency_stop_all
├── 20260425-cc-cloud_to_ros_bridge.py       ← LiveKit data → /cmd_vel (PR-3 将扩展 /lift_cmd)
├── 20260425-cc-cloud_teleop_bridge_node.py  ← (DEPRECATED, livekit+rclpy 同进程冲突, 见 SECURITY.md)
│
├── ros2_ws/src/
│   ├── teleop_livekit_bridge/               ← VP↔ROS 2 bridge (Python ROS 2 包)
│   │   └── teleop_livekit_bridge/
│   │       ├── lk_data_bridge_node.py       ← head pose receive + cmd dispatch
│   │       ├── livekit_bridge_node.py       ← (legacy, sender 形式)
│   │       └── sender_core.py               ← 视频 sender helper (大部分被 systemd unit 替代)
│   ├── teleop_dm_motor/                     ← DM4310 head motor controller (mock 默认)
│   ├── teleop_msgs/                         ← HeadPose / RecalibrateAxis / SetPreset 消息
│   ├── teleop_bringup/                      ← launch files (full / sender_only / motor_only)
│   ├── airbot_msgs/, airbot_description/, airbot_bringup/   ← AIRBOT URDF + bridge
│
├── dm_motor/
│   ├── PRESETS.md                           ← L1-L5 跟手档位说明
│   ├── SESSION_LOG.md                       ← Agent 会话日志 (历史 secret 已 redact)
│   ├── calibration.example.json             ← 校准数据示例 (真实 calibration 在 .gitignore)
│   ├── DM_Control_Python/                   ← 达妙官方 Python 驱动 (MIT, 上游 cmjang/DM_Control_Python)
│   └── scripts/
│       ├── dm_motor_zero_calibration.py     ← 永久零点 (首次安装跑一次)
│       ├── dm_motor_axis_verify.py          ← 互动测物理范围 + 验证轴对应
│       ├── dm_motor_half_amp_sweep.py       ← 半幅摆动验证 (可选)
│       └── dm_motor_vp_control.py           ← VP → 电机 实时控制 (含 5 档 preset)
│
├── setup/
│   └── cyclonedds-eno1.xml                  ← Cyclone DDS XML 模板 (锁 eno1)
│
└── systemd/                                 ← user systemd units (RMW=Cyclone env 已 baked in)
    ├── zed-hw-gst.service                   ← GStreamer NVENC → TCP 5004
    ├── zed-hw-lk.service                    ← lk CLI publish → Aliyun LiveKit
    ├── zed-hw-pose.service                  ← legacy raw pose receiver (与 teleop-lk-data-bridge 互斥)
    ├── teleop-lk-data-bridge.service        ← VP data → ROS 2 /vp/head_pose
    ├── teleop-safety.service                ← 14 AIRBOT 安全服务
    ├── teleop-dm-motor.service              ← DM4310 head motor 控制 (mock 默认)
    └── lk-sender.service                    ← legacy (zedmini_livekit_sender.py)
```

## Key data flows (this directory's slice)

| Direction | Path |
|---|---|
| Video Orin → VP | ZED Mini → `gst_hw_tcp.sh` (NVENC, intra-only 1-slice 30fps 6Mbps Main) → TCP 5004 → `zed-hw-lk.service` (lk CLI) → Aliyun → VP |
| VP pose → Orin | VP LiveKit data → `teleop-lk-data-bridge.service` → `/vp/head_pose` topic → `teleop-dm-motor.service` (mock until DM4310 hooked) |
| ASUS cmd_vel → CangJie | ASUS → LiveKit data → `cloud_to_ros_bridge.py` → ROS 2 `/cmd_vel` (Cyclone DDS) → CangJie chassis (PR-3) |
| AIRBOT safety | (any client) → 14 std_srvs/Trigger via `teleop-safety.service` → arm freeze / clear / re-enable / e-stop-all |

## Known debt / caveats

- `setup_linux.sh` 是 PR-1 之前的脚本, 路径可能还引用 `device-a-linux/`; 用前请通读. PR-2 不动它的内容, 仅做 import 与扫描.
- `dm_motor/SESSION_LOG.md` 历史上含明文 secret literal (line 96), PR-2 已 redact (替换为指向 `../.env.example` 的引用).
- `cloud_to_ros_bridge.py` **仅** parse `cmd_vel`; `lift_cmd` 解析在 PR-3 的 CangJie 端 + Orin 端联合添加.
- `cloud_teleop_bridge_node.py` 是早期单进程 livekit+rclpy 实现, 因 SDK 冲突 freezing (见 [agent-bus `feedback_livekit_rclpy_same_proc_conflict`](../SECURITY.md)) 已被 `cloud_to_ros_bridge.py` 取代; 保留作历史参考.
- `lk-sender.service` / `zed-hw-pose.service` 标为 legacy. 生产运行的是 `zed-hw-gst` + `zed-hw-lk` (双 service 替代 lk-sender) 与 `teleop-lk-data-bridge` (替代 zed-hw-pose).
- 第三方 SDK (求之科技 AIRBOT, 达妙 DM-CAN) 不在仓库; 见 [`SECURITY.md`](../SECURITY.md) 的红线说明.

## See also

- [`../ARCHITECTURE.md`](../ARCHITECTURE.md) — 系统总览
- [`../SECURITY.md`](../SECURITY.md) — env var 政策 + secret rotation 流程
- [`../.env.example`](../.env.example) — 必需 env vars 模板
- [`../rain-cangjie/README.md`](../rain-cangjie/README.md) — CangJie 底盘 + lift 端 (PR-3)
