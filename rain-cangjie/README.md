# rain-cangjie/ — CangJie Pi (Rain 底盘 + 升降)

> Last updated: 2026-04-26 (PR-3). See top-level [`../ARCHITECTURE.md`](../ARCHITECTURE.md) for system context.

## Role

The CangJie Raspberry Pi 4 是 **Rain-side 低层控制器**: 4-轮 swerve 舵轮 IK + DM3519 lift 升降. 跑一个 ROS 2 节点订阅跨机来的 `/cmd_vel` (底盘) 和 `/lift_cmd` (升降), 解算后通过 USB-CAN 写底层电机.

## Hardware

- Raspberry Pi 4 Model B Rev 1.5, 4 GB RAM
- Hostname `CangJie`, sudo user `cangjie`
- Two USB-CAN dongles (calibrated, but **当前未插**, 见 "Known debt"):
  - **ACM0** @ 500 kbit CANopen → 2× ZLAC8015D 驱动器 → 4 个驱动轮 (FL/FR/RL/RR)
  - **ACM1** @ 1 Mbit MIT/一拖四 共享总线 → 4× DM4310 (舵向, MIT 协议) + 1× DM3519 (升降, 一拖四协议)
- 当前 USB-CAN 未插 → MOCK 模式默认; chassis_controller_v3 不实发 CAN, 只打印解算结果

## Network

- **Primary**: `eth0 192.168.8.7/24` (Rain LAN island; 千兆非网管交换机 + Huawei CPE 5S 5G 上行)
- **Fallback**: `wlan0 192.168.0.150/24` (家里 WiFi)
- **Tailscale**: `100.70.149.16` (`thedaosheng@` tailnet)
- ROS 2 跨机 multicast 通过 Cyclone DDS 锁 `eth0` 接口 (双 link 不锁会跑 wlan0, 静默 fail)

## Bring-up from zero

```bash
# 1. Clone repo
git clone https://github.com/thedaosheng/Remote_Control.git ~/Remote_Control
cd ~/Remote_Control

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
export CYCLONEDDS_URI=file:///home/cangjie/cyclonedds.xml
EOF
source ~/.bashrc

# 4. Copy Cyclone XML to home (锁 eth0 multicast)
cp rain-cangjie/setup/cyclonedds-eth0.xml ~/cyclonedds.xml

# 5. LiveKit credentials (只 ros_to_cloud_bridge.py 需要; chassis_controller 自身不连云)
cp .env.example .env
# edit .env, fill in LIVEKIT_API_KEY + LIVEKIT_API_SECRET (从 Aliyun /etc/livekit.yaml 取)
set -a && source .env && set +a

# 6. Install Python deps
/usr/bin/python3 -m pip install --user --upgrade pip
/usr/bin/python3 -m pip install --user \
    python-can==3.3.2 pyserial==3.5 numpy \
    livekit livekit-api    # 后两个仅 ros_to_cloud_bridge.py 需要

# 7. (可选) Build teleop_chassis_safety ROS 2 包
cd ~/Remote_Control/rain-cangjie/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 8. (USB-CAN 插好后) 配 slcand
#    sudo slcand -o -s8 /dev/ttyACM0 can0  (500kbit, ZLAC)
#    sudo slcand -o -s8 /dev/ttyACM1 can1  (1Mbit, DM4310 + DM3519)
#    Or use the project's slcan@.service template after installing it.

# 9. 跑 chassis_controller v3 (mock, 默认; 当前 USB-CAN 未插的状态)
cd ~/Remote_Control
python3 rain-cangjie/20260427-cc-chassis_controller_node_v3.py

# 10. Verify (在另一个 shell)
ros2 topic list | grep -E 'cmd_vel|lift_cmd|chassis_state'
ros2 topic pub /lift_cmd std_msgs/msg/Float32 "data: 0.30" -r 5
# chassis_v3 应打印: [lift] norm=+0.30 → I=+1.50 A → int16=+1228 → CAN 0x200 [0x04 0xcc 0x00 0x00 0x00 0x00 0x00 0x00]
```

## Lift integration demo (mock mode, 端到端无硬件)

四段链路, 任一段掉线都各自输出诊断:

1. **ASUS** (PR-4) 跑 `operator-asus/asus_teleop_gui.py` — Tkinter GUI, G 键发 `+0.5`, H 键发 `-0.5`, T 键停; 把它们包成 `{"type":"lift_cmd","speed":±X}` 走 LiveKit data channel.
2. **Aliyun LiveKit Server** 转发到 room `teleop-room`.
3. **Orin** 跑 `rain-orin/20260425-cc-cloud_to_ros_bridge.py` — 订阅 LiveKit data, 派发 `cmd_vel` / `lift_cmd` 到 ROS 2.
4. **CangJie** 跑 `rain-cangjie/20260427-cc-chassis_controller_node_v3.py` — 订阅 `/lift_cmd` (Float32) → 乘 `lift_max_current_a` → DM3519 一拖四 0x200 帧 → mock log.

Mock 日志期望 (按 `speed=+0.30, lift_max_current_a=5.0` 默认):

```
[lift] norm=+0.30 → I=+1.50 A → int16=+1228 → CAN 0x200 [0x04 0xcc 0x00 0x00 0x00 0x00 0x00 0x00]
```

**直测 (跳过 LiveKit, 适合 chassis_v3 本地 debug)**:

```bash
# Terminal A: chassis_v3
python3 rain-cangjie/20260427-cc-chassis_controller_node_v3.py

# Terminal B: 直接 pub /lift_cmd (在同 ROS 2 域的任意一台机器, 比如 Orin)
ros2 topic pub /lift_cmd std_msgs/msg/Float32 "data: 0.5" -r 5

# 1 秒看 chassis_v3 mock log; Ctrl-C B 后, watchdog 0.5s 后会触发归零警告
```

## Files in this directory

```
rain-cangjie/
├── README.md                                    ← 本文件
├── 20260331-cc-system_topology.md               ← 早期系统拓扑 (history; pre-pivot)
│
├── 20260425-cc-ros_to_cloud_bridge.py           ← /chassis_state → LiveKit data (PR-2 explicitly skipped, PR-3 imports)
├── 20260427-cc-chassis_controller_node_v3.py    ← ★ 主节点: /cmd_vel + /lift_cmd → swerve IK + DM3519 lift
│
├── scripts/                                     ← 16 个 4/18-4/20 调试脚本 (steer/scan/zlac/dm3519/kinematics)
│   ├── 20260418-cc-pi_steer_test.py
│   ├── 20260420-cc-pi_3519_test.py              ← DM3519 一拖四协议参考 (header 含完整协议描述)
│   ├── 20260420-cc-pi_kinematics_demo.py        ← 7 模式舵轮演示 (canonical optimize_wheel 来源)
│   ├── 20260420-cc-pi_zlac_*.py                 ← ZLAC 调试 (scan/change_nodeid/wheels_demo/...)
│   ├── 20260420-cc-pi_dm3519_probe.py
│   └── ... (其余 11 个)
│
├── legacy/                                      ← 已被取代, 留作参考
│   └── 20260424-cc-chassis_controller_node.py   ← v2: 仅 chassis IK; optimize_wheel 双 flip bug
│
├── ros2_ws/src/teleop_chassis_safety/           ← 4/24 sub-agent delivery (chassis_safety + zlac_diag + damiao_diag)
│   ├── chassis_safety_node.py                   ← 主 safety node
│   ├── zlac_diag.py / damiao_diag.py            ← 实时错误码 + 诊断
│   ├── zlac_errors.yaml / damiao_errors.yaml    ← 错误码字典
│   ├── config/chassis_safety.yaml               ← 阈值配置
│   ├── systemd/chassis-safety.service           ← systemd unit
│   ├── package.xml / setup.py / setup.cfg       ← ament_python 标准
│   └── test/test_chassis_safety_node.py         ← unit tests
│
└── setup/
    └── cyclonedds-eth0.xml                      ← Cyclone DDS XML 模板 (锁 eth0)
```

## chassis_controller v3 vs v2

| 特性 | v2 (`legacy/`) | v3 (顶层) |
|---|---|---|
| /cmd_vel 订阅 | ✓ | ✓ |
| /lift_cmd 订阅 | ✗ | **✓ 新增** |
| DM3519 一拖四 mock 输出 | ✗ | **✓ 新增** (CAN 0x200 frame) |
| optimize_wheel 实现 | bug: 双 flip = no-op 舵向 | canonical (181 demo 一致) |
| Watchdog | 单一 (cmd_vel) | **双 (cmd_vel + lift) 互不影响** |
| /chassis_state 内容 | wheels only | wheels + lift |
| ROS 2 param `lift_max_current_a` | n/a | **新增, 默认 5.0 A** |

v2 暂留 `legacy/` 作历史参考 — 切换链路 + 跨 PR review 时方便对照. 未来 v3 sed 稳定后可以删.

## Known debt

- **USB-CAN 未插** → `mock=true` 是当前唯一可跑模式. `chassis_v3` 切 `mock:=false` 会立刻 raise `NotImplementedError`. 真模式 wiring (slcand init + ZLAC CANopen + MIT/一拖四 双线程) 留给下个 PR, USB-CAN 接好后做.
- **`lift_max_current_a=5.0 A` 是猜测值**, 没在真机标定. 真接 DM3519 + 实际 lift 框架前 — 应分两步: (a) 空载 0.3 A 试 LED 反馈正常, (b) 带载从 0.5 → 1.5 A 慢爬测响应; 然后再调 max.
- **chassis_safety 与 chassis_controller_v3 还没集成** — 4/24 deploy 的 `teleop_chassis_safety` 包是独立 ROS 2 节点, 当前通过 ROS 2 topic 间接联动 (state pub/sub), 没有显式同步. 未来一个 PR 可以折叠或形式化 topic 合约.
- **DM3519 19.2:1 减速比** 还没标进 v3 (做电流 → 实际扭矩转换); 当前 `lift_max_current_a` 直接走电流域, 后续力反馈 / 反向力计算需要补.
- **`~/ops-backend/` (FastAPI 车队 dashboard) 没进本 PR** — 4/24 写, 目前在 CangJie `~/ops-backend/` + Aliyun `/root/ops-backend/`. PR-3 故意排除原因: (a) 跟 chassis 控制 orthogonal (前端 dashboard, 不该跟底盘绑死); (b) `main.py` 自带 `LIVEKIT_API_SECRET="teleop_secret_key_2026"` default value, 单独 PR 走 secret env 改造更干净. 留给未来 cloud/ops-backend/ 子目录的独立 PR.

## See also

- [`../ARCHITECTURE.md`](../ARCHITECTURE.md) — 系统总览
- [`../SECURITY.md`](../SECURITY.md) — env var 政策 + secret rotation 流程
- [`../.env.example`](../.env.example) — 必需 env vars 模板
- [`../rain-orin/README.md`](../rain-orin/README.md) — Orin 端 (高层 ROS 2 + 视频 pipeline + bridges)
