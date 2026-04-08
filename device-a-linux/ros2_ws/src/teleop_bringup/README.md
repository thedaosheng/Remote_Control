# teleop_bringup — Teleop 系统启动包

> ROS2 化的 Teleop 系统总入口。一行命令起 LiveKit sender + 达妙电机控制 + 调试工具。

---

## 一键启动

### 全套（默认）
```bash
ros2 launch teleop_bringup full.launch.py
```
启动:
- `livekit_bridge` 节点 (推视频 + 接 VP head pose, publish 到 /vp/head_pose)
- `dm_motor_controller` 节点 (订阅 /vp/head_pose, 200Hz MIT 控制达妙双电机)
- 默认 preset = L4 (跟手档)

### 切档位（一句话）
```bash
ros2 launch teleop_bringup full.launch.py preset:=L5
ros2 launch teleop_bringup full.launch.py preset:=L1   # 安全档
```

### 只起电机不起 sender（mock pose 测试用）
```bash
ros2 launch teleop_bringup motor_only.launch.py preset:=L4
```

### 只起 sender 不起电机（视频调试）
```bash
systemctl --user stop lk-s2     # 先停老的 systemd
ros2 launch teleop_bringup sender_only.launch.py
```

### 没硬件时（mock 模式）
```bash
ros2 launch teleop_bringup full.launch.py mock_motor:=true
ros2 launch teleop_bringup motor_only.launch.py mock_motor:=true preset:=L3
```

### 调试模式（自动开 rqt_plot + rosbag）
```bash
ros2 launch teleop_bringup full.launch.py debug:=true
# rosbag 录到 /tmp/teleop_bag, 退出后用 ros2 bag play 回放
```

---

## 运行时调参（不重启）

启动一次 launch 之后，开另一个终端跑这些：

### 改 KP / KD
```bash
ros2 param set /dm_motor_controller kp 30.0
ros2 param set /dm_motor_controller kd 1.5
```

### 切预设档位（批量更新所有相关参数）
```bash
ros2 param set /dm_motor_controller preset L5
# 自动应用 L5 的 kp=30, kd=0.8, rate_limit=600, ff_gain=1.5, ff_ema_alpha=0.8
```

### 改极性（如果转向反了）
```bash
ros2 param set /dm_motor_controller sign_yaw -1
ros2 param set /dm_motor_controller sign_pitch -1
```

### 改速度前馈
```bash
ros2 param set /dm_motor_controller feedforward_enabled false   # 关掉前馈
ros2 param set /dm_motor_controller ff_gain 0.5                  # 减半
ros2 param set /dm_motor_controller ff_ema_alpha 0.3             # 更平滑
```

### 看当前所有参数
```bash
ros2 param list /dm_motor_controller
ros2 param dump /dm_motor_controller   # dump 成 yaml
```

---

## Service 调用

### 失能电机（允许手动掰）
```bash
ros2 service call /dm_motor/disable std_srvs/srv/Trigger
```

### 重新使能
```bash
ros2 service call /dm_motor/enable std_srvs/srv/Trigger
```

### 把当前位置永久写 Flash 当零点
```bash
ros2 service call /dm_motor/set_zero std_srvs/srv/Trigger
```

### 缓慢回零
```bash
ros2 service call /dm_motor/ramp_to_zero std_srvs/srv/Trigger
```

### 切预设（service 版本，跟 ros2 param set preset 等价）
```bash
ros2 service call /dm_motor/set_preset teleop_msgs/srv/SetPreset "{preset_name: 'L5'}"
```

---

## 调试工具

### 实时看 VP head pose 数据
```bash
ros2 topic echo /vp/head_pose          # 完整 HeadPose (含四元数)
ros2 topic echo /vp/head_pose_rpy      # 简化 RPY (rqt_plot 友好)
```

### rqt_plot 实时曲线
```bash
# 自动启动 (用 debug 模式)
ros2 launch teleop_bringup full.launch.py debug:=true

# 或者手动启动
ros2 run rqt_plot rqt_plot \
  /vp/head_pose_rpy/vector/x \
  /vp/head_pose_rpy/vector/y \
  /vp/head_pose_rpy/vector/z
```

### 看电机实际位置（跟踪误差分析）
```bash
ros2 topic echo /dm_motor/joint_states
ros2 topic echo /dm_motor/target_states

# 同时看目标和实际, 算跟踪误差
ros2 run rqt_plot rqt_plot \
  /dm_motor/target_states/position[0] \
  /dm_motor/joint_states/position[0]
```

### rosbag 录制 (离线分析用)
```bash
# 自动录 (debug 模式)
ros2 launch teleop_bringup full.launch.py debug:=true
# 退出 launch 后:
ros2 bag play /tmp/teleop_bag

# 或者手动录指定 topic
ros2 bag record -o my_record /vp/head_pose /dm_motor/joint_states
```

### 频率检查
```bash
ros2 topic hz /vp/head_pose            # 应该 ~30 Hz
ros2 topic hz /dm_motor/joint_states   # 应该 ~100 Hz
```

### 节点和图
```bash
ros2 node list                         # 看哪些节点在跑
ros2 node info /dm_motor_controller    # 看节点的 sub/pub/service
rqt_graph                              # 可视化节点拓扑
```

---

## Mock pose 测试 (没 VP 也能玩电机)

```bash
# 终端 1: 启动 motor_only
ros2 launch teleop_bringup motor_only.launch.py

# 终端 2: 用 ros2 topic pub 模拟 30Hz pose 数据
ros2 topic pub --rate 30 /vp/head_pose teleop_msgs/msg/HeadPose \
  '{header: {frame_id: "vp_head_initial"},
    pitch_deg: 10.0, yaw_deg: 5.0, roll_deg: 0.0,
    pos_x: 0.0, pos_y: 1.5, pos_z: 0.0,
    quat_x: 0.0, quat_y: 0.0, quat_z: 0.0, quat_w: 1.0}'

# 电机应该会动到 pitch=10° yaw=5° 的位置
```

---

## 常见排查

### 1. 启动失败 "Motor communication failed"
- 检查 24V 电源是否接通
- 检查 USB-CAN 串口路径: `ls /dev/serial/by-id/` 找 HDSC
- 用裸 Python 测一下: `python3 -c "..."` (见 `dm_motor/SESSION_LOG.md`)
- 临时用 mock 模式跑通其他测试: `mock_motor:=true`

### 2. 启动失败 "Device '/dev/video0' is busy"
- 老的 systemd lk-s2 还在占 ZED Mini
- `systemctl --user stop lk-s2` 然后重启 launch

### 3. preset 切换没生效（kp 没变）
- 这是个已知 bug, 启动时 preset:=L? 通过 launch 命令行传入会触发批量更新
- 但**初始化阶段** callback 还没注册, 所以代码里加了启动时的"显式应用 startup_preset"步骤
- 运行时改 preset 用 `ros2 param set /dm_motor_controller preset L?` 应该立即生效

### 4. ros2 launch 之后看到多个 /dm_motor_controller
- 之前的测试进程没干净退, 留下 zombie
- `pkill -9 -f dm_motor_controller_node` 清理

### 5. cmake 报 "ament_package not found"
- shell 环境问题
- 重开 bash 后再 `source /opt/ros/humble/setup.bash` 然后 `colcon build`
- 不要在同一个 shell 里反复 source 后又 build

---

## 文件结构

```
teleop_bringup/
├── README.md              ← 本文件
├── launch/
│   ├── full.launch.py        # 完整系统 (sender + 电机)
│   ├── motor_only.launch.py  # 只电机
│   └── sender_only.launch.py # 只 sender
├── config/
│   └── (留给后续 yaml 配置)
├── package.xml
└── setup.py
```

## 相关 package

| Package | 说明 |
|---------|------|
| `teleop_msgs` | 自定义消息 (HeadPose) 和服务 (SetPreset, RecalibrateAxis) |
| `teleop_livekit_bridge` | LiveKit ↔ ROS2 桥接节点 (sender_core 包装) |
| `teleop_dm_motor` | 达妙电机控制节点 (含 5 档预设 + 6 service) |

## 老脚本对照（迁移前后）

| 老脚本 | ROS2 替代 | 命令 |
|--------|----------|------|
| `scripts/zedmini_livekit_sender.py` | `livekit_bridge_node` | `ros2 run teleop_livekit_bridge livekit_bridge_node` |
| `scripts/dm_motor_vp_control.py` | `dm_motor_controller_node` | `ros2 run teleop_dm_motor dm_motor_controller_node` |
| `--preset L4` | `-p preset:=L4` | `ros2 launch ... preset:=L4` |
| `--kp 30 --kd 1.0` | `-p kp:=30.0 -p kd:=1.0` | 同上 |
| Ctrl+C 安全归零 | service `/dm_motor/ramp_to_zero` | 自动 + service 触发 |

老脚本仍然保留在 `scripts/`，可以作为 fallback。新的 ROS2 路径是推荐方式。
