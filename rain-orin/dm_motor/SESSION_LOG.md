# Agent 会话日志 — 达妙电机端到端调试

> 这个文件是 **不同 Agent 之间交互的总结**。每次重要的会话/决定/教训都写在这里。
> Agent 在新机器或新任务上接手前应该先读这个文件,避免走重复的弯路。

---

## 2026-04-07 — 完整调试链 (从硬件第一次上电到 VP 控制电机跑通)

### 这次会话做了什么

1. **修 sender 30fps 限制**: `publish_options.video_encoding.max_framerate=100` (默认 30, SDK 砍帧)
2. **CAM_FPS 60 → 100**: ZED Mini WVGA 1344x376 硬件离散档 100/60/30/15
3. **改 sender 日志为纯物理命名 RPY**: 删掉数学命名 (Roll(绕X)/Pitch(绕Y)/Yaw(绕Z)) 避免歧义
4. **达妙四轮交互校准**:
   - 第一次设零点 (用户最初摆的姿态) → 测两侧极值 (Yaw/Pitch 各掰一次)
   - 半幅摆动验证物理映射 (CAN 0x01 = Yaw 左右转头, CAN 0x02 = Pitch 俯仰)
5. **vp_control 加速度前馈**: dq_target 用 EMA 平滑后的 vp pose 微分,跟手感大幅提升
6. **vp_control 5 档预设 (L1-L5)**: 一键切换 KP/KD/rate-limit/ff-gain/ff-ema 组合
7. **重新校准零点 (路径 B)**: 把电机停在工作区中央位置时重设 Flash 零点 → 工作区相对零点对称
8. **Pitch 通讯异常排查**: read_motor_param=None → 物理 24V/CAN 重插 → 恢复
9. **二次极值测量** (Yaw/Pitch 各两侧)
10. **Pitch 工作区手动剪到 ±60 半幅**: 中心保持 +4.22°
11. **vp_control 映射改为 VP 中立 → 电机零点 (0, 0)**: 不再用 workspace_center

### 这次会话的关键决定 (按时间顺序)

| # | 决定 | 原因 |
|---|------|------|
| 1 | 用 HDSC USB-CAN 不用 SocketCAN | 达妙官方驱动只支持 pyserial CDC 模式 |
| 2 | sender 必须用 /usr/bin/python3 | conda 缺 GStreamer python-gi 模块 |
| 3 | sender publish 必须显式设 max_framerate | LiveKit Python SDK 默认 30 fps |
| 4 | sender 日志用纯物理命名 | 数学命名 (Roll(绕X)) vs 物理命名 (PITCH 点头) 容易让人误判 swap 反了 |
| 5 | 达妙物理映射: CAN 0x01=Yaw, CAN 0x02=Pitch | 实测验证 (用户掰电机看哪个轴动) |
| 6 | 达妙极性: yaw_sign=+1, pitch_sign=+1 | 实测验证 (默认就对) |
| 7 | vp_control 加速度前馈用 EMA α=0.5 + ff_gain=1.0 | 跟手感 vs 噪声平衡, L4 档默认 |
| 8 | 5 档预设 L1-L5 | 用户能根据使用场景一键切换 |
| 9 | 重新校准零点 = 物理工作区中央 | 让左右余量对称, 第一次零点设歪了 |
| 10 | Pitch 工作区手动剪到 ±60 半幅 | 物理可达 187° 太大没必要, 限制软件用 120° |
| 11 | VP 中立 → 电机零点 (0, 0) | 用户明确诉求: 进 Immersive 时电机有"绝对零点", workspace_center 不直观 |

### 重要踩坑 (避免重复)

1. **达妙串口路径**: 默认 `/dev/ttyACM0` (HDSC), 不是 `/dev/ttyACM2` (Touch 力反馈笔). 用 `ls /dev/serial/by-id/` 找带 `HDSC_CDC_Device` 的链接。
2. **SDK 异步刷新延迟**: `mc.refresh_motor_status()` 之后立即 `getPosition()` 第一次会读到默认值 0.000°. 必须 warmup 几次再读, 取中位数。
3. **Pitch 通讯断了表现**: `read_motor_param` 返回 `None`, 但 `getPosition()` 不报错, 仍返回默认值 0.000°. 容易误以为电机在零位, 必须先 `read_motor_param(PMAX)` 验证通讯。
4. **set_zero_position 失败的"无声故障"**: 如果某个电机通讯断了, `mc.set_zero_position(motor)` 不会抛异常, 也不会写 Flash, 但你不知道。每次设零点后必须验证。
5. **达妙 J4310 自锁**: 失能后用手能掰动, 但 enable 状态下掰不动 (会被位置环拉回)。
6. **VP 进 Immersive 那一帧 = 零点**: 进入时如果不是"自然朝前"姿态, 后续相对偏移就一直偏。要重设: Disconnect → Connect → 自然朝前 → 再进 Immersive。
7. **MIT 模式速度前馈的"无声噪声"**: dq_target 直接传 vp pose 微分会让电机震 (高频噪声放大)。必须 EMA 平滑后再传。
8. **达妙物理可达远超用户需求**: Pitch 单轴可达 187°, 但人头物理活动 ±60° 就够了。靠软件 max/min 限制。

### 当前的"全局零点"定义

- **电机1 (Yaw)**: Flash 零点 = 物理工作区中央 (在 2026-04-07 19:00 重设过)
- **电机2 (Pitch)**: Flash 零点 = 用户最初校准位置 (没重设, 但巧合接近物理中央, 偏差 +4.22°)
- **vp_control 映射**: VP 中立 → 电机 (0°, 0°) (= 上述 Flash 零点)

### Linux 端关键文件路径 (开发机 rhz@linux)

| 用途 | 路径 |
|------|------|
| sender (推视频 + 接 VP pose) | `/home/rhz/teleop/scripts/20260406-cc-zedmini_livekit_sender.py` |
| sender systemd unit | `~/.config/systemd/user/lk-s2.service` (transient) 或 `lk-sender.service` |
| 达妙驱动库 (vendor) | `/home/rhz/teleop/DM_Control_Python/` |
| 零点校准 | `/home/rhz/teleop/scripts/20260407-cc-dm_motor_zero_calibration.py` |
| 互动测物理范围 | `/home/rhz/teleop/scripts/20260407-cc-dm_motor_axis_verify.py` |
| 半幅摆动测试 | `/home/rhz/teleop/scripts/20260407-cc-dm_motor_half_amp_sweep.py` |
| **VP→电机 实时控制 (核心)** | `/home/rhz/teleop/scripts/20260407-cc-dm_motor_vp_control.py` |
| 校准数据 (机器特定) | `/home/rhz/teleop/scripts/dm_motor_calibration.json` |
| 校准备份 | `dm_motor_calibration.json.bak_before_recalib` (Yaw 重设零点前) |
| 校准备份 | `dm_motor_calibration.json.bak_before_recollect` (Pitch 二次测量前) |
| 档位说明 | `/home/rhz/teleop/scripts/dm_motor_vp_control_PRESETS.md` |

### Mac VP 端关键文件路径

| 用途 | 路径 |
|------|------|
| Xcode project | `/Users/Zhuanz/teleop/RemoteControl/` |
| LiveKit 连接管理 | `RemoteControl/RemoteControl/LiveKit/LiveKitManager.swift` |
| Head pose tracker (共享缓存) | `RemoteControl/RemoteControl/Tracking/HeadPoseTracker.swift` |
| Metal 立体渲染 | `RemoteControl/RemoteControl/Rendering/StereoVideoRenderer.swift` |
| Metal shader | `RemoteControl/RemoteControl/Rendering/VideoShaders.metal` |

### 三端连接信息

```
A端 Linux 工控机 (rhz@?)
  /home/rhz/teleop/scripts/20260406-cc-zedmini_livekit_sender.py
  systemd: lk-s2 (transient) — 100 fps NVENC H264
  ↓
B端 阿里云 LiveKit Server
  ssh root@39.102.113.104
  /etc/livekit.yaml
  端口 7880 (WebSocket) / 7881 (RTC TCP) / 50000-60000 UDP
  房间: teleop-room  Key/Secret: 见 ../.env.example + ../SECURITY.md (历史值已 redact)
  ↓
C端 Apple Vision Pro + Mac
  ssh Zhuanz@192.168.0.212
  /Users/Zhuanz/teleop/RemoteControl/ (Xcode 26.4)
```

### 下次 Agent 接手前必读

1. 读这个文件
2. 读 `device-a-linux/README.md` (硬件 + 系统要求 + 故障排查)
3. 读 `device-a-linux/dm_motor/PRESETS.md` (跟手感档位)
4. 检查电机当前位置: `/usr/bin/python3 -c "...stable_read..."` (本文件下一节有片段)
5. 如果不确定状态, **先 `read_motor_param(PMAX)` 验证通讯**, 不要假设 getPosition 返回的就是真实位置

### 快速诊断命令片段

```bash
# 验证两个电机都在线
/usr/bin/python3 -c "
import sys; sys.path.insert(0, '/home/rhz/teleop/DM_Control_Python')
import serial; from DM_CAN import Motor, MotorControl, DM_Motor_Type, DM_variable
ser = serial.Serial('/dev/ttyACM0', 921600, timeout=1)
mc = MotorControl(ser)
m1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
m2 = Motor(DM_Motor_Type.DM4310, 0x02, 0x12)
mc.addMotor(m1); mc.addMotor(m2)
import time; time.sleep(0.3)
print('Yaw   PMAX =', mc.read_motor_param(m1, DM_variable.PMAX))  # 期望 12.5
print('Pitch PMAX =', mc.read_motor_param(m2, DM_variable.PMAX))  # 期望 12.5
ser.close()
"

# 看 sender 状态 + RPY 实时数据
journalctl --user -u lk-s2 -f | grep -A 3 "Pose #"

# 看 sender fps 是否到 100
journalctl --user -u lk-s2 --no-pager -n 20 | grep "推帧"
```

### 留给下一个 Agent 的待办

- [x] ✅ 整套代码 ROS2 化改造 (2026-04-08 完成, 详见下一节)
- [ ] VP pose 30Hz → 60/90Hz 测试 (改 LiveKitConfig.poseFrequency 一行常量, 在 Mac VP 端 Swift 代码里)
- [ ] sender 端是不是还有别的隐藏参数没设 (比如 jitter buffer, NACK)
- [ ] 实测端到端 motion-to-photon 延迟 (用屏幕计时器拍照法)
- [ ] 把 ROS 化代码跟真实 VP 一起做端到端验证 (我做完了 mock + 真机两种 e2e 测试,
      但还没拿真实 VP 的 head pose 走全流程, 等用户配合)
- [ ] 加 dm_motor_calibration_node (互动测物理范围的 ROS service 包装)
      —— 当前的方案是 axis_verify 老脚本仍然可用,calibration_node 不是必需

---

## 2026-04-08 — ROS2 化改造 (完整 6 stage)

### 这次会话做了什么

按用户要求把整套 Linux 代码 ROS2 化改造, 目标:
- 一行命令起整套 (`ros2 launch teleop_bringup full.launch.py`)
- 一句话切调试参数 (`preset:=L5` / `mock_motor:=true` / `debug:=true`)
- 运行时 dynamic param 改 KP/KD/preset 不重启
- 老 scripts/ 完全不动作为 fallback

### 创建的 4 个新 ROS2 package (在 device-a-linux/ros2_ws/src/)

| Package | 类型 | 作用 |
|---------|------|------|
| `teleop_msgs` | ament_cmake | 自定义 msg/srv: HeadPose / SetPreset / RecalibrateAxis |
| `teleop_livekit_bridge` | ament_python | LiveKit ↔ ROS2 桥, 包装老 sender + publish /vp/head_pose |
| `teleop_dm_motor` | ament_python | 达妙双电机 ROS 控制 (核心), 含 200Hz 控制环 + 5 service + dynamic params + mock_mode |
| `teleop_bringup` | ament_python | launch 总入口, 含 README + 3 个 launch 文件 |

### 关键设计点

- **sender_core.py**: 从老 sender 复制, 加一个 `ROS_POSE_CALLBACK` 全局 hook,
  livekit_bridge_node 启动时注入 publish 函数, sender 收到 pose 时回调 publish 到 ROS topic
- **mock_mode**: dm_motor_controller_node 加 `mock_mode` 参数,
  没硬件时 `mock_motor:=true` 跳过串口/电机初始化, 控制循环空跑, 用于纯软件 e2e 测试
- **preset 启动时显式应用**: launch 命令行 `preset:=L?` 触发 set_parameter 在 callback
  注册之前, 所以 init 末尾要显式应用一次 startup_preset 才能让 kp/kd 生效
- **200Hz 控制循环跑专用线程**: 跟 rclpy spin 隔离, 通过 lock 共享 vp_pose 缓存
- **livekit_bridge 双线程**: rclpy spin 一个线程, sender_core asyncio + GLib 自己一个 event loop, 互不干扰

### 启动命令速查

```bash
# 全套 (sender + 电机)
ros2 launch teleop_bringup full.launch.py

# 切档
ros2 launch teleop_bringup full.launch.py preset:=L5

# 没硬件 (mock 模式)
ros2 launch teleop_bringup full.launch.py mock_motor:=true

# 只电机
ros2 launch teleop_bringup motor_only.launch.py preset:=L4

# 只 sender (要先 systemctl --user stop lk-s2)
ros2 launch teleop_bringup sender_only.launch.py

# 调试模式 (rqt + rosbag)
ros2 launch teleop_bringup full.launch.py debug:=true
```

### 运行时调参 (不重启)

```bash
ros2 param set /dm_motor_controller kp 30.0
ros2 param set /dm_motor_controller preset L5    # 触发批量更新
ros2 param set /dm_motor_controller sign_yaw -1  # 极性翻转

ros2 service call /dm_motor/disable std_srvs/srv/Trigger
ros2 service call /dm_motor/set_zero std_srvs/srv/Trigger
```

### 测试覆盖

| Stage | 测试 | 结果 |
|-------|------|------|
| 1 骨架 | colcon build 4 包 + ros2 run 4 节点起来 | ✅ |
| 2 livekit_bridge | stop lk-s2 + ros2 run + ros2 topic list 看 /vp/head_pose | ✅ |
| 3 dm_motor (mock) | ros2 run + topic + service + dynamic param 切 preset L5 | ✅ |
| 3 dm_motor (real) | 真实电机 PMAX 通讯 + ramp 到 0 + control loop 200Hz | ✅ |
| 4 launch (mock) | motor_only mock_motor:=true preset:=L3 → kp 自动 20.0 | ✅ |
| 4 launch (real, full) | full preset:=L5 → 2 节点同时起 + 4 topic + kp 30 kd 0.8 | ✅ |
| 5 README | teleop_bringup/README.md 完整覆盖所有调试场景 | ✅ |

### 遇到的坑 (后续 Agent 避雷)

1. **colcon build 报 `ament_package not found`**: shell 环境问题, 在干净的 `bash -c 'source ... && colcon build ...'` 子 shell 里 build 才稳
2. **`read_motor_param` 第一次返回 None**: SDK 异步刷新延迟, 必须重试 5 次每次 0.2s
3. **preset 启动时不生效**: launch 命令行 `preset:=L3` 设的 ROS param 在 callback 注册之前已经发生, 所以 init 末尾要再显式 set_parameters 一次. 加了 `_handling_preset` flag 防止递归
4. **`create_timer` 是周期 timer 不是 one-shot**: 之前用错了, 改成 `_handling_preset` flag
5. **kill 节点后 zombie**: 测试时 SIGINT/timeout 杀进程, 残留 zombie 节点干扰 `ros2 node list`. 用 `pkill -9 -f dm_motor_controller_node` 清理
6. **mock_mode 是必备**: 硬件不在线时 (24V 断了/CAN 松了), 没 mock_mode 节点完全起不来, 整个 e2e 测试做不下去

### 没做的 (留给下一个 Agent)

- **真实 VP 端到端测试**: 我做完了 mock + 真机本地测试, 但还没拿真实 VP head pose 跑全链路 (sender 收 VP → bridge publish → motor_controller 控制电机). 用户需要配合戴 VP 测一次
- **dm_motor_calibration_node**: 互动校准 service 包装, 不是必需 (axis_verify 老脚本可以用)
- **删除老脚本**: 用户说"老脚本完全不动" → scripts/ 一行不删

### Linux 端关键文件路径补充 (ROS2 部分)

| 用途 | 路径 |
|------|------|
| ROS2 工作空间 | `/home/rhz/teleop/ros2_ws/` |
| 4 个 teleop_* package | `ros2_ws/src/teleop_msgs/`, `teleop_livekit_bridge/`, `teleop_dm_motor/`, `teleop_bringup/` |
| dm_motor 配置 | `ros2_ws/src/teleop_dm_motor/config/dm_motor.yaml` (默认参数) |
| dm_motor 档位 | `ros2_ws/src/teleop_dm_motor/config/dm_motor_presets.yaml` (L1-L5) |
| dm_motor 校准 | `ros2_ws/src/teleop_dm_motor/config/dm_motor_calibration.yaml` (示例, 真实数据机器特定) |
| launch 入口 | `ros2_ws/src/teleop_bringup/launch/full.launch.py` |
| README | `ros2_ws/src/teleop_bringup/README.md` (所有调试命令) |
| Vendored 达妙驱动 | `ros2_ws/src/teleop_dm_motor/teleop_dm_motor/lib/DM_CAN.py` |
