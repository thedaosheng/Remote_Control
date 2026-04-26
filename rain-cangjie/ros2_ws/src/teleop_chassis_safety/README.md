# teleop_chassis_safety

仓颉 / Faber 底盘 safety 节点 — watchdog + 风车 (Phone Crazy) 分级状态机 + motor fault aggregation。

与昨天上线的 `teleop-safety.service` (AIRBOT 双臂 14 个 safety ROS 2 services) 接口风格对齐。

## 架构

```
┌───────────────────────────────────────────────────────────────────────┐
│                    chassis_safety_node (this pkg)                     │
│                                                                       │
│   NOMINAL ─(cmd stale ≥200ms)─▶ WARN                                  │
│      ▲                            │                                   │
│      │                            ▼ (≥500ms or motor soft fault)      │
│      │                       SOFT_FENGCHE                             │
│      │                        │     │                                 │
│      │                        │     │ 3× clear+reenable attempts      │
│      │                        ▼     ▼                                 │
│      └── (success) ────── recovery  exhausted / hard fault / ≥1000ms  │
│                                      │                                │
│                                      ▼                                │
│                                HARD_FENGCHE ◀──── /chassis/emergency_stop
│                                      │                                │
│                                      └─── /chassis/reset_safety ───▶ NOMINAL │
│                                                                       │
└───────────────────────────────────────────────────────────────────────┘

  CAN bus (can0, 500kbps)                  serial (/dev/ttyACM1)
  ┌─────────────────┐                      ┌──────────────────┐
  │ ZLAC8015D ×2    │                      │ DM4310 ×4 steer  │
  │ Node 2: RL/RR   │                      │ 0x03/0x04/0x05/  │
  │ Node 3: FL/FR   │                      │ 0x06 (TX IDs)    │
  └────────┬────────┘                      └────────┬─────────┘
           │                                         │
           │ SDO quick_stop / fault_reset            │ /chassis/driver/dm_cmd
           │ (safety-owned write path)               │ (intent → driver node)
           │ heartbeat + EMCY listen                 │ /chassis/driver/dm_reply
           │                                         │ (parsed → diag)
```

## 文件清单

| File | 用途 |
|---|---|
| `chassis_safety_node.py` | 主节点 (828 行) |
| `teleop_chassis_safety/chassis_safety_node.py` | 包内副本 (ament_python entry point 指向此) |
| `config/chassis_safety.yaml` | 参数 (阈值 / 电机 ID / ops_backend URL) |
| `zlac_diag.py` + `zlac_errors.yaml` | ZLAC fault 解析 (sub-agent A 产出) |
| `damiao_diag.py` + `damiao_errors.yaml` | DM fault 解析 (sub-agent B 产出) |
| `package.xml` + `setup.py` | ROS 2 ament_python 包 |
| `systemd/chassis-safety.service` | systemd unit 模板 |
| `test/test_chassis_safety_node.py` | 8 个单测 (纯 Python, 不需要 ROS 2) |

## 服务接口 (std_srvs/Trigger)

| Service | 行为 |
|---|---|
| `/chassis/emergency_stop` | 立即进入 HARD_FENGCHE, CAN quick_stop 所有 ZLAC + 发 DM disable 意图 |
| `/chassis/freeze` | 进入 SOFT_FENGCHE, 启动 3-retry 恢复循环 |
| `/chassis/clear_errors` | ZLAC fault_reset + 发 DM clear_error 意图 |
| `/chassis/re_enable` | CiA402 使能序列 (0x06 → 0x07 → 0x0F) |
| `/chassis/reset_safety` | **只在 HARD_FENGCHE 下生效** — 清硬锁回 NOMINAL |
| `/chassis/status` | 返回 JSON 状态快照 (message 字段) |

## Topics

- Sub: `/cmd_vel` (Twist) — watchdog 输入
- Sub: `/mujoco/swerve_cmd` (Float64MultiArray[8]) — alt watchdog 输入
- Sub: `/chassis/driver/dm_reply` (String-JSON) — 从 steer driver node 喂进 MIT reply (driver 要改)
- Pub: `/chassis/health` (1 Hz, String-JSON) — 聚合状态
- Pub: `/chassis/event` (String-JSON) — 状态转换事件
- Pub: `/chassis/driver/dm_cmd` (String-JSON) — DM 意图 (driver 消费后实际执行)

## 风车分级 (YZL 2026-04-24 设计)

1. **NOMINAL** — 一切正常
2. **WARN** — cmd 超时 200–500 ms, 仅告警不动手
3. **SOFT_FENGCHE** — cmd 超时 500–1000 ms 或电机 soft 故障
   - 立即 velocity=0
   - 自动 clear_errors + re_enable (最多 3 次, 每次失败间隔 0.5 s)
   - 成功 → NOMINAL; 3 次失败 → HARD_FENGCHE
4. **HARD_FENGCHE** — cmd 超时 ≥1000 ms, 电机硬故障, 或 SOFT 恢复失败
   - ZLAC quick_stop + DM disable
   - **sticky** — 必须 `/chassis/reset_safety` 才能离开

## 数据流: safety node 如何拿到 motor 状态

- **ZLAC**: safety 自己开 `can.Bus(channel="can0")` 当 **listener** — 被动收 EMCY (0x80+NodeID) 和 heartbeat (0x700+NodeID)。**写**路径也自己走 (quick_stop / fault_reset SDO), 与 swerve_zlac_driver_node 的 velocity 写并行, SocketCAN 层多路复用, 保证 driver 崩了 safety 仍可达。
- **DM4310**: serial 被 steer driver node 独占; safety 通过 topic `/chassis/driver/dm_reply` 被动收 raw MIT reply (driver 需要发送), 通过 topic `/chassis/driver/dm_cmd` 下发意图 (driver 需要订阅并执行).

> **Driver node 要做两件改动才能完整集成**:
> 1. 发布 `/chassis/driver/dm_reply` (JSON: `{"can_id": 0x13, "frame_hex": "aabbcc..."}`)
> 2. 订阅 `/chassis/driver/dm_cmd` 并消费 `{cmd: enable|disable|clear_error|freeze}`

## 部署到 chassis Pi

1. 把这个目录 scp 到 Pi 的 `~/ros2_ws/src/teleop_chassis_safety/`
2. `cd ~/ros2_ws && colcon build --packages-select teleop_chassis_safety`
3. `source install/setup.bash`
4. 手动跑一次验证: `ros2 run teleop_chassis_safety chassis_safety_node --ros-args --params-file src/teleop_chassis_safety/config/chassis_safety.yaml`
5. systemd: `sudo cp systemd/chassis-safety.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable --now chassis-safety.service`

## Watchdog 阈值调优

defaults 来自 YZL 2026-04-24 对齐:

| Threshold | Default | Tune up 的原因 | Tune down 的原因 |
|---|---|---|---|
| `warn_ms` | 200 | 网络抖动大 (公网 RTT ≥80 ms) | 希望更快告警 |
| `soft_ms` | 500 | 单次 WiFi retransmit 可能吃 300 ms | 控制 loop 对 lag 敏感 |
| `hard_ms` | 1000 | 操作员可能短暂切走 VP 焦点 | 场景有人且需要强保护 |
| `max_soft_retries` | 3 | 允许多点硬件毛刺 | 场景要求故障立即停止 |

## Ops backend 集成 (可选)

设 `ops_backend_url` param 为 WebSocket URL (例如 `ws://192.168.0.181:8080/ws/push`), safety node 会:
- 每 1 Hz 推 `/chassis/health` 快照 → `/ws/state`
- 每次 event 推一条 → `/ws/logs`
Backend 由 sub-agent C 产出, 在 `/Users/edward/Claude/ops-backend/`.

## 已知 TODO

- [ ] Driver node 扩展 `/chassis/driver/dm_{cmd,reply}` topics (不在本包)
- [ ] 集成 sub-agent A 的 `zlac_diag.py` (已落盘, 待 import 验证)
- [ ] 集成 sub-agent B 的 `damiao_diag.py` (等 sub-agent B 完工)
- [ ] LD_LIBRARY 等 ROS 2 环境在 systemd unit 里显式 source (当前是 bash -lc)
- [ ] 真机 1-wheel 单元测试: 拔 can0 线验证 HARD 自动激活
