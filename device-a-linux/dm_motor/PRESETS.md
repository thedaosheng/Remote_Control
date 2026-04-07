# 达妙电机 VP 控制 — 跟手感梯度档位

> 5 个档位,从最稳到最跟手。直接拷命令跑。
> 找到你喜欢的档位之后告诉我,我把它写成默认。

## 五个梯度

| 档位 | 跟手感 | 震动风险 | 适用场景 |
|------|--------|----------|----------|
| **L1 安全** | ★ | 无 | 第一次跑 / 演示给别人看 / 不熟悉电机时 |
| **L2 温和** | ★★ | 极低 | 日常调试 / 不追求跟手 |
| **L3 默认** | ★★★ | 低 | **推荐起步** |
| **L4 跟手** | ★★★★ | 中 | 想要明显跟手感 / 你现在的需求 |
| **L5 激进** | ★★★★★ | 高 | 极限测试 / 要求 1:1 同步 |

---

## L1 — 安全档

```bash
/usr/bin/python3 /home/rhz/teleop/scripts/20260407-cc-dm_motor_vp_control.py \
  --kp 8 --kd 1.2 --rate-limit 100 \
  --feedforward off
```
- 低刚度 + 100°/s 限速 + 无速度前馈
- 电机"软"且慢,绝对不会过冲或震动
- 跟踪误差 ~5-7°

## L2 — 温和档

```bash
/usr/bin/python3 /home/rhz/teleop/scripts/20260407-cc-dm_motor_vp_control.py \
  --kp 15 --kd 1.2 --rate-limit 200 \
  --feedforward on --ff-gain 0.5 --ff-ema 0.3
```
- 中等刚度 + 半幅前馈 + 温和 EMA
- 跟踪误差 ~3-4°,有微弱跟手感

## L3 — 默认档 ⭐ (推荐起步)

```bash
/usr/bin/python3 /home/rhz/teleop/scripts/20260407-cc-dm_motor_vp_control.py \
  --kp 20 --kd 1.0 --rate-limit 300 \
  --feedforward on --ff-gain 1.0 --ff-ema 0.5
```
- 跟踪误差 ~1-2°,跟手感明显
- 这一组是 baseline,所有命令行参数都是默认值
- 等价于 `--kp 20 --kd 1.0 --rate-limit 300` (其他用默认)

## L4 — 跟手档 (你现在想要的)

```bash
/usr/bin/python3 /home/rhz/teleop/scripts/20260407-cc-dm_motor_vp_control.py \
  --kp 25 --kd 1.0 --rate-limit 500 \
  --feedforward on --ff-gain 1.2 --ff-ema 0.7
```
- KP 提到 25,跟踪误差 <1°
- 速度前馈增益 1.2 倍,加速更猛
- EMA α=0.7,组延迟从 33ms 降到 14ms
- rate-limit 500°/s,几乎不限速
- 跟手感强烈,头快速转电机也跟得上

## L5 — 激进档 (有震动风险)

```bash
/usr/bin/python3 /home/rhz/teleop/scripts/20260407-cc-dm_motor_vp_control.py \
  --kp 30 --kd 0.8 --rate-limit 600 \
  --feedforward on --ff-gain 1.5 --ff-ema 0.8
```
- KP=30 (Notion 文档推荐工作上限)
- ff-gain 1.5 倍超调
- EMA α=0.8 几乎无平滑,直接传速度
- **如果电机发出"嗡"声或抖动,马上 Ctrl+C 退**

---

## 各旋钮的物理含义 (调参手册)

| 参数 | 默认 | 范围 | 调大的效果 | 调大的副作用 |
|------|------|------|-----------|--------------|
| `--kp` | 20 | 8~50 | 跟踪误差减小 | 太大震动/啸叫 |
| `--kd` | 1.0 | 0.3~3.0 | 抑制振荡更稳 | 太大粘滞、跟不动 |
| `--rate-limit` | 300 | 50~600 | 允许更快目标变化 | (软件层面,无危险) |
| `--ff-gain` | 1.0 | 0~2.0 | 前馈速度放大 | 容易过冲,头停了电机还在动 |
| `--ff-ema` | 0.5 | 0.1~1.0 | 速度响应更快 | 单帧噪声放大,容易抖 |

## 调参顺序建议

1. **先动 KP**(影响最大): 8 → 15 → 20 → 25 → 30
2. **再动 ff-ema**(降延迟): 0.3 → 0.5 → 0.7
3. **最后动 ff-gain**(超调): 1.0 → 1.2 → 1.5
4. **rate-limit 通常不用动**: 300 一般够,只有 L4/L5 才提

如果出现震动/啸叫:
- KD 加 0.5 → 减小振荡
- ff-gain 减半 → 减小过冲
- ff-ema 减 0.2 → 减小高频噪声

## 故障档位

如果电机突然乱跳或者啸叫:
```bash
# 紧急切回 L1 安全档
/usr/bin/python3 /home/rhz/teleop/scripts/20260407-cc-dm_motor_vp_control.py \
  --kp 8 --kd 1.2 --rate-limit 100 --feedforward off
```

---

## 系统瓶颈分析 (为什么再调也快不到哪去)

数据链路的延迟分解:

| 环节 | 延迟 | 能优化吗 |
|------|------|----------|
| VP 头部追踪 | ~5 ms | 不能 (visionOS 内部) |
| LiveKitManager `poseTimer` 30 Hz 采样 | **平均 17 ms,最坏 33 ms** | ★ **可以提到 60/90 Hz** |
| LiveKit Data Channel 网络传输 | ~30-50 ms | 不能 (要走云端中继) |
| sender `_on_data_received` + UDP 转发 | <1 ms | 已最优 |
| 本脚本 EMA 平滑组延迟 (α=0.5) | ~33 ms | 可调 (α 越大越短) |
| 控制循环 200 Hz dt | ~5 ms | 已最优 |
| 电机 CAN 通讯 + 物理响应 | ~5-10 ms | 不能 |
| **总计** | **~95-150 ms** | |

**最大的可优化点是 VP 端 pose 发送频率 (`LiveKitConfig.poseFrequency`)**:

当前: `static let poseFrequency: Double = 30.0`

如果改成 60.0 → 平均延迟省 8ms (33→17→8)
如果改成 90.0 → 平均延迟省 11ms (再省 ~3ms)

需要改 `controller/visionos-native/RemoteControl/RemoteControl/LiveKit/LiveKitManager.swift`,
然后 SSH 到 Mac xcodebuild 重新打包。这是个一次性的事,改了一直生效。

**结论**: 如果你已经在 L4 跑顺了还想更跟,告诉我,我把 VP pose 提到 90 Hz。
