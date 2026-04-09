# Touch 力反馈笔 — 完整使用指南

## 这是什么

3D Systems Touch (Phantom Omni) 力反馈设备的驱动、力渲染 Demo、以及 Agent 集成文档。
包含 8 种力渲染效果的真机验证代码，可直接编译运行。

## 设备信息

| 参数 | 值 |
|------|-----|
| 型号 | 3D Systems Touch (Phantom Omni USB) |
| USB VID:PID | 2988:0302 |
| 串口 | /dev/ttyACMx (x 取决于 USB 插入顺序) |
| 自由度 | 6-DOF 输入 (位置+姿态), 3-DOF 力输出 (Fx/Fy/Fz) |
| 最大力 | 3.3 N |
| 连续力 | 0.88 N |
| 伺服频率 | ~1000 Hz |
| 坐标系 | X=右 ±100mm, Y=上 0~200mm, Z=朝用户 ±100mm, 原点=底座中心 |

## 快速开始 (3 步)

```bash
# 1. 部署 (首次 or 重启后)
bash TouchUsage/scripts/deploy.sh

# 2. 交互式体验 8 种力渲染效果
bash TouchUsage/scripts/run_interactive.sh

# 3. 后台运行 (Agent 自动切模式)
bash TouchUsage/scripts/run_background.sh
echo 3 > /tmp/force_mode   # 切换到效果 3
cat /tmp/force_status       # 查看实时状态
echo q > /tmp/force_mode   # 停止
```

## 8 种力渲染效果

| 键 | 效果 | 体验方式 |
|----|------|---------|
| 0 | OFF 关闭力输出 | 自由移动，无任何力 |
| 1 | WALL 刚度墙 | 往下压碰到硬面 (Y=80mm)，越压越硬 |
| 2 | SPRING 弹簧回中 | 笔被拉向原点，拉远了松手会弹回来 |
| 3 | VISCOUS 粘滞力场 | 像在蜂蜜里移动，快速甩动阻力大 |
| 4 | FRICTION 表面摩擦 | 先下压碰面再水平滑动，感受拖拽阻力 |
| 5 | MAGNET 磁吸 | 靠近原点被吸住，需要用力拉开 |
| 6 | GRAVITY 重力井 | 温和引力把你拉向中心 |
| 7 | TEXTURE 振动纹理 | 下压后左右滑动，搓衣板凹凸感 |
| 8 | CHANNEL 虚拟引导槽 | 只能左右走，偏离轨道有恢复力 |

## 文件结构

```
TouchUsage/
├── README.md              ← 本文件 (Agent 读这个就知道怎么用)
├── AGENT_GUIDE.md         ← Agent 专属指南 (编程接口/已知坑/集成方法)
├── src/
│   ├── force_interactive.c  ← 交互式 demo (键盘切换效果)
│   └── force_background.c   ← 后台 demo (文件控制, Agent 用)
└── scripts/
    ├── deploy.sh            ← 一键部署 (补丁库+编译)
    ├── run_interactive.sh   ← 启动交互式 demo
    └── run_background.sh    ← 启动后台 demo
```

## 已知限制

- Touch 只有 **3-DOF 力输出** (Fx/Fy/Fz)，无法输出力矩
- 这是**动觉反馈** (kinesthetic)，不是触觉反馈 (cutaneous) — 无法模拟皮肤纹理/温度
- 补丁库在 `/tmp` 下，**重启后丢失**，需要重新运行 `deploy.sh`
- 程序必须用 SIGINT (Ctrl+C) 或 `echo q > /tmp/force_mode` 退出，**强杀 (kill -9) 会锁死设备**，需拔插 USB
