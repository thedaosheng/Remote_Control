# 其他力反馈开源项目评估

## 1. Force Dimension SDK (forcedimension.com)

- **设备**: Omega, Delta, Sigma 系列（工业级，非 Touch）
- **状态**: ❌ 不适用（不支持 Touch X）
- **价值**: 力渲染算法思路可参考，但硬件不兼容

## 2. Haply Robotics (haply.co)

- **设备**: Haply Inverse3（开源力反馈设备）
- **GitHub**: https://github.com/HaplyHaptics
- **状态**: ⚠️ 不同硬件平台
- **价值**: 开源硬件+软件全栈参考，Python API 设计优秀

## 3. haptic_teleoperation (ROS)

- **GitHub**: 搜索 "haptic teleoperation ROS"
- **状态**: ⚠️ 大多为 ROS1，需移植
- **价值**: 力反馈遥操作的数据流架构参考

## 4. PyHaptics / python-haptics

- **状态**: ❌ 大多为概念验证项目
- **价值**: Python 力渲染 API 设计参考

## 5. SOFA Framework (sofa-framework.org)

- **GitHub**: https://github.com/sofa-framework/sofa
- **状态**: ✅ 活跃维护
- **价值**: 医疗仿真领域的力渲染标准框架，支持软组织建模
- **注意**: 极其庞大，编译复杂，适合深度医疗仿真场景

## 总结

| 项目 | 硬件兼容 | 活跃度 | 推荐度 | 原因 |
|------|---------|--------|--------|------|
| CHAI3D | ✅ Touch X | 中 | ⭐⭐⭐⭐⭐ | 最直接可用 |
| OpenHaptics | ✅ Touch X | 高（官方）| ⭐⭐⭐⭐⭐ | 官方 SDK，必须 |
| SOFA | ⚠️ 需适配 | 高 | ⭐⭐⭐ | 医疗仿真参考 |
| Haply | ❌ 不兼容 | 中 | ⭐⭐ | API 设计参考 |
| H3DAPI | ✅ Touch X | 低 | ⭐ | 维护停滞 |
