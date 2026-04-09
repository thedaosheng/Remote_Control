# OpenHaptics SDK 示例评估

## 项目信息
- **来源**: 3D Systems 官方 SDK
- **社区示例**: GitHub 搜索 "OpenHaptics example"
- **SDK 下载**: 3D Systems 开发者网站（需注册）

## 状态: ⚠️ 需要物理设备 + SDK 安装

## 概述
OpenHaptics 是 3D Systems 官方提供的 Touch 系列设备开发 SDK。
包含两层 API：
- **HDAPI (Haptic Device API)**: 底层设备访问，直接控制力输出
- **HLAPI (Haptic Library API)**: 高层力渲染，自动碰撞检测

## 关键 GitHub 社区项目

### 1. SensAble OpenHaptics Examples
- 原始 SDK 附带的示例程序
- C++ 代码，展示基础力渲染
- 包括: HelloHapticDevice, FrictionlessPlane, PointManipulation 等

### 2. haptic-teleoperation-ros
- ROS + OpenHaptics 遥操作示例
- 可直接参考 ROS2 集成方案

### 3. OpenHaptics-Unity-Plugin
- Unity 集成插件
- C# 封装，展示力渲染 API 用法

## HDAPI 核心力渲染回调

```cpp
// 1kHz 伺服循环回调 — 这是力渲染的核心
HDCallbackCode HDCALLBACK forceCallback(void *data) {
    hdBeginFrame(hdGetCurrentDevice());

    // 读取设备位置
    hduVector3Dd position;
    hdGetDoublev(HD_CURRENT_POSITION, position);

    // 计算并设置力
    hduVector3Dd force(0, 0, 0);

    // 示例：平面刚度墙 (z < 0 时产生向上的力)
    if (position[2] < 0) {
        force[2] = -position[2] * stiffness;  // k * penetration
    }

    hdSetDoublev(HD_CURRENT_FORCE, force);
    hdEndFrame(hdGetCurrentDevice());
    return HD_CALLBACK_CONTINUE;
}
```

## 对本项目的价值
- Touch X 设备驱动的标准 API
- 力渲染回调模式是行业标准
- 未来真机集成时的必需 SDK
