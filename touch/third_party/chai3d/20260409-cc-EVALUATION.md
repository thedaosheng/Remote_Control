# CHAI3D 评估报告

## 项目信息
- **GitHub**: https://github.com/chai3d/chai3d
- **Stars**: ~800+
- **最后更新**: 2023 (活跃维护)
- **许可证**: BSD 3-Clause

## 状态: ⚠️ 部分可用（需要物理设备）

## 概述
CHAI3D (Computer Haptics and Active Interface) 是最主流的开源力渲染框架，
由斯坦福大学 Conti 教授团队开发。支持多种力反馈设备。

## 核心功能
- **基础力渲染**: 刚度墙、摩擦力、粘滞力
- **代理点算法 (God-Object / Proxy)**: Zilles & Salisbury 方法
- **网格碰撞**: 支持复杂三角网格的接触检测
- **力效果器 (cGenericEffect)**: 可编程力渲染管道
- **多设备支持**: Touch/Touch X, Phantom Premium, Falcon, Omega 等
- **OpenGL 可视化**: 内置 3D 渲染引擎

## 内置力效果
1. `cEffectMagnet` — 磁吸效果
2. `cEffectStickSlip` — 粘滑效果
3. `cEffectSurface` — 表面接触（刚度+摩擦）
4. `cEffectVibration` — 振动效果
5. `cEffectViscosity` — 粘滞效果

## 编译依赖
- CMake 3.10+
- C++14 编译器
- OpenGL + GLEW + GLFW
- OpenHaptics SDK (可选，用于 Touch 设备)
- Eigen3 (矩阵运算)

## 安装命令 (参考)
```bash
git clone https://github.com/chai3d/chai3d.git
cd chai3d
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
# 示例在 bin/ 目录下
```

## 对本项目的价值
- 力渲染算法参考实现（可移植到 Python/MuJoCo）
- God-Object 代理点算法的工业级实现
- 设备抽象层可作为 Touch X 驱动参考
