#!/bin/bash
# 运行所有 MuJoCo demo 脚本并生成截图
# 使用方法: bash touch/20260409-cc-run_all.sh

set -e
export MPLBACKEND=Agg
cd "$(dirname "$0")/mujoco_sim"

echo "=== 1/4: 生成模拟轨迹数据 ==="
python3 20260409-cc-mock_touch_data.py

echo "=== 2/4: 基础几何体碰撞力测试 ==="
python3 20260409-cc-touch_probe_basic.py

echo "=== 3/4: 表面纹理力渲染测试 ==="
python3 20260409-cc-touch_probe_textures.py

echo "=== 4/4: 力渲染效果可视化 ==="
python3 20260409-cc-force_visualizer.py

echo "=== 完成! 截图列表: ==="
ls -la ../screenshots/*.png 2>/dev/null | wc -l
echo "张截图已生成"
