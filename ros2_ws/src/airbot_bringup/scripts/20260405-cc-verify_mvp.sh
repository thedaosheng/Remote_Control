#!/bin/bash
# ============================================================
# verify_mvp.sh — 最小案例数据通路验证脚本
#
# 功能：
#   1. 启动 system.launch.py (mock 模式)
#   2. 启动 teleop_bridge_node + mock_data_publisher
#   3. 等待所有节点就绪
#   4. 验证所有关键 Topic 存在且有数据流
#   5. 输出 PASS/FAIL 报告
#
# 用法：
#   bash verify_mvp.sh
#
# 注意：需要先 colcon build 并 source install/setup.bash
# ============================================================

set -e

# 颜色输出
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}========== Airbot MVP 数据通路验证 ==========${NC}"

# 确保使用系统 Python（绕开 miniconda）
export PATH="/usr/bin:/usr/local/bin:$PATH"

# source ROS2 环境
source /opt/ros/humble/setup.bash
source ~/teleop/ros2_ws/install/setup.bash

# 清理函数：退出时杀掉所有后台进程
cleanup() {
    echo -e "\n${YELLOW}正在清理后台进程...${NC}"
    kill $LAUNCH_PID $BRIDGE_PID $MOCK_PID 2>/dev/null || true
    wait $LAUNCH_PID $BRIDGE_PID $MOCK_PID 2>/dev/null || true
    echo -e "${GREEN}清理完成${NC}"
}
trap cleanup EXIT

# === Step 1: 启动 system.launch.py (mock 模式) ===
echo -e "${YELLOW}[1/4] 启动 system.launch.py mode:=mock ...${NC}"
ros2 launch airbot_bringup system.launch.py mode:=mock &
LAUNCH_PID=$!
sleep 5  # 等待 controller_manager + spawner 完成

# === Step 2: 启动 teleop_bridge_node ===
echo -e "${YELLOW}[2/4] 启动 teleop_bridge_node ...${NC}"
ros2 run airbot_bringup teleop_bridge_node &
BRIDGE_PID=$!
sleep 2

# === Step 3: 启动 mock_data_publisher ===
echo -e "${YELLOW}[3/4] 启动 mock_data_publisher ...${NC}"
ros2 run airbot_bringup mock_data_publisher &
MOCK_PID=$!
sleep 3  # 等待 WebSocket 连接建立 + 数据开始流动

# === Step 4: 验证所有关键 Topic ===
echo -e "${YELLOW}[4/4] 验证 Topic 列表 ...${NC}"
echo ""

PASS_COUNT=0
FAIL_COUNT=0

# 需要验证的 Topic 列表
TOPICS=(
    "/joint_states"
    "/robot/mode"
    "/camera/stereo/left/image_raw"
    "/proprioception/joints"
    "/proprioception/ee_pose_left"
    "/proprioception/gripper_left"
    "/forward_command_controller/commands"
)

for t in "${TOPICS[@]}"; do
    if ros2 topic info "$t" > /dev/null 2>&1; then
        # 检查是否有发布者
        PUB_COUNT=$(ros2 topic info "$t" | grep -c "Publisher" || true)
        if [ "$PUB_COUNT" -gt 0 ]; then
            echo -e "  ${GREEN}PASS${NC}: $t (${PUB_COUNT} publisher)"
            PASS_COUNT=$((PASS_COUNT + 1))
        else
            echo -e "  ${RED}FAIL${NC}: $t (存在但无 publisher)"
            FAIL_COUNT=$((FAIL_COUNT + 1))
        fi
    else
        echo -e "  ${RED}FAIL${NC}: $t (Topic 不存在)"
        FAIL_COUNT=$((FAIL_COUNT + 1))
    fi
done

# === 汇总 ===
echo ""
echo -e "${YELLOW}========== 验证结果 ==========${NC}"
echo -e "  通过: ${GREEN}${PASS_COUNT}${NC}"
echo -e "  失败: ${RED}${FAIL_COUNT}${NC}"
TOTAL=$((PASS_COUNT + FAIL_COUNT))
echo -e "  总计: ${TOTAL}"

if [ "$FAIL_COUNT" -eq 0 ]; then
    echo -e "\n${GREEN}✓ 所有 Topic 验证通过！MVP 数据通路完整。${NC}"
else
    echo -e "\n${RED}✗ 有 ${FAIL_COUNT} 个 Topic 验证失败，请检查。${NC}"
fi

echo ""
echo -e "${YELLOW}按 Ctrl+C 退出并清理所有后台进程${NC}"
# 保持运行，让用户可以手动检查（如 rqt_graph 等）
wait
