#!/usr/bin/env python3
"""
mock_data_publisher — 模拟数据发布节点（最小案例测试用）

功能：
  1. 通过 WebSocket 发送正弦波关节角（模拟 VP/主手臂输入）
     左臂 joint1 做 0~0.5rad 正弦摆动，其余关节保持 0
  2. 同时发布所有感知 Topic 的假数据：
     /camera/stereo/left/image_raw — 随机噪声图像 640x480
     /proprioception/joints       — 和指令一致的关节状态
     /proprioception/ee_pose_left — 固定位置的 PoseStamped
     /proprioception/gripper_left — aperture=0.5 的 GripperState

数据流：
  本节点 → WebSocket → teleop_bridge_node → ros2_control → RViz2
  本节点 → 感知 Topics → 后续策略网络 / 数据采集
"""

import math
import time
import json
import asyncio
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from airbot_msgs.msg import GripperState

# websockets 用于向 teleop_bridge_node 发送模拟指令
try:
    import websockets
except ImportError:
    print("ERROR: 缺少 websockets 包，请运行: pip3 install websockets")
    raise


# 23 个位置关节：左臂 8 + 右臂 8 + 升降 1 + 云台 2 + 转向 4
# 轮子驱动（4个）由 joint_state_broadcaster 自动广播，不走 forward_command_controller
NUM_JOINTS = 23
# WebSocket 地址（teleop_bridge_node 监听的端口）
WS_URI = 'ws://localhost:9090'


class MockDataPublisher(Node):
    """模拟数据发布器：正弦波关节角 + 假感知数据"""

    def __init__(self):
        super().__init__('mock_data_publisher')

        # === 感知数据发布者 ===
        # 立体相机左目图像
        self.image_pub = self.create_publisher(
            Image, '/camera/stereo/left/image_raw', 10
        )
        # 本体感知：关节状态（和发送的指令一致）
        self.joints_pub = self.create_publisher(
            JointState, '/proprioception/joints', 10
        )
        # 本体感知：左臂末端位姿
        self.ee_pose_pub = self.create_publisher(
            PoseStamped, '/proprioception/ee_pose_left', 10
        )
        # 本体感知：左手夹爪状态
        self.gripper_pub = self.create_publisher(
            GripperState, '/proprioception/gripper_left', 10
        )

        # === 内部状态 ===
        self.start_time = time.time()
        self.current_positions = [0.0] * NUM_JOINTS

        # 关节名：左臂8 + 右臂8 + 升降1 + 云台2 + 转向4 = 23
        self.joint_names = [
            f'{side}_joint{i}'
            for side in ['left', 'right']
            for i in range(1, 9)
        ] + [
            'lift_joint',
            'rentou_joint1', 'rentou_joint2',
            'fl_steer_joint', 'fr_steer_joint',
            'rl_steer_joint', 'rr_steer_joint',
        ]

        # === 50Hz 定时器：发布感知数据 + WebSocket 发送指令 ===
        self.timer = self.create_timer(1.0 / 50.0, self._timer_callback)

        # === WebSocket 客户端（独立线程异步发送）===
        self.ws_connected = False
        self.ws_thread = threading.Thread(target=self._run_ws_client, daemon=True)
        self.ws_thread.start()

        self.get_logger().info('mock_data_publisher 已启动')

    def _timer_callback(self):
        """50Hz 回调：生成正弦波 + 发布所有感知数据"""
        now = time.time()
        t = now - self.start_time

        # === 生成正弦波关节位置 ===
        # 顺序: left_joint1~8(0~7), right_joint1~8(8~15),
        #        lift_joint(16), rentou_j1(17), rentou_j2(18)
        self.current_positions = [0.0] * NUM_JOINTS

        # --- 左臂 ---
        # joint1 (index=0): 0~0.5 rad 正弦摆动
        self.current_positions[0] = 0.25 + 0.25 * math.sin(2.0 * math.pi * 0.5 * t)
        # joint2 (index=1): 小幅摆动
        self.current_positions[1] = -0.2 * math.sin(2.0 * math.pi * 0.3 * t)
        # joint3 (index=2): 摆动
        self.current_positions[2] = 0.3 * math.sin(2.0 * math.pi * 0.3 * t)
        # 夹爪同步：joint7(6) = joint8(7)
        grip_val = 0.018 + 0.018 * math.sin(2.0 * math.pi * 0.4 * t)
        self.current_positions[6] = grip_val
        self.current_positions[7] = grip_val

        # --- 右臂 ---
        # joint1 (index=8): 反相摆动
        self.current_positions[8] = -0.25 - 0.25 * math.sin(2.0 * math.pi * 0.5 * t)
        # 夹爪同步
        self.current_positions[14] = grip_val
        self.current_positions[15] = grip_val

        # --- 升降机构 ---
        # lift_joint (index=16): 0~0.3m 慢速升降，周期 6 秒
        self.current_positions[16] = 0.15 + 0.15 * math.sin(2.0 * math.pi * (1.0/6.0) * t)

        # --- 云台 ---
        # yaw (index=17): 左右摆动 ±0.5 rad
        self.current_positions[17] = 0.5 * math.sin(2.0 * math.pi * 0.2 * t)
        # pitch (index=18): 俯仰 ±0.3 rad
        self.current_positions[18] = 0.3 * math.sin(2.0 * math.pi * 0.15 * t)

        # --- 舵轮转向 ---
        # 4 个转向关节 (index 19~22) 同步旋转，演示全向转向
        # 周期 5 秒，全部轮组同方向转动 → 模拟侧移 / 旋转预备
        steer_angle = 0.8 * math.sin(2.0 * math.pi * 0.2 * t)
        self.current_positions[19] = steer_angle   # fl_steer
        self.current_positions[20] = steer_angle   # fr_steer
        self.current_positions[21] = steer_angle   # rl_steer
        self.current_positions[22] = steer_angle   # rr_steer

        # === 发布 /proprioception/joints ===
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.header.frame_id = 'world'
        js_msg.name = self.joint_names
        js_msg.position = self.current_positions
        js_msg.velocity = [0.0] * NUM_JOINTS
        js_msg.effort = [0.0] * NUM_JOINTS
        self.joints_pub.publish(js_msg)

        # === 发布 /camera/stereo/left/image_raw（随机噪声 640x480 RGB）===
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_left'
        img_msg.height = 480
        img_msg.width = 640
        img_msg.encoding = 'rgb8'
        img_msg.is_bigendian = False
        img_msg.step = 640 * 3  # 每行字节数
        # 随机噪声图像数据
        img_msg.data = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8).tobytes()
        self.image_pub.publish(img_msg)

        # === 发布 /proprioception/ee_pose_left（固定位置）===
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        # 固定位置：大致在左臂 TCP 的初始位置
        pose_msg.pose.position.x = 0.3
        pose_msg.pose.position.y = 0.2
        pose_msg.pose.position.z = 0.8
        pose_msg.pose.orientation.w = 1.0  # 单位四元数（无旋转）
        self.ee_pose_pub.publish(pose_msg)

        # === 发布 /proprioception/gripper_left（固定开合度 0.5）===
        grip_msg = GripperState()
        grip_msg.left_aperture = 0.5
        grip_msg.right_aperture = 0.5
        grip_msg.left_force = 0.0
        grip_msg.right_force = 0.0
        self.gripper_pub.publish(grip_msg)

    def _run_ws_client(self):
        """独立线程运行 WebSocket 客户端，持续向 teleop_bridge_node 发送指令"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._ws_send_loop())

    async def _ws_send_loop(self):
        """异步循环：连接 WebSocket 并以 50Hz 发送关节指令"""
        while True:
            try:
                async with websockets.connect(WS_URI) as ws:
                    self.ws_connected = True
                    self.get_logger().info(f'已连接 WebSocket: {WS_URI}')
                    while True:
                        # 发送当前关节位置到 teleop_bridge_node
                        payload = json.dumps({
                            'positions': self.current_positions,
                            'mode': 'teleop',
                        })
                        await ws.send(payload)
                        await asyncio.sleep(1.0 / 50.0)
            except Exception as e:
                self.ws_connected = False
                self.get_logger().warn(f'WebSocket 连接失败: {e}，2秒后重试...')
                await asyncio.sleep(2.0)


def main(args=None):
    rclpy.init(args=args)
    node = MockDataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
