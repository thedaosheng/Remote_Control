#!/usr/bin/env python3
"""
teleop_bridge_node — WebSocket → ROS2 桥接节点

功能：
  1. 启动 WebSocket 服务器监听 ws://0.0.0.0:9090
  2. 接收 JSON: {"positions": [14个float], "mode": "teleop"}
  3. 发布 /forward_command_controller/commands (Float64MultiArray)
  4. 发布 /robot/mode (String)
  5. 50Hz 定时器：没有新数据时持续发送当前关节位置（保持不动）

数据流：
  Vision Pro / 主手臂  →  WebSocket JSON  →  本节点  →  ROS2 话题
"""

import json
import asyncio
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

# websockets 是纯 Python 包，系统 apt 装或 pip 装均可
try:
    import websockets
except ImportError:
    print("ERROR: 缺少 websockets 包，请运行: pip3 install websockets")
    raise


# 23 个位置关节：左臂 8 + 右臂 8 + 升降 1 + 云台 2 + 转向 4
NUM_JOINTS = 23


class TeleopBridgeNode(Node):
    """WebSocket 接收遥操作指令，转发到 ros2_control"""

    def __init__(self):
        super().__init__('teleop_bridge_node')

        # === 发布者 ===
        # /forward_command_controller/commands: 14 维关节位置指令
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_command_controller/commands',
            10,
        )
        # /robot/mode: 系统运行模式字符串
        self.mode_pub = self.create_publisher(String, '/robot/mode', 10)

        # === 内部状态 ===
        # 当前关节位置（初始全零），没有新指令时持续发送这个
        self.current_positions = [0.0] * NUM_JOINTS
        self.current_mode = 'idle'
        # 标记是否有新数据到达（用于日志，不影响发送逻辑）
        self.data_received = False

        # === 50Hz 定时器：持续发送指令 ===
        self.timer = self.create_timer(1.0 / 50.0, self._timer_callback)

        # === 启动 WebSocket 服务器（在独立线程中运行 asyncio 事件循环）===
        self.ws_thread = threading.Thread(target=self._run_ws_server, daemon=True)
        self.ws_thread.start()

        self.get_logger().info('teleop_bridge_node 已启动，WebSocket 监听 ws://0.0.0.0:9090')

    def _timer_callback(self):
        """50Hz 定时回调：发送当前关节位置和模式"""
        # 发布关节指令
        cmd_msg = Float64MultiArray()
        cmd_msg.data = self.current_positions
        self.cmd_pub.publish(cmd_msg)

        # 发布模式
        mode_msg = String()
        mode_msg.data = self.current_mode
        self.mode_pub.publish(mode_msg)

    def _run_ws_server(self):
        """在独立线程中运行 asyncio WebSocket 服务器"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._ws_serve())

    async def _ws_serve(self):
        """启动 WebSocket 服务并永久运行"""
        async with websockets.serve(self._ws_handler, '0.0.0.0', 9090):
            self.get_logger().info('WebSocket 服务器已启动 ws://0.0.0.0:9090')
            await asyncio.Future()  # 永久运行

    async def _ws_handler(self, websocket):
        """
        处理单个 WebSocket 连接

        期望 JSON 格式:
          {"positions": [14个float], "mode": "teleop"}
        """
        client_addr = websocket.remote_address
        self.get_logger().info(f'WebSocket 客户端已连接: {client_addr}')
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    # 解析 positions 字段（必须是 14 个浮点数）
                    positions = data.get('positions', [])
                    if len(positions) == NUM_JOINTS:
                        self.current_positions = [float(p) for p in positions]
                        self.data_received = True
                    else:
                        self.get_logger().warn(
                            f'positions 长度错误: 期望 {NUM_JOINTS}, 收到 {len(positions)}'
                        )
                    # 解析 mode 字段（可选）
                    mode = data.get('mode', '')
                    if mode:
                        self.current_mode = mode
                except json.JSONDecodeError:
                    self.get_logger().warn(f'无效 JSON: {message[:100]}')
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info(f'WebSocket 客户端断开: {client_addr}')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
