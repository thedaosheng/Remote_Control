#!/usr/bin/env python3
"""
teleop_bridge_node — WebSocket 接收 VP 指令 -> ROS2 Topic

数据流:
  Apple Vision Pro
    -> WebSocket JSON: {"left":[j1..j6], "right":[j1..j6]}
    -> /joint_commands              (Float64MultiArray, 给 motor_control_node)
    -> /forward_command_controller/commands  (直接驱动 mock hardware)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import asyncio, websockets, json, threading

ZERO_POSITION = [0.0] * 12


class TeleopBridgeNode(Node):
    def __init__(self):
        super().__init__('teleop_bridge_node')

        # 参数
        self.declare_parameter('ws_host', '0.0.0.0')
        self.declare_parameter('ws_port', 8080)
        self.declare_parameter('publish_rate_hz', 50.0)
        ws_host = self.get_parameter('ws_host').value
        ws_port = self.get_parameter('ws_port').value

        # 发布器
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/joint_commands', 10)
        self.hw_pub = self.create_publisher(
            Float64MultiArray, '/forward_command_controller/commands', 10)

        # 共享缓存 + 锁
        self._cmd = ZERO_POSITION.copy()
        self._lock = threading.Lock()

        # 定时发布 50Hz
        rate = self.get_parameter('publish_rate_hz').value
        self.create_timer(1.0 / rate, self._publish_cmd)

        # WebSocket 服务器在独立线程运行
        threading.Thread(
            target=lambda: asyncio.run(self._ws_serve(ws_host, ws_port)),
            daemon=True).start()

        self.get_logger().info(
            f'TeleopBridgeNode 启动，ws://{ws_host}:{ws_port}')

    async def _ws_serve(self, host, port):
        async with websockets.serve(self._ws_handler, host, port):
            await asyncio.Future()

    async def _ws_handler(self, websocket):
        """
        接收 JSON: {"left":[6个弧度], "right":[6个弧度]}
        写入共享缓存，由定时器发布
        """
        self.get_logger().info(f'VP 连接: {websocket.remote_address}')
        try:
            async for msg in websocket:
                try:
                    d = json.loads(msg)
                    cmd = list(d.get('left', [0.0]*6)) + list(d.get('right', [0.0]*6))
                    if len(cmd) == 12:
                        with self._lock:
                            self._cmd = cmd
                    else:
                        self.get_logger().warn(f'关节数量错误: {len(cmd)}')
                except json.JSONDecodeError as e:
                    self.get_logger().warn(f'JSON 解析失败: {e}')
        except Exception as e:
            self.get_logger().info(f'VP 断开: {e}')

    def _publish_cmd(self):
        with self._lock:
            cmd = self._cmd.copy()
        msg = Float64MultiArray(data=cmd)
        self.cmd_pub.publish(msg)
        self.hw_pub.publish(msg)


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
