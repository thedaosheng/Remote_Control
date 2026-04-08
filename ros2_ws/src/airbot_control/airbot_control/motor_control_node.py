#!/usr/bin/env python3
"""
motor_control_node — 订阅关节指令 -> mock 执行 / 真实 CAN 驱动

数据流:
  /joint_commands (Float64MultiArray, 12个弧度)
    -> motor_control_node
    -> [仿真] 打印日志 + 回显到 /joint_states_echo
    -> [真实] CAN 帧 -> DM-J4310 电机 (后续实现)

关节顺序:
  left_joint1~6,  right_joint1~6
  与 controllers.yaml 及 teleop_bridge_node 保持一致
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math

JOINT_NAMES = [
    'left_joint1',  'left_joint2',  'left_joint3',
    'left_joint4',  'left_joint5',  'left_joint6',
    'right_joint1', 'right_joint2', 'right_joint3',
    'right_joint4', 'right_joint5', 'right_joint6',
]


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # 参数
        self.declare_parameter('mode', 'mock')       # mock | can
        self.declare_parameter('log_interval', 1.0)  # 每隔多少秒打印一次日志
        self._mode = self.get_parameter('mode').value
        self._log_interval = self.get_parameter('log_interval').value

        # 订阅 /joint_commands
        self.sub = self.create_subscription(
            Float64MultiArray,
            '/joint_commands',
            self._cmd_callback,
            10)

        # 发布回显（可选，方便用 ros2 topic echo 验证）
        self.echo_pub = self.create_publisher(
            JointState, '/joint_states_echo', 10)

        # 日志节流计数器
        self._recv_count = 0
        self._last_log_time = self.get_clock().now()

        self.get_logger().info(
            f'MotorControlNode 启动，模式: {self._mode}，'
            f'订阅 /joint_commands')

    def _cmd_callback(self, msg: Float64MultiArray):
        """
        接收关节指令回调

        mock 模式: 打印指令（节流，每 log_interval 秒一次）
        can  模式: 调用 CAN 发送函数（后续实现）
        """
        self._recv_count += 1
        positions = list(msg.data)

        if len(positions) != 12:
            self.get_logger().warn(f'关节数量错误: {len(positions)}，期望 12')
            return

        # 节流日志：避免 50Hz 刷屏
        now = self.get_clock().now()
        elapsed = (now - self._last_log_time).nanoseconds * 1e-9
        if elapsed >= self._log_interval:
            left  = [f'{math.degrees(p):.1f}°' for p in positions[:6]]
            right = [f'{math.degrees(p):.1f}°' for p in positions[6:]]
            self.get_logger().info(
                f'[#{self._recv_count}] '
                f'左臂: {left} | 右臂: {right}')
            self._last_log_time = now

        # mock 模式：直接回显为 JointState
        if self._mode == 'mock':
            self._publish_echo(positions)
        elif self._mode == 'can':
            self._send_can(positions)

    def _publish_echo(self, positions):
        """将收到的指令回显为 JointState，方便验证话题通路"""
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES
        js.position = positions
        js.velocity = [0.0] * 12
        js.effort   = [0.0] * 12
        self.echo_pub.publish(js)

    def _send_can(self, positions):
        """
        [预留] 真实 CAN 驱动接口

        后续实现：
          for i, pos in enumerate(positions):
              motor_id = JOINT_TO_MOTOR_ID[i]
              can_frame = dm_j4310_position_cmd(motor_id, pos)
              self.can_bus.send(can_frame)
        """
        self.get_logger().warn('CAN 模式尚未实现，忽略指令')


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
