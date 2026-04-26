#!/usr/bin/env python3
"""
==============================================================
  livekit_bridge_node — LiveKit ↔ ROS2 桥接节点
==============================================================

职责:
  1. 启动 GStreamer pipeline 从 ZED Mini 采集 → 推视频到 LiveKit (sender_core)
  2. 监听 LiveKit Data Channel 收 VP head pose
  3. publish 到 ROS topic:
       /vp/head_pose       teleop_msgs/HeadPose
       /vp/head_pose_rpy   geometry_msgs/Vector3Stamped  (rqt_plot 友好的简化版)

线程模型:
  - 主线程: asyncio event loop (LiveKit Python SDK 要求)
  - GLib 线程: GStreamer 信号回调 (sender_core 自动启动)
  - rclpy spin 线程: rclpy.spin() 在独立线程跑, publisher.publish 是线程安全的
  - LiveKit data_received 回调: 在 asyncio loop 里被调,
    通过 sender_core.ROS_POSE_CALLBACK 调到本节点的 publish_pose,
    publish_pose 直接调 publisher (不需要回到主线程)

启动:
  ros2 run teleop_livekit_bridge livekit_bridge_node

注意:
  - 视频/LiveKit 配置目前还在 sender_core.py 顶部 const 里, Stage 4 launch
    时会通过 ROS parameter override (用环境变量传给 sender_core 模块)
  - sender_core 内部仍然 UDP 转发到 :9000 (向后兼容老的 dm_motor_vp_control.py),
    Stage 3 的 dm_motor_controller_node 会通过 ROS topic 接收, 不再依赖 UDP
"""

import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped

from teleop_msgs.msg import HeadPose
from teleop_livekit_bridge import sender_core


class LiveKitBridgeNode(Node):
    """LiveKit Python SDK ↔ ROS2 桥接节点."""

    def __init__(self):
        super().__init__('livekit_bridge')

        # 声明 ROS 参数 (留给 Stage 4 launch 配置, 现在用默认值)
        self.declare_parameter('pose_topic', '/vp/head_pose')
        self.declare_parameter('pose_rpy_topic', '/vp/head_pose_rpy')

        pose_topic = self.get_parameter('pose_topic').value
        rpy_topic  = self.get_parameter('pose_rpy_topic').value

        # 创建 publisher
        # 用默认 QoS (depth=10), 30Hz pose 数据足够
        self.head_pose_pub = self.create_publisher(HeadPose, pose_topic, 10)
        self.head_pose_rpy_pub = self.create_publisher(Vector3Stamped, rpy_topic, 10)

        # 把 publish 方法注入到 sender_core 的全局 hook
        # sender_core._on_data_received 里检测到非 None 就会调用
        sender_core.ROS_POSE_CALLBACK = self._on_pose_received

        self.publish_count = 0
        self.get_logger().info(
            f'[livekit_bridge] node started. '
            f'Publishing to {pose_topic} + {rpy_topic}'
        )
        self.get_logger().info('  ★ ROS pose callback installed into sender_core')
        self.get_logger().info('  → 现在启动 sender_core 主流程 (GStreamer + LiveKit)')

    def _on_pose_received(self, *, pitch_deg, yaw_deg, roll_deg, pos, quat, ts):
        """sender_core._on_data_received 调到这里, 把 pose 转成 ROS message 发出."""
        try:
            stamp = self.get_clock().now().to_msg()

            # 1. 完整的 HeadPose
            msg = HeadPose()
            msg.header.stamp = stamp
            msg.header.frame_id = 'vp_head_initial'  # 相对 VP 进 Immersive 那一刻的姿态
            msg.pitch_deg = float(pitch_deg)
            msg.yaw_deg   = float(yaw_deg)
            msg.roll_deg  = float(roll_deg)
            msg.pos_x = float(pos[0])
            msg.pos_y = float(pos[1])
            msg.pos_z = float(pos[2])
            msg.quat_x = float(quat[0])
            msg.quat_y = float(quat[1])
            msg.quat_z = float(quat[2])
            msg.quat_w = float(quat[3])
            self.head_pose_pub.publish(msg)

            # 2. 简化版 Vector3Stamped (rqt_plot 直接用)
            #    x = roll(侧倾), y = pitch(点头), z = yaw(转头)
            rpy = Vector3Stamped()
            rpy.header.stamp = stamp
            rpy.header.frame_id = 'vp_head_initial'
            rpy.vector.x = float(roll_deg)
            rpy.vector.y = float(pitch_deg)
            rpy.vector.z = float(yaw_deg)
            self.head_pose_rpy_pub.publish(rpy)

            self.publish_count += 1
            # 每 150 次 (~5 秒 @ 30Hz) 打印一次确认
            if self.publish_count == 1 or self.publish_count % 150 == 0:
                self.get_logger().info(
                    f'  publish #{self.publish_count}: '
                    f'P={pitch_deg:+.1f}° Y={yaw_deg:+.1f}° R={roll_deg:+.1f}°'
                )
        except Exception as e:
            self.get_logger().error(f'publish failed: {e}')


def main(args=None):
    """主入口: 起 ROS Node + spin 线程, 然后跑 sender_core.main() (asyncio 主循环)."""
    rclpy.init(args=args)
    node = LiveKitBridgeNode()

    # rclpy spin 跑在独立线程, 不会阻塞 sender_core 的 asyncio
    # 因为 publisher.publish() 线程安全, 不需要 asyncio + rclpy 的复杂集成
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node),
        daemon=True,
        name='rclpy-spin',
    )
    spin_thread.start()
    node.get_logger().info('  rclpy spin thread started, handing over to sender_core...')

    try:
        # sender_core.main() 内部启动 GLib 线程 + asyncio main loop, 阻塞直到 Ctrl+C
        sender_core.main()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('shutting down...')
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
