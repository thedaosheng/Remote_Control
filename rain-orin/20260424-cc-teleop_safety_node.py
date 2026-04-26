#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
====================================================================
  teleop_safety_node — AIRBOT 双臂 safety / recovery ROS2 Services
====================================================================

Addresses YZL's 2026-04-24 demand:
  "机械臂到奇怪位置能手动 disable, 回零后再启用. 电机报错能先失能再使能恢复"

Exposes std_srvs/Trigger services per arm (left on :50051 / right on :50052):

  /arm/left/emergency_stop   → set_arm_emergency_stop(True), motor torque OFF
  /arm/left/clear_errors     → clear_arm_motor_err + clear_eef_motor_err
  /arm/left/freeze           → enter_gravity_compensation_mode (low current, 手动可推)
  /arm/left/return_zero      → return_zero (走预设轨迹回零点)
  /arm/left/re_enable        → acquire_control + switch_controller(mit_control)

  (same for /arm/right/*)

  /arms/emergency_stop_all   → stop both (bus-wide panic button)
  /arms/status               → get_service_state 两臂

内部通过 arm_sdk.AirbotClient 调用 gRPC(到本机 airbot-arm@<idx>-<port>),
gRPC server 必须先起来 (sudo systemctl start airbot-arm@2-50051).
如果 gRPC 连不上 (e.g. 右臂 motor4 fail) service 返回 success=False + 错误说明.

Data-channel passthrough (separate ROS2 subscriber):
  VP 可以 publish_data 到 teleop-room, 发送 JSON:
    {"type": "cmd", "cmd": "emergency_stop_all"}
    {"type": "cmd", "cmd": "freeze", "arm": "left"}
    {"type": "cmd", "cmd": "return_zero", "arm": "left"}
  lk_data_bridge 节点扩展后 parse → 调对应本地 service.
"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

import arm_sdk


class ArmSafetyHelper:
    """一臂的 safety API 封装. 每次调用独立建 connection, 避免 lease 长持有."""

    def __init__(self, port: int, logger):
        self._port = port
        self._log = logger

    def _connect(self):
        return arm_sdk.AirbotClient("localhost", self._port)

    def _safe(self, fn, *args, **kwargs):
        """Run fn against a fresh client, returns (ok, message)."""
        try:
            c = self._connect()
        except Exception as e:
            return False, f"grpc connect :{self._port} failed: {e}"
        try:
            c.acquire_control()
            result = fn(c, *args, **kwargs)
            c.release_control()
            return True, result
        except Exception as e:
            try:
                c.release_control()
            except Exception:
                pass
            return False, f"{fn.__name__} failed: {e}"
        finally:
            try:
                c.close()
            except Exception:
                pass

    def emergency_stop(self):
        return self._safe(lambda c: (c.set_arm_emergency_stop(True), "torque OFF"))

    def clear_errors(self):
        def _do(c):
            c.clear_arm_motor_err()
            c.clear_eef_motor_err()
            return "errors cleared"
        return self._safe(_do)

    def freeze_gravity_comp(self):
        return self._safe(lambda c: (c.enter_gravity_compensation_mode(), "gravity_comp ON"))

    def return_zero(self):
        return self._safe(lambda c: (c.return_zero(), "return_zero issued"))

    def re_enable(self):
        def _do(c):
            c.switch_controller(arm_sdk.Controller.mit_control)
            return "re-enabled (mit_control)"
        return self._safe(_do)

    def status(self):
        try:
            c = self._connect()
            s = c.get_service_state()
            c.close()
            return True, str(s)
        except Exception as e:
            return False, f"grpc :{self._port} unreachable: {e}"


class TeleopSafetyNode(Node):
    def __init__(self):
        super().__init__('teleop_safety')

        self.left = ArmSafetyHelper(port=50051, logger=self.get_logger())
        self.right = ArmSafetyHelper(port=50052, logger=self.get_logger())

        def mk(arm_name, helper, action_name, action_fn):
            def callback(request, response):
                ok, msg = action_fn()
                response.success = bool(ok)
                response.message = f"[{arm_name}/{action_name}] {msg}"
                self.get_logger().info(response.message)
                return response
            srv_name = f"/arm/{arm_name}/{action_name}"
            self.create_service(Trigger, srv_name, callback)
            return srv_name

        # Per-arm services
        for arm_name, h in [("left", self.left), ("right", self.right)]:
            mk(arm_name, h, "emergency_stop", h.emergency_stop)
            mk(arm_name, h, "clear_errors",   h.clear_errors)
            mk(arm_name, h, "freeze",         h.freeze_gravity_comp)
            mk(arm_name, h, "return_zero",    h.return_zero)
            mk(arm_name, h, "re_enable",      h.re_enable)
            mk(arm_name, h, "status",         h.status)

        # Bus-wide panic button
        def emergency_stop_all_cb(request, response):
            ok_l, msg_l = self.left.emergency_stop()
            ok_r, msg_r = self.right.emergency_stop()
            response.success = bool(ok_l or ok_r)  # 至少一臂成功就算 true (panic always reports)
            response.message = f"left: {msg_l} | right: {msg_r}"
            self.get_logger().warn(f"[arms/emergency_stop_all] {response.message}")
            return response
        self.create_service(Trigger, "/arms/emergency_stop_all", emergency_stop_all_cb)

        # Global status
        def status_all_cb(request, response):
            ok_l, msg_l = self.left.status()
            ok_r, msg_r = self.right.status()
            response.success = True
            response.message = f"left: {msg_l} | right: {msg_r}"
            return response
        self.create_service(Trigger, "/arms/status", status_all_cb)

        self.get_logger().info("[teleop_safety] 12 services ready.")
        self.get_logger().info("  /arm/{left|right}/{emergency_stop,clear_errors,freeze,return_zero,re_enable,status}")
        self.get_logger().info("  /arms/emergency_stop_all   /arms/status")


def main(args=None):
    rclpy.init(args=args)
    node = TeleopSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
