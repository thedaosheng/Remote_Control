#!/usr/bin/env python3
"""
chassis_safety_node.py — 仓颉/Faber 底盘 safety 节点

监控 chassis ROS 2 cmd 信号 + ZLAC8015D + DM4310 motor 状态,
按"风车"(Phone Crazy) 分级模型响应:

    NOMINAL → WARN → SOFT_FENGCHE → HARD_FENGCHE

软风车 (可恢复 / 瞬态) 处理: 最多 `max_soft_retries` 次 clear_errors + re_enable
超过阈值升级为硬风车 (sticky, 需人工 reset_safety)

服务 (std_srvs/Trigger, 对齐 teleop_safety_node 风格):
  /chassis/emergency_stop    硬风车 (quick_stop all wheels immediately)
  /chassis/freeze            软风车 (velocity=0, 自动尝试恢复)
  /chassis/clear_errors      ZLAC Fault Reset + DM clear
  /chassis/re_enable         CiA402 re-enable 序列
  /chassis/reset_safety      从 HARD_FENGCHE → NOMINAL (user-triggered)
  /chassis/status            返回完整状态快照 JSON

Topics:
  sub: {cmd_topic} (Twist)                 watchdog 输入
  sub: /mujoco/swerve_cmd (Float64MultiArray[8])  alt watchdog 输入
  pub: /chassis/health (1Hz, String-JSON)  聚合状态
  pub: /chassis/event (String-JSON)        状态转换/故障事件 (structured)

Params (with defaults):
  cmd_topic                "/cmd_vel"
  watchdog_warn_ms         200
  watchdog_soft_ms         500
  watchdog_hard_ms         1000
  max_soft_retries         3
  zlac_node_ids            [2, 3]
  zlac_can_channel         "can0"
  dm_steer_can_ids         [0x13, 0x14, 0x15, 0x16]     # FL FR RL RR (from pi_kinematics_demo)
  dm_steer_names           ["FL", "FR", "RL", "RR"]
  health_pub_hz            1.0
  watchdog_check_hz        50.0
  ops_backend_url          ""     非空时把 event 推到 ops-backend WebSocket

设计: safety node 独立持有 can0 句柄做写操作 (quick_stop / fault_reset),
      与 swerve_zlac_driver_node 的 velocity 写并行. SocketCAN 多路复用,
      保证 driver 崩溃时 estop 依然可达. DM4310 serial (/dev/ttyACM1)
      归 steer 驱动 node 独占, safety 通过 topic /chassis/driver/dm_cmd
      把清错/使能意图传递过去.
"""
from __future__ import annotations

import enum
import json
import threading
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any, Literal

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

# ------------------------------------------------------------------
# Optional sibling modules. First try relative import (installed
# package path, ros2 run context); fall back to top-level (direct
# `python3 chassis_safety_node.py` dev context). Finally tolerate
# absence so the node still starts with reduced diagnostic fidelity.
# ------------------------------------------------------------------
try:
    from .zlac_diag import ZlacDiagnostic  # type: ignore[import-not-found]
    _HAS_ZLAC_DIAG = True
except (ImportError, ValueError):
    try:
        from zlac_diag import ZlacDiagnostic  # type: ignore[import-not-found, no-redef]
        _HAS_ZLAC_DIAG = True
    except Exception:
        ZlacDiagnostic = None  # type: ignore[misc, assignment]
        _HAS_ZLAC_DIAG = False

try:
    from .damiao_diag import DamiaoDiagnostic  # type: ignore[import-not-found]
    _HAS_DM_DIAG = True
except (ImportError, ValueError):
    try:
        from damiao_diag import DamiaoDiagnostic  # type: ignore[import-not-found, no-redef]
        _HAS_DM_DIAG = True
    except Exception:
        DamiaoDiagnostic = None  # type: ignore[misc, assignment]
        _HAS_DM_DIAG = False

try:
    from .ops_backend_client import OpsBackendClient  # type: ignore[import-not-found]
    _HAS_OPS_CLIENT = True
except (ImportError, ValueError):
    try:
        from ops_backend_client import OpsBackendClient  # type: ignore[import-not-found, no-redef]
        _HAS_OPS_CLIENT = True
    except Exception:
        OpsBackendClient = None  # type: ignore[misc, assignment]
        _HAS_OPS_CLIENT = False


# ==================================================================
# 状态机
# ==================================================================

class FengcheState(enum.IntEnum):
    NOMINAL = 0
    WARN = 1
    SOFT_FENGCHE = 2
    HARD_FENGCHE = 3


@dataclass
class ChassisEvent:
    timestamp: float
    source: str        # "watchdog" | "motor" | "service" | "chassis_safety"
    severity: str      # "debug" | "info" | "warn" | "error" | "critical"
    event: str
    details: dict[str, Any]


# ==================================================================
# CiA402 write helpers (safety node's private bus handle)
# ==================================================================

# Controlword values
CW_DISABLE_VOLT = 0x0000
CW_QUICK_STOP   = 0x0002
CW_SHUTDOWN     = 0x0006
CW_SWITCH_ON    = 0x0007
CW_ENABLE_OP    = 0x000F
CW_FAULT_RESET  = 0x0080

SDO_WRITE_1B = 0x2F
SDO_WRITE_2B = 0x2B
SDO_WRITE_4B = 0x23


def _build_sdo_write(node_id: int, index: int, subindex: int,
                     value: int, size: int) -> tuple[int, bytes]:
    """Build a raw SDO expedited write frame. Returns (cob_id, 8-byte data)."""
    cmd = {1: SDO_WRITE_1B, 2: SDO_WRITE_2B, 4: SDO_WRITE_4B}.get(size, SDO_WRITE_4B)
    if value < 0:
        uv = value & 0xFFFFFFFF
    else:
        uv = value & 0xFFFFFFFF
    data = bytes([
        cmd,
        index & 0xFF, (index >> 8) & 0xFF,
        subindex & 0xFF,
        uv & 0xFF, (uv >> 8) & 0xFF, (uv >> 16) & 0xFF, (uv >> 24) & 0xFF,
    ])
    return (0x600 + node_id, data)


# ==================================================================
# Main node
# ==================================================================

class ChassisSafetyNode(Node):
    """仓颉/Faber 底盘 safety 节点 — watchdog + 风车 state machine + motor fault aggregation."""

    _SEV_RANK = {"debug": 0, "info": 1, "warn": 2, "error": 3, "critical": 4}

    def __init__(self) -> None:
        super().__init__("chassis_safety_node")

        # -------- Parameters --------
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("watchdog_warn_ms", 200)
        self.declare_parameter("watchdog_soft_ms", 500)
        self.declare_parameter("watchdog_hard_ms", 1000)
        self.declare_parameter("max_soft_retries", 3)
        self.declare_parameter("zlac_node_ids", [2, 3])
        self.declare_parameter("zlac_can_channel", "can0")
        self.declare_parameter("dm_steer_can_ids", [0x13, 0x14, 0x15, 0x16])
        self.declare_parameter("dm_steer_names", ["FL", "FR", "RL", "RR"])
        self.declare_parameter("health_pub_hz", 1.0)
        self.declare_parameter("watchdog_check_hz", 50.0)
        self.declare_parameter("ops_backend_url", "")
        self.declare_parameter("motor_poll_hz", 5.0)
        self.declare_parameter("startup_grace_s", 2.0)
        self.declare_parameter("soft_retry_backoff_s", 0.5)

        self.cmd_topic: str = self.get_parameter("cmd_topic").value
        self.warn_ms: int = int(self.get_parameter("watchdog_warn_ms").value)
        self.soft_ms: int = int(self.get_parameter("watchdog_soft_ms").value)
        self.hard_ms: int = int(self.get_parameter("watchdog_hard_ms").value)
        self.max_retries: int = int(self.get_parameter("max_soft_retries").value)
        self.zlac_node_ids: list[int] = list(self.get_parameter("zlac_node_ids").value)
        self.dm_can_ids: list[int] = list(self.get_parameter("dm_steer_can_ids").value)
        self.dm_names: list[str] = list(self.get_parameter("dm_steer_names").value)
        self.ops_backend_url: str = self.get_parameter("ops_backend_url").value
        self.startup_grace_s: float = float(self.get_parameter("startup_grace_s").value)
        self.soft_retry_backoff_s: float = float(self.get_parameter("soft_retry_backoff_s").value)

        # -------- State --------
        self._state: FengcheState = FengcheState.NOMINAL
        self._state_lock = threading.RLock()
        self._last_cmd_ts: float | None = None
        self._startup_ts: float = time.monotonic()
        self._soft_retry_count: int = 0
        self._recovery_in_progress: bool = False
        self._latest_fault: dict | None = None

        # -------- ROS interfaces (callback group needed by diag subs below) --------
        self._cb = ReentrantCallbackGroup()

        # -------- Diagnostic subsystems (best-effort init) --------
        self._bus: Any = None          # python-can Bus (safety-owned)
        self._diag_zlac: Any = None
        self._diag_dm: Any = None
        self._ops_client: Any = None
        self._init_diagnostics()
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        fast_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Twist, self.cmd_topic, self._on_cmd, fast_qos,
                                 callback_group=self._cb)
        self.create_subscription(Float64MultiArray, "/mujoco/swerve_cmd",
                                 self._on_swerve_cmd, fast_qos, callback_group=self._cb)

        self._health_pub = self.create_publisher(String, "/chassis/health", 10)
        self._event_pub = self.create_publisher(String, "/chassis/event", 50)
        self._dm_cmd_pub = self.create_publisher(String, "/chassis/driver/dm_cmd", 10)

        # Services
        self.create_service(Trigger, "/chassis/emergency_stop",
                            self._srv_estop, callback_group=self._cb)
        self.create_service(Trigger, "/chassis/freeze",
                            self._srv_freeze, callback_group=self._cb)
        self.create_service(Trigger, "/chassis/clear_errors",
                            self._srv_clear, callback_group=self._cb)
        self.create_service(Trigger, "/chassis/re_enable",
                            self._srv_reenable, callback_group=self._cb)
        self.create_service(Trigger, "/chassis/reset_safety",
                            self._srv_reset, callback_group=self._cb)
        self.create_service(Trigger, "/chassis/status",
                            self._srv_status, callback_group=self._cb)

        # Timers
        wd_period = 1.0 / float(self.get_parameter("watchdog_check_hz").value)
        poll_period = 1.0 / float(self.get_parameter("motor_poll_hz").value)
        hp_period = 1.0 / float(self.get_parameter("health_pub_hz").value)
        self.create_timer(wd_period, self._watchdog_tick, callback_group=self._cb)
        self.create_timer(poll_period, self._motor_poll_tick, callback_group=self._cb)
        self.create_timer(hp_period, self._publish_health, callback_group=self._cb)

        self._emit_event(
            severity="info",
            source="chassis_safety",
            event="node_started",
            details={
                "state": self._state.name,
                "cmd_topic": self.cmd_topic,
                "thresholds_ms": {"warn": self.warn_ms, "soft": self.soft_ms, "hard": self.hard_ms},
                "max_retries": self.max_retries,
                "modules": {
                    "zlac_diag": _HAS_ZLAC_DIAG,
                    "dm_diag": _HAS_DM_DIAG,
                    "ops_client": _HAS_OPS_CLIENT and bool(self.ops_backend_url),
                    "can_bus": self._bus is not None,
                },
            },
        )

    # ==============================================================
    # Initialisation helpers
    # ==============================================================

    def _init_diagnostics(self) -> None:
        # Open own CAN bus handle (read + write)
        try:
            import can  # type: ignore[import-not-found]
            self._bus = can.interface.Bus(
                channel=self.get_parameter("zlac_can_channel").value,
                interface="socketcan",
            )
            self.get_logger().info(
                f'safety CAN bus opened on {self.get_parameter("zlac_can_channel").value}')
        except Exception as e:
            self.get_logger().warn(f"safety CAN bus open failed: {e}")
            self._bus = None

        # ZLAC diagnostic (EMCY + heartbeat listener + fault decode)
        if _HAS_ZLAC_DIAG and self._bus is not None:
            try:
                self._diag_zlac = ZlacDiagnostic(bus=self._bus, node_ids=self.zlac_node_ids)
                self._diag_zlac.start_listener()
                self.get_logger().info(f"ZLAC diag listener started for {self.zlac_node_ids}")
            except Exception as e:
                self.get_logger().warn(f"ZLAC diag init failed: {e}")
                self._diag_zlac = None

        # DaMiao diagnostic (parser only; motor replies flow via driver node topic)
        if _HAS_DM_DIAG:
            try:
                motor_map = dict(zip(self.dm_can_ids, self.dm_names, strict=False))
                self._diag_dm = DamiaoDiagnostic(motor_map=motor_map)
                # Subscribe to driver's raw MIT replies
                self.create_subscription(
                    String, "/chassis/driver/dm_reply",
                    self._on_dm_reply, 50, callback_group=self._cb,
                )
                self.get_logger().info(f"DM diag parser ready for {motor_map}")
            except Exception as e:
                self.get_logger().warn(f"DM diag init failed: {e}")
                self._diag_dm = None

        # Ops backend WebSocket client
        if _HAS_OPS_CLIENT and self.ops_backend_url:
            try:
                self._ops_client = OpsBackendClient(
                    url=self.ops_backend_url, source="chassis",
                )
                self._ops_client.connect()
                self.get_logger().info(f"ops backend connected: {self.ops_backend_url}")
            except Exception as e:
                self.get_logger().warn(f"ops backend connect failed: {e}")
                self._ops_client = None

    # ==============================================================
    # Subscribers
    # ==============================================================

    def _on_cmd(self, _msg: Twist) -> None:
        with self._state_lock:
            self._last_cmd_ts = time.monotonic()

    def _on_swerve_cmd(self, _msg: Float64MultiArray) -> None:
        with self._state_lock:
            self._last_cmd_ts = time.monotonic()

    def _on_dm_reply(self, msg: String) -> None:
        if self._diag_dm is None:
            return
        try:
            data = json.loads(msg.data)
            raw = bytes.fromhex(data["frame_hex"])
            can_id = int(data["can_id"])
            self._diag_dm.parse_mit_reply(raw, can_id)
        except Exception as e:
            self.get_logger().debug(f"dm_reply parse err: {e}")

    # ==============================================================
    # Watchdog tick (50 Hz)
    # ==============================================================

    def _watchdog_tick(self) -> None:
        now = time.monotonic()
        with self._state_lock:
            last = self._last_cmd_ts
            current = self._state

        # Startup grace: don't trigger anything until we've had time to boot + first cmd
        if last is None:
            if (now - self._startup_ts) > self.startup_grace_s and current == FengcheState.NOMINAL:
                # Long time no cmd — warn but don't soft-fengche (not running yet vs runaway)
                if (now - self._startup_ts) > (self.startup_grace_s + self.warn_ms / 1000.0):
                    self._transition(FengcheState.WARN,
                                     reason=f"no_cmd_since_startup age_s={now - self._startup_ts:.1f}",
                                     fault=None)
            return

        age_ms = (now - last) * 1000.0
        target = self._classify_cmd_age(age_ms)

        # Don't downgrade out of HARD_FENGCHE — sticky
        if current == FengcheState.HARD_FENGCHE:
            return

        self._transition(target,
                         reason=f"cmd_age_ms={age_ms:.0f}",
                         fault=None)

    def _classify_cmd_age(self, age_ms: float) -> FengcheState:
        if age_ms >= self.hard_ms:
            return FengcheState.HARD_FENGCHE
        if age_ms >= self.soft_ms:
            return FengcheState.SOFT_FENGCHE
        if age_ms >= self.warn_ms:
            return FengcheState.WARN
        return FengcheState.NOMINAL

    # ==============================================================
    # Motor poll tick (5 Hz) — aggregate hardware faults
    # ==============================================================

    def _motor_poll_tick(self) -> None:
        fault = self._poll_motor_faults()
        with self._state_lock:
            self._latest_fault = fault
            current = self._state

        if fault is None:
            return

        sev: str = fault.get("severity", "none")
        if sev == "hard":
            if current != FengcheState.HARD_FENGCHE:
                self._transition(FengcheState.HARD_FENGCHE,
                                 reason=f'motor_hard {fault.get("source")}:{fault.get("code")}',
                                 fault=fault)
        elif sev == "soft":
            if current in (FengcheState.NOMINAL, FengcheState.WARN):
                self._transition(FengcheState.SOFT_FENGCHE,
                                 reason=f'motor_soft {fault.get("source")}:{fault.get("code")}',
                                 fault=fault)

    def _poll_motor_faults(self) -> dict | None:
        """Return the most-severe fault across ZLAC+DM, or None."""
        reports: list[dict] = []

        if self._diag_zlac is not None:
            for nid in self.zlac_node_ids:
                try:
                    r = self._diag_zlac.poll_fault(nid, timeout=0.05)
                    if r and getattr(r, "severity", "none") != "none":
                        reports.append({
                            "source": "zlac",
                            "node_id": nid,
                            "code": int(getattr(r, "code", 0)),
                            "zh": getattr(r, "zh", ""),
                            "severity": getattr(r, "severity", "soft"),
                            "recoverable": bool(getattr(r, "recoverable", False)),
                        })
                except Exception as e:
                    self.get_logger().debug(f"zlac poll {nid}: {e}")
            # Bus health
            try:
                bh = self._diag_zlac.check_bus_health()
                if not bh.get("healthy", True):
                    reports.append({
                        "source": "zlac_bus",
                        "code": -1,
                        "zh": f'CAN 总线异常: {bh.get("state")}',
                        "severity": "hard",
                        "recoverable": False,
                    })
            except Exception:
                pass

        if self._diag_dm is not None:
            now_w = time.time()
            for cid, name in zip(self.dm_can_ids, self.dm_names, strict=False):
                try:
                    if self._diag_dm.check_offline(cid, now_w):
                        reports.append({
                            "source": "damiao",
                            "motor_id": cid,
                            "motor_name": name,
                            "code": -1,
                            "zh": f"DM 电机 {name} 离线",
                            "severity": "hard",
                            "recoverable": False,
                        })
                except Exception:
                    pass

        if not reports:
            return None
        rank = {"none": 0, "soft": 1, "hard": 2}
        return max(reports, key=lambda r: rank.get(r.get("severity", "none"), 0))

    # ==============================================================
    # State transition
    # ==============================================================

    def _transition(self, new: FengcheState, reason: str, fault: dict | None) -> None:
        with self._state_lock:
            old = self._state
            if old == new:
                return
            # HARD is sticky
            if old == FengcheState.HARD_FENGCHE and new != FengcheState.HARD_FENGCHE:
                return
            self._state = new
            if new == FengcheState.SOFT_FENGCHE and old != FengcheState.SOFT_FENGCHE:
                self._soft_retry_count = 0
            elif new not in (FengcheState.SOFT_FENGCHE,):
                self._soft_retry_count = 0

        sev_map = {
            FengcheState.NOMINAL: "info",
            FengcheState.WARN: "warn",
            FengcheState.SOFT_FENGCHE: "error",
            FengcheState.HARD_FENGCHE: "critical",
        }
        self._emit_event(
            severity=sev_map[new],
            source="watchdog" if fault is None else "motor",
            event=f"transition_{old.name}_to_{new.name}",
            details={"reason": reason, "fault": fault},
        )

        if new == FengcheState.HARD_FENGCHE:
            self._enact_hard_fengche(reason)
        elif new == FengcheState.SOFT_FENGCHE:
            self._enact_soft_fengche(reason)

    # ==============================================================
    # Fengche enactment
    # ==============================================================

    def _enact_soft_fengche(self, reason: str) -> None:
        """Soft: quick_stop + clear + re-enable, up to max_retries."""
        self._emit_event("warn", "chassis_safety", "soft_fengche_enter", {"reason": reason})
        with self._state_lock:
            if self._recovery_in_progress:
                return
            self._recovery_in_progress = True
        threading.Thread(target=self._soft_recovery_loop, daemon=True).start()

    def _soft_recovery_loop(self) -> None:
        try:
            while True:
                with self._state_lock:
                    if self._state != FengcheState.SOFT_FENGCHE:
                        return
                    self._soft_retry_count += 1
                    attempt = self._soft_retry_count
                    retries_cap = self.max_retries

                self._emit_event(
                    "info", "chassis_safety", "soft_recovery_attempt",
                    {"attempt": attempt, "max": retries_cap},
                )

                # 1. Halt: quick_stop all ZLAC + publish DM freeze intent
                self._write_zlac_quick_stop()
                self._publish_dm_cmd("freeze")
                time.sleep(0.3)

                # 2. Clear errors
                zlac_cleared = self._write_zlac_fault_reset()
                self._publish_dm_cmd("clear_error")
                time.sleep(0.3)

                # 3. Re-enable
                zlac_enabled = self._write_zlac_enable_sequence()
                self._publish_dm_cmd("enable")
                time.sleep(0.5)

                # 4. Verify no latched fault
                residual = self._poll_motor_faults()
                has_hard_residual = residual is not None and residual.get("severity") == "hard"
                success = zlac_cleared and zlac_enabled and not has_hard_residual

                if success:
                    self._emit_event(
                        "info", "chassis_safety", "soft_recovery_success",
                        {"attempt": attempt, "residual": residual},
                    )
                    with self._state_lock:
                        if self._state == FengcheState.SOFT_FENGCHE:
                            self._state = FengcheState.NOMINAL
                        self._soft_retry_count = 0
                    return

                if attempt >= retries_cap:
                    self._emit_event(
                        "critical", "chassis_safety", "soft_recovery_exhausted",
                        {
                            "attempts": attempt,
                            "zlac_cleared": zlac_cleared,
                            "zlac_enabled": zlac_enabled,
                            "residual_fault": residual,
                        },
                    )
                    self._transition(
                        FengcheState.HARD_FENGCHE,
                        reason=f"soft_retry_exhausted_{attempt}",
                        fault=residual,
                    )
                    return

                self._emit_event(
                    "warn", "chassis_safety", "soft_recovery_retry_backoff",
                    {"attempt": attempt, "backoff_s": self.soft_retry_backoff_s},
                )
                time.sleep(self.soft_retry_backoff_s)
        finally:
            with self._state_lock:
                self._recovery_in_progress = False

    def _enact_hard_fengche(self, reason: str) -> None:
        """Hard: immediate estop, best-effort, no further retries."""
        self._emit_event("critical", "chassis_safety", "hard_fengche_enter", {"reason": reason})
        self._write_zlac_quick_stop()
        self._publish_dm_cmd("disable")

    # ==============================================================
    # CAN writes (safety-owned bus)
    # ==============================================================

    def _can_send(self, cob_id: int, data: bytes) -> bool:
        if self._bus is None:
            return False
        try:
            import can  # type: ignore[import-not-found]
            self._bus.send(can.Message(
                arbitration_id=cob_id, data=data, is_extended_id=False))
            return True
        except Exception as e:
            self.get_logger().warn(f"can send failed cob=0x{cob_id:X}: {e}")
            return False

    def _write_zlac_controlword(self, value: int) -> bool:
        """Write 0x6040 Controlword = value for all ZLAC nodes."""
        ok = True
        for nid in self.zlac_node_ids:
            cob, data = _build_sdo_write(nid, 0x6040, 0, value, 2)
            ok = self._can_send(cob, data) and ok
        return ok

    def _write_zlac_quick_stop(self) -> bool:
        return self._write_zlac_controlword(CW_QUICK_STOP)

    def _write_zlac_fault_reset(self) -> bool:
        return self._write_zlac_controlword(CW_FAULT_RESET)

    def _write_zlac_enable_sequence(self) -> bool:
        ok = True
        for cw in (CW_SHUTDOWN, CW_SWITCH_ON, CW_ENABLE_OP):
            if not self._write_zlac_controlword(cw):
                ok = False
            time.sleep(0.05)
        return ok

    def _publish_dm_cmd(self, cmd: Literal["enable", "disable", "clear_error", "freeze"]) -> None:
        """Publish DM intent for driver node (which owns serial) to enact."""
        msg = String()
        msg.data = json.dumps({"cmd": cmd, "ts": time.time(),
                               "can_ids": [int(c) for c in self.dm_can_ids]})
        self._dm_cmd_pub.publish(msg)

    # ==============================================================
    # Service callbacks
    # ==============================================================

    def _srv_estop(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        self._transition(FengcheState.HARD_FENGCHE, reason="service_estop", fault=None)
        resp.success = True
        resp.message = "Hard 风车 engaged — chassis frozen until reset_safety"
        return resp

    def _srv_freeze(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        self._transition(FengcheState.SOFT_FENGCHE, reason="service_freeze", fault=None)
        resp.success = True
        resp.message = f"Soft 风车 engaged — will retry up to {self.max_retries} times"
        return resp

    def _srv_clear(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        ok_z = self._write_zlac_fault_reset()
        self._publish_dm_cmd("clear_error")
        self._emit_event("info", "service", "clear_errors", {"zlac_ok": ok_z})
        resp.success = ok_z
        resp.message = f"zlac_fault_reset={ok_z}, dm_clear_cmd_published=true"
        return resp

    def _srv_reenable(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        ok_z = self._write_zlac_enable_sequence()
        self._publish_dm_cmd("enable")
        self._emit_event("info", "service", "re_enable", {"zlac_ok": ok_z})
        resp.success = ok_z
        resp.message = f"zlac_reenable={ok_z}, dm_enable_cmd_published=true"
        return resp

    def _srv_reset(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        with self._state_lock:
            old = self._state
            if old != FengcheState.HARD_FENGCHE:
                resp.success = False
                resp.message = f"Not in HARD_FENGCHE (current={old.name}); nothing to reset"
                return resp
            self._state = FengcheState.NOMINAL
            self._soft_retry_count = 0
            self._last_cmd_ts = None
            self._startup_ts = time.monotonic()
            self._recovery_in_progress = False
        self._emit_event(
            "warn", "service", "reset_safety",
            {"previous": old.name, "new": FengcheState.NOMINAL.name},
        )
        resp.success = True
        resp.message = "Safety state reset to NOMINAL (startup grace re-armed)"
        return resp

    def _srv_status(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        resp.success = True
        resp.message = json.dumps(self._build_status(), default=str, ensure_ascii=False)
        return resp

    # ==============================================================
    # Health + event publishing
    # ==============================================================

    def _publish_health(self) -> None:
        snapshot = self._build_status()
        msg = String()
        msg.data = json.dumps(snapshot, default=str, ensure_ascii=False)
        self._health_pub.publish(msg)
        if self._ops_client is not None:
            try:
                self._ops_client.push_state(snapshot)
            except Exception as e:
                self.get_logger().debug(f"ops push_state: {e}")

    def _build_status(self) -> dict:
        with self._state_lock:
            state_name = self._state.name
            retries = self._soft_retry_count
            last_cmd = self._last_cmd_ts
            latest_fault = self._latest_fault

        now = time.monotonic()
        cmd_age_ms = ((now - last_cmd) * 1000.0) if last_cmd is not None else None

        zlac_status: dict | None = None
        if self._diag_zlac is not None:
            try:
                nodes: dict[int, dict] = {}
                now_w = time.time()
                for nid in self.zlac_node_ids:
                    hb_ts = self._diag_zlac.last_heartbeat(nid)
                    nodes[nid] = {
                        "heartbeat_age_s": (now_w - hb_ts) if hb_ts else None,
                    }
                zlac_status = {
                    "bus": self._diag_zlac.check_bus_health(),
                    "nodes": nodes,
                }
            except Exception as e:
                zlac_status = {"error": str(e)}

        dm_status: dict | None = None
        if self._diag_dm is not None:
            try:
                per_motor: dict[str, dict] = {}
                now_w = time.time()
                for cid, name in zip(self.dm_can_ids, self.dm_names, strict=False):
                    last = self._diag_dm.last_seen(cid)
                    per_motor[name] = {
                        "can_id": hex(cid),
                        "last_seen_age_s": (now_w - last) if last else None,
                        "offline": self._diag_dm.check_offline(cid, now_w),
                    }
                dm_status = per_motor
            except Exception as e:
                dm_status = {"error": str(e)}

        return {
            "timestamp": time.time(),
            "state": state_name,
            "soft_retry_count": retries,
            "cmd_age_ms": cmd_age_ms,
            "cmd_topic": self.cmd_topic,
            "thresholds_ms": {"warn": self.warn_ms, "soft": self.soft_ms, "hard": self.hard_ms},
            "max_soft_retries": self.max_retries,
            "latest_fault": latest_fault,
            "zlac": zlac_status,
            "damiao": dm_status,
            "can_bus_open": self._bus is not None,
        }

    def _emit_event(self, severity: str, source: str, event: str, details: dict) -> None:
        ev = ChassisEvent(
            timestamp=time.time(),
            source=source,
            severity=severity,
            event=event,
            details=details,
        )
        payload = json.dumps(asdict(ev), default=str, ensure_ascii=False)
        msg = String()
        msg.data = payload
        self._event_pub.publish(msg)

        log = self.get_logger()
        if severity == "debug":
            log.debug(f"[{event}] {payload}")
        elif severity == "info":
            log.info(f"[{event}] {payload}")
        elif severity == "warn":
            log.warn(f"[{event}] {payload}")
        else:
            log.error(f"[{event}] {payload}")

        if self._ops_client is not None:
            try:
                self._ops_client.push_log(
                    severity=severity, event=event, details={**details, "source": source})
            except Exception as e:
                log.debug(f"ops push_log: {e}")

    # ==============================================================
    # Shutdown
    # ==============================================================

    def on_shutdown(self) -> None:
        # Log-only final notice; no topic publish at shutdown (publisher's
        # context can already be invalid if SIGINT triggered a partial teardown).
        try:
            self.get_logger().info(f"node_shutdown final_state={self._state.name}")
        except Exception:
            pass
        try:
            if self._diag_zlac is not None:
                self._diag_zlac.stop_listener()
        except Exception:
            pass
        try:
            if self._bus is not None:
                self._bus.shutdown()
        except Exception:
            pass
        try:
            if self._ops_client is not None:
                self._ops_client.close()
        except Exception:
            pass


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ChassisSafetyNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.on_shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        # rclpy's SIGINT handler may already have called shutdown; guard against double-call.
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
