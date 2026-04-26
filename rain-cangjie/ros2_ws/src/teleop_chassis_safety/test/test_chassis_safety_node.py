#!/usr/bin/env python3
"""
test_chassis_safety_node.py — chassis_safety_node unit tests (no ROS 2 required)

测试思路: 把 ROS 2 的 Node 基类 mock 掉, 只验证纯 Python 状态机逻辑:
  - 风车分级阈值 (classify_cmd_age)
  - 状态转换 sticky 特性 (HARD → 不能下降)
  - Event severity 映射
  - SDO write frame 编码
  - reset_safety 只在 HARD 下生效

不测试:
  - ROS 2 service callback 绑定 (需要 rclpy 运行时)
  - 真实 CAN 发送 (需要 can0 接口)
  - 子模块完整集成 (需要 zlac_diag/damiao_diag 安装)

运行:
  python3 -m pytest test/test_chassis_safety_node.py -v
  或直接: python3 test/test_chassis_safety_node.py
"""
from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "teleop_chassis_safety"))


def _stub_rclpy_and_msgs() -> None:
    """Stub rclpy + ROS message imports so chassis_safety_node imports cleanly."""
    for mod_name in (
        "rclpy", "rclpy.node", "rclpy.qos", "rclpy.callback_groups",
        "rclpy.executors",
        "std_msgs", "std_msgs.msg",
        "std_srvs", "std_srvs.srv",
        "geometry_msgs", "geometry_msgs.msg",
    ):
        if mod_name not in sys.modules:
            sys.modules[mod_name] = MagicMock()

    class _FakeNode:
        def __init__(self, *a, **kw):
            self._logger = MagicMock()
            self._params: dict = {}

        def declare_parameter(self, name, default):
            self._params[name] = default

        def get_parameter(self, name):
            p = MagicMock()
            p.value = self._params[name]
            return p

        def get_logger(self):
            return self._logger

        def create_subscription(self, *a, **kw):
            return MagicMock()

        def create_publisher(self, *a, **kw):
            return MagicMock()

        def create_service(self, *a, **kw):
            return MagicMock()

        def create_timer(self, *a, **kw):
            return MagicMock()

        def destroy_node(self):
            pass

    sys.modules["rclpy.node"].Node = _FakeNode
    sys.modules["rclpy.qos"].QoSProfile = MagicMock()
    sys.modules["rclpy.qos"].ReliabilityPolicy = MagicMock()
    sys.modules["rclpy.callback_groups"].ReentrantCallbackGroup = MagicMock()


_stub_rclpy_and_msgs()

from chassis_safety_node import (  # noqa: E402
    CW_ENABLE_OP,
    CW_FAULT_RESET,
    CW_QUICK_STOP,
    ChassisSafetyNode,
    FengcheState,
    _build_sdo_write,
)


def _make_node() -> ChassisSafetyNode:
    """Construct a node with diagnostics bypassed."""
    with patch.object(ChassisSafetyNode, "_init_diagnostics", lambda self: None):
        n = ChassisSafetyNode()
    n._bus = None
    n._diag_zlac = None
    n._diag_dm = None
    n._ops_client = None
    return n


def test_sdo_frame_encoding() -> None:
    cob, data = _build_sdo_write(4, 0x6040, 0, 0x000F, 2)
    assert cob == 0x604
    assert data[0] == 0x2B
    assert data[1] == 0x40 and data[2] == 0x60
    assert data[3] == 0x00
    assert data[4] == 0x0F and data[5] == 0x00

    cob, data = _build_sdo_write(2, 0x60FF, 0, -100, 4)
    assert cob == 0x602
    assert data[0] == 0x23
    assert data[4] == 0x9C and data[5] == 0xFF and data[6] == 0xFF and data[7] == 0xFF


def test_classify_cmd_age_thresholds() -> None:
    n = _make_node()
    assert n._classify_cmd_age(50) == FengcheState.NOMINAL
    assert n._classify_cmd_age(250) == FengcheState.WARN
    assert n._classify_cmd_age(600) == FengcheState.SOFT_FENGCHE
    assert n._classify_cmd_age(1500) == FengcheState.HARD_FENGCHE
    assert n._classify_cmd_age(200) == FengcheState.WARN
    assert n._classify_cmd_age(500) == FengcheState.SOFT_FENGCHE
    assert n._classify_cmd_age(1000) == FengcheState.HARD_FENGCHE


def test_hard_is_sticky() -> None:
    n = _make_node()
    n._transition(FengcheState.HARD_FENGCHE, reason="test", fault=None)
    assert n._state == FengcheState.HARD_FENGCHE
    n._transition(FengcheState.NOMINAL, reason="test", fault=None)
    assert n._state == FengcheState.HARD_FENGCHE
    n._transition(FengcheState.WARN, reason="test", fault=None)
    assert n._state == FengcheState.HARD_FENGCHE


def test_reset_safety_clears_hard() -> None:
    n = _make_node()
    n._transition(FengcheState.HARD_FENGCHE, reason="test", fault=None)
    class _Resp:
        success = None
        message = ""
    resp = n._srv_reset(object(), _Resp())
    assert resp.success is True
    assert n._state == FengcheState.NOMINAL


def test_reset_safety_noop_when_not_hard() -> None:
    n = _make_node()
    n._state = FengcheState.WARN
    class _Resp:
        success = None
        message = ""
    resp = n._srv_reset(object(), _Resp())
    assert resp.success is False
    assert "Not in HARD_FENGCHE" in resp.message


def test_state_transitions_emit_events() -> None:
    n = _make_node()
    n._event_pub = MagicMock()
    n._transition(FengcheState.WARN, reason="synthetic", fault=None)
    assert n._event_pub.publish.called
    payload = n._event_pub.publish.call_args[0][0].data
    assert "transition_NOMINAL_to_WARN" in payload


def test_build_status_snapshot_shape() -> None:
    n = _make_node()
    snap = n._build_status()
    required = {"timestamp", "state", "cmd_age_ms", "thresholds_ms",
                "max_soft_retries", "latest_fault", "zlac", "damiao"}
    assert required.issubset(snap.keys())
    assert snap["state"] == "NOMINAL"


def test_controlword_constants() -> None:
    assert CW_QUICK_STOP == 0x0002
    assert CW_FAULT_RESET == 0x0080
    assert CW_ENABLE_OP == 0x000F


def _run_all() -> int:
    tests = [
        test_sdo_frame_encoding,
        test_classify_cmd_age_thresholds,
        test_hard_is_sticky,
        test_reset_safety_clears_hard,
        test_reset_safety_noop_when_not_hard,
        test_state_transitions_emit_events,
        test_build_status_snapshot_shape,
        test_controlword_constants,
    ]
    fails = 0
    for t in tests:
        try:
            t()
            print(f"  ✓ {t.__name__}")
        except AssertionError as e:
            fails += 1
            print(f"  ✗ {t.__name__}: {e}")
        except Exception as e:
            fails += 1
            print(f"  ✗ {t.__name__}: {type(e).__name__}: {e}")
    return fails


if __name__ == "__main__":
    rc = _run_all()
    print(f"\n{'FAIL: ' + str(rc) + ' test(s) failed' if rc else 'All tests passed'}")
    sys.exit(rc)
