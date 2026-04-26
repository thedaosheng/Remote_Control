#!/usr/bin/env python3
"""
================================================================
  zlac_diag_test.py — zlac_diag 模块单元测试
================================================================

可用 pytest 或直接 python zlac_diag_test.py 运行.

测试覆盖:
  1. YAML 加载 + schema 完整性 (每个 code 必填字段)
  2. code_to_zh / classify / is_recoverable 已知 + 未知 code
  3. parse_emcy_frame 伪造 EMCY 帧
  4. 无硬件构造 (bus=None)
  5. 心跳 / bus-health / drain_emcy
"""
from __future__ import annotations

import logging
import struct
import sys
import time
from pathlib import Path

# 把本目录加入 sys.path 以支持直接运行
_HERE = Path(__file__).parent.resolve()
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from zlac_diag import (  # noqa: E402
    PSEUDO_CODE_BUS_OFF,
    PSEUDO_CODE_HEARTBEAT_LOST,
    FaultReport,
    ZlacDiagnostic,
)

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s %(name)s %(levelname)s %(message)s",
)
log = logging.getLogger("zlac_diag_test")


# ============================================================
# Helpers
# ============================================================
REQUIRED_FIELDS_CIA = {"source", "zh", "en", "severity", "recoverable",
                       "latching", "notes", "verified"}
REQUIRED_FIELDS_MODBUS = {"zh", "en", "severity", "recoverable", "latching",
                          "notes", "verified"}


def _new_diag() -> ZlacDiagnostic:
    """sibling YAML + no bus"""
    return ZlacDiagnostic(bus=None, node_ids=[2, 3])


# ============================================================
# Test 1 — YAML schema
# ============================================================
def test_yaml_schema() -> None:
    """每个 cia402 / modbus code 必须有完整字段."""
    diag = _new_diag()
    root = diag._dict.get("zlac8015d", {})
    assert root, "yaml root 'zlac8015d' missing"

    cia = root.get("cia402_error_codes", {})
    assert cia, "cia402_error_codes 必须非空"
    for code, entry in cia.items():
        missing = REQUIRED_FIELDS_CIA - set(entry.keys())
        assert not missing, f"cia402 code 0x{code:04X} 缺字段 {missing}"
        assert entry["severity"] in ("none", "soft", "hard"), \
            f"cia402 code 0x{code:04X} severity 非法"

    mod = root.get("modbus_fault_codes", {})
    assert mod, "modbus_fault_codes 必须非空"
    for code, entry in mod.items():
        missing = REQUIRED_FIELDS_MODBUS - set(entry.keys())
        assert not missing, f"modbus code 0x{code:04X} 缺字段 {missing}"
        assert entry["severity"] in ("none", "soft", "hard"), \
            f"modbus code 0x{code:04X} severity 非法"

    # statusword_bits
    sw = root.get("statusword_bits", {})
    assert sw, "statusword_bits 必须非空"
    assert 3 in sw and sw[3]["role"] == "fault", \
        "statusword bit 3 必须是 fault"
    assert 5 in sw, "statusword bit 5 (quick_stop) 必须定义"

    # EMCY frame format
    emcy = root.get("emcy_frame_format", {})
    assert emcy, "emcy_frame_format 必须定义"
    assert emcy.get("cob_id_base") == 0x80
    assert emcy.get("dlc") == 8
    assert len(emcy.get("bytes", [])) == 8

    # heartbeat
    hb = root.get("heartbeat", {})
    assert hb.get("cob_id_base") == 0x700
    assert hb.get("default_period_ms", 0) > 0
    assert hb.get("recommended_timeout_ms", 0) >= hb["default_period_ms"]

    log.info("test_yaml_schema OK: cia402=%d modbus=%d", len(cia), len(mod))


# ============================================================
# Test 2 — code_to_zh / classify / recoverable
# ============================================================
def test_code_to_zh_known() -> None:
    diag = _new_diag()
    # modbus 已知
    assert diag.code_to_zh(0x0000) == "无故障"
    assert diag.code_to_zh(0x0001) == "过压"
    assert diag.code_to_zh(0x0200) == "HALL 故障"
    assert diag.code_to_zh(0x0400) == "电机温度过高"
    # cia402 已知
    assert diag.code_to_zh(0x7305) == "编码器/HALL 传感器故障"
    assert "过流" in diag.code_to_zh(0x2310)
    log.info("test_code_to_zh_known OK")


def test_code_to_zh_unknown() -> None:
    diag = _new_diag()
    zh = diag.code_to_zh(0xABCD)
    assert "未知故障" in zh and "ABCD" in zh, f"fallback failed: {zh}"
    # 伪 code
    assert "心跳丢失" in diag.code_to_zh(PSEUDO_CODE_HEARTBEAT_LOST)
    assert "Bus-Off" in diag.code_to_zh(PSEUDO_CODE_BUS_OFF)
    log.info("test_code_to_zh_unknown OK")


def test_classify_known() -> None:
    diag = _new_diag()
    # modbus 表里的 severity
    assert diag.classify(0x0000) == "none"
    assert diag.classify(0x0001) == "soft"      # over-voltage
    assert diag.classify(0x0004) == "hard"      # over-current
    assert diag.classify(0x0008) == "soft"      # over-load
    assert diag.classify(0x0200) == "hard"      # HALL
    assert diag.classify(0x0100) == "hard"      # EEPROM
    assert diag.classify(0x0400) == "soft"      # high motor temp
    # cia402
    assert diag.classify(0x2310) == "hard"      # over-current
    assert diag.classify(0x7305) == "hard"      # HALL
    assert diag.classify(0x3110) == "soft"      # over-voltage mains
    # 伪 code
    assert diag.classify(PSEUDO_CODE_BUS_OFF) == "hard"
    assert diag.classify(PSEUDO_CODE_HEARTBEAT_LOST) == "hard"
    log.info("test_classify_known OK")


def test_classify_unknown_heuristic() -> None:
    """未登记 code 按 CiA 高位 nibble 启发."""
    diag = _new_diag()
    # 0x2xxx → hard (current)
    assert diag.classify(0x2999) == "hard"
    # 0x5xxx → hard (device hardware)
    assert diag.classify(0x5999) == "hard"
    # 0x3xxx → soft (voltage)
    assert diag.classify(0x3999) == "soft"
    # 0x4xxx → soft (temperature)
    assert diag.classify(0x4999) == "soft"
    # 0x8xxx → soft (monitoring / following)
    assert diag.classify(0x8999) == "soft"
    # 其他 → soft (保守)
    assert diag.classify(0x1999) == "soft"
    log.info("test_classify_unknown_heuristic OK")


def test_is_recoverable() -> None:
    diag = _new_diag()
    assert diag.is_recoverable(0x0001) is True       # over-voltage soft
    assert diag.is_recoverable(0x0200) is False      # HALL hard
    assert diag.is_recoverable(0x0004) is False      # over-current hard
    # 未知 code: 基于 classify
    assert diag.is_recoverable(0x2999) is False      # hard → False
    assert diag.is_recoverable(0x8999) is True       # soft → True
    log.info("test_is_recoverable OK")


# ============================================================
# Test 3 — EMCY frame parsing
# ============================================================
def test_parse_emcy_known_code() -> None:
    """伪造一个 ZLAC HALL fault EMCY 帧, 验证 parse."""
    diag = _new_diag()
    # HALL fault cia402 code = 0x7305
    data = bytearray(8)
    struct.pack_into("<H", data, 0, 0x7305)  # error code
    data[2] = 0x01  # error register: generic
    # bytes 3-7: manufacturer specific (ZLAC 布局未知) — 填 0
    cob_id = 0x80 + 2   # node 2
    rpt = diag.parse_emcy_frame(cob_id, bytes(data))

    assert rpt is not None
    assert rpt.node_id == 2
    assert rpt.code == 0x7305
    assert "HALL" in rpt.zh or "编码器" in rpt.zh
    assert rpt.severity == "hard"
    assert rpt.recoverable is False
    assert rpt.source == "emcy"
    assert rpt.raw == bytes(data)
    log.info("test_parse_emcy_known_code OK: %s", rpt)


def test_parse_emcy_unknown_code() -> None:
    diag = _new_diag()
    data = bytearray(8)
    struct.pack_into("<H", data, 0, 0xDEAD)
    cob_id = 0x80 + 3  # node 3
    rpt = diag.parse_emcy_frame(cob_id, bytes(data))
    assert rpt is not None
    assert rpt.code == 0xDEAD
    assert "未知" in rpt.zh
    assert rpt.source == "emcy"
    log.info("test_parse_emcy_unknown_code OK")


def test_parse_emcy_reset_frame() -> None:
    """CiA301 规范: error_code=0 的 EMCY = '错误已清除'"""
    diag = _new_diag()
    data = bytes(8)  # all zeros
    cob_id = 0x80 + 2
    rpt = diag.parse_emcy_frame(cob_id, data)
    assert rpt is not None
    assert rpt.code == 0x0000
    assert rpt.severity == "none"
    assert "清除" in rpt.zh or "reset" in rpt.en.lower()
    log.info("test_parse_emcy_reset_frame OK")


def test_parse_emcy_wrong_node() -> None:
    """来自非监控 node 的 EMCY 返回 None."""
    diag = _new_diag()
    data = bytearray(8)
    struct.pack_into("<H", data, 0, 0x7305)
    cob_id = 0x80 + 99   # 不在 [2, 3]
    rpt = diag.parse_emcy_frame(cob_id, bytes(data))
    assert rpt is None
    log.info("test_parse_emcy_wrong_node OK")


def test_parse_emcy_not_emcy_cob() -> None:
    diag = _new_diag()
    rpt = diag.parse_emcy_frame(0x600 + 2, bytes(8))
    assert rpt is None
    log.info("test_parse_emcy_not_emcy_cob OK")


# ============================================================
# Test 4 — 无硬件构造
# ============================================================
def test_construct_no_bus() -> None:
    diag = ZlacDiagnostic(bus=None, node_ids=[2, 3])
    assert diag.node_ids == [2, 3]
    assert diag.poll_fault(2) is None
    assert diag.send_fault_reset(2) is False
    # bus_health
    health = diag.check_bus_health()
    assert health["state"] == "UNKNOWN"
    assert health["healthy"] is False
    # listener no-op
    diag.start_listener()  # should log warn and return
    diag.stop_listener()
    log.info("test_construct_no_bus OK")


# ============================================================
# Test 5 — 心跳 / drain_emcy
# ============================================================
class _MockHealthyBus:
    """模拟 bus.state == ACTIVE 的最小 bus (仅用于测试 heartbeat 超时逻辑)."""
    class _State:
        name = "ACTIVE"
    state = _State()
    # 测试不调用 send/recv, 所以不必实现


def test_heartbeat_tracking() -> None:
    """单测 heartbeat 超时逻辑. 绕开 bus_health 用 mock bus."""
    # 先用 bus=None 验证 last_heartbeat + drain
    diag = _new_diag()
    now = time.time()

    # 还没收到心跳
    assert diag.last_heartbeat(2) is None

    # 注入心跳到内部状态
    with diag._hb_lock:
        diag._last_heartbeat[2] = now
    assert diag.last_heartbeat(2) == now

    # 用 mock healthy bus 验证 detect_dropped 的 heartbeat-age 分支
    # (bus=None 时 check_bus_health 会返回 unhealthy, 短路为 dropped=True,
    # 那是正确行为但不是我们在此测试的分支)
    diag2 = ZlacDiagnostic(bus=_MockHealthyBus(), node_ids=[2, 3])
    with diag2._hb_lock:
        diag2._last_heartbeat[2] = now
    assert diag2.check_bus_health()["healthy"] is True
    assert diag2.detect_dropped(2, now) is False            # 刚收到
    assert diag2.detect_dropped(2, now + 0.05) is False     # 50ms < 300ms
    assert diag2.detect_dropped(2, now + 5.0) is True       # 5s >> 300ms

    # bus=None 时 detect_dropped 应保守报 dropped=True (bus_health unhealthy)
    assert diag.detect_dropped(2, now) is True
    log.info("test_heartbeat_tracking OK")


def test_drain_emcy_ring() -> None:
    """emcy_buffer 环形缓冲 + drain/peek 语义."""
    diag = _new_diag()
    # 注入 40 条, 应截断到 _emcy_buffer_max
    for i in range(40):
        rpt = FaultReport(
            node_id=2, subindex=None, code=0x0001,
            zh="过压", en="Over-voltage",
            severity="soft", recoverable=True,
            source="emcy", timestamp=time.time(),
            raw=None,
        )
        with diag._emcy_lock:
            buf = diag._emcy_buffer[2]
            buf.append(rpt)
            if len(buf) > diag._emcy_buffer_max:
                del buf[: len(buf) - diag._emcy_buffer_max]

    peek = diag.peek_emcy(2)
    assert len(peek) == diag._emcy_buffer_max

    drained = diag.drain_emcy(2)
    assert len(drained) == diag._emcy_buffer_max
    # drain 后应空
    assert diag.peek_emcy(2) == []
    log.info("test_drain_emcy_ring OK")


def test_highest_bit_code() -> None:
    """modbus bitmap 最高 bit 提取."""
    diag = _new_diag()
    assert diag._highest_bit_code(0x0000) == 0
    assert diag._highest_bit_code(0x0001) == 0x0001
    assert diag._highest_bit_code(0x0400) == 0x0400
    # 多 bit: 0x0201 (HALL + over-voltage) → 取 HALL (0x0200)
    assert diag._highest_bit_code(0x0201) == 0x0200
    # 0x0005 (over-current + over-voltage) → 取 over-current (0x0004)
    assert diag._highest_bit_code(0x0005) == 0x0004
    log.info("test_highest_bit_code OK")


# ============================================================
# 主入口
# ============================================================
def _run_all() -> int:
    """未装 pytest 时的 fallback runner."""
    tests = [
        test_yaml_schema,
        test_code_to_zh_known,
        test_code_to_zh_unknown,
        test_classify_known,
        test_classify_unknown_heuristic,
        test_is_recoverable,
        test_parse_emcy_known_code,
        test_parse_emcy_unknown_code,
        test_parse_emcy_reset_frame,
        test_parse_emcy_wrong_node,
        test_parse_emcy_not_emcy_cob,
        test_construct_no_bus,
        test_heartbeat_tracking,
        test_drain_emcy_ring,
        test_highest_bit_code,
    ]
    fails = 0
    for t in tests:
        try:
            t()
        except AssertionError as exc:
            fails += 1
            log.error("FAIL %s: %s", t.__name__, exc)
        except Exception as exc:  # noqa: BLE001
            fails += 1
            log.exception("ERROR %s: %s", t.__name__, exc)
    total = len(tests)
    log.info("=" * 50)
    log.info("Result: %d/%d passed, %d failed", total - fails, total, fails)
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(_run_all())
