#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
  damiao_diag_test.py - damiao_diag.py 单元测试
================================================================================
覆盖:
  1. YAML schema 加载 / 健壮性
  2. MIT 反馈帧解码 - 正常帧 (state=1 使能)
  3. MIT 反馈帧解码 - 故障帧 (state ∈ {8, 9, A, B, C, D, E})
  4. 位置 / 速度 / 扭矩的数值反映射 (边界 + 中点 + 正负对称)
  5. 温度字段
  6. 帧长异常处理
  7. 命令帧构造 (enable/disable/clear_error/set_zero)
  8. 看门狗 / 掉线检测 (update_last_seen / check_offline / offline_motors)
  9. 线程安全 (多线程并发 parse + check_offline)
  10. DM3519 的 vmax=200 限值差异

运行:
  - pytest: pytest damiao_diag_test.py -v
  - 独立:   python damiao_diag_test.py
"""
from __future__ import annotations

import sys
import time
import threading
from pathlib import Path
from typing import List, Tuple

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from damiao_diag import (
    DamiaoDiagnostic,
    FaultReport,
    decode_mit_reply_standalone,
    CMD_ENABLE_DATA,
    CMD_DISABLE_DATA,
    CMD_CLEAR_ERROR_DATA,
    CMD_SET_ZERO_DATA,
)


# --------------------------------------------------------------------------- #
# 辅助函数 - 构造符合达妙反馈帧协议的 8 字节 raw
# --------------------------------------------------------------------------- #
def make_reply_frame(
    state: int,
    id_low4: int,
    pos_rad: float,
    vel_rad_s: float,
    torque_nm: float,
    t_mos: int,
    t_rotor: int,
    pmax: float = 12.5,
    vmax: float = 30.0,
    tmax: float = 10.0,
) -> bytes:
    """
    编码与 DM_CAN.py controlMIT() 反向对应, 用于单元测试.
    输入浮点值, 输出符合反馈帧 bit 布局的 8 字节.
    """
    # 浮点 -> 定点
    def f2u(x: float, xmin: float, xmax: float, bits: int) -> int:
        span = xmax - xmin
        norm = (x - xmin) / span
        max_v = (1 << bits) - 1
        u = int(round(norm * max_v))
        # clip
        return max(0, min(max_v, u))

    q_uint = f2u(pos_rad, -pmax, pmax, 16)        # 16-bit
    dq_uint = f2u(vel_rad_s, -vmax, vmax, 12)     # 12-bit
    tau_uint = f2u(torque_nm, -tmax, tmax, 12)    # 12-bit

    b = bytearray(8)
    b[0] = ((state & 0xF) << 4) | (id_low4 & 0xF)
    b[1] = (q_uint >> 8) & 0xFF
    b[2] = q_uint & 0xFF
    b[3] = (dq_uint >> 4) & 0xFF
    b[4] = ((dq_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF)
    b[5] = tau_uint & 0xFF
    b[6] = t_mos & 0xFF
    b[7] = t_rotor & 0xFF
    return bytes(b)


YAML_PATH = Path(__file__).parent / "damiao_errors.yaml"


# =========================================================================== #
# Section 1: YAML 加载
# =========================================================================== #
class TestYamlLoad:
    def test_yaml_exists(self):
        assert YAML_PATH.exists(), f"YAML must exist at {YAML_PATH}"

    def test_yaml_parses(self):
        import yaml as _y
        with YAML_PATH.open("r", encoding="utf-8") as f:
            data = _y.safe_load(f)
        assert isinstance(data, dict)
        assert "dm4310" in data
        assert "dm3519" in data

    def test_yaml_has_required_error_codes(self):
        """至少需要有 0x8 过压 ~ 0xE 过载 这些权威故障码."""
        import yaml as _y
        with YAML_PATH.open("r", encoding="utf-8") as f:
            data = _y.safe_load(f)
        for model in ("dm4310", "dm3519"):
            codes = data[model]["state_error_codes"]
            # YAML 里的 key 可能被解析为 int (0x8 = 8) 或 str
            keys_normalized = set()
            for k in codes:
                if isinstance(k, int):
                    keys_normalized.add(k)
                else:
                    keys_normalized.add(int(str(k), 0))
            for required in [0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xE]:
                assert required in keys_normalized, f"{model} missing code 0x{required:X}"

    def test_diag_loads_yaml(self):
        diag = DamiaoDiagnostic(
            motor_map={0x13: "RR"},
            yaml_path=YAML_PATH,
            motor_model="dm4310",
        )
        # 加载后 state code 0x8 必须是 "过压保护"
        assert "过压" in diag.code_to_zh(0x8)

    def test_diag_handles_missing_yaml(self):
        """不提供 YAML 时应当用内置默认值, 不崩."""
        diag = DamiaoDiagnostic(
            motor_map={0x13: "RR"},
            yaml_path=None,
            motor_model="dm4310",
        )
        assert diag.code_to_zh(0x1) == "使能"
        assert diag.code_to_zh(0xA) == "过电流保护"

    def test_diag_handles_bad_yaml_path(self):
        """YAML 路径不存在时应当 warning + fallback."""
        diag = DamiaoDiagnostic(
            motor_map={0x13: "RR"},
            yaml_path=Path("/nonexistent/path.yaml"),
            motor_model="dm4310",
        )
        # 仍能用内置默认值
        assert diag.code_to_zh(0x8) == "过压保护"


# =========================================================================== #
# Section 2: 解码正常帧 (state = 0x1 使能)
# =========================================================================== #
class TestDecodeNormal:
    def test_enabled_zero_state(self):
        """state=1 使能, pos=0, vel=0, tau=0, T_MOS=25°C, T_rotor=30°C."""
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"}, motor_model="dm4310")
        frame = make_reply_frame(state=0x1, id_low4=0x3,
                                 pos_rad=0.0, vel_rad_s=0.0, torque_nm=0.0,
                                 t_mos=25, t_rotor=30)
        rep = diag.parse_mit_reply(frame, can_id=0x13)
        assert rep.motor_id == 0x13
        assert rep.motor_name == "RR"
        assert rep.state == 0x1
        assert rep.state_zh == "使能"
        assert rep.error_code == 0x0
        assert rep.error_zh == "无故障"
        assert rep.severity == "none"
        assert rep.is_fault() is False
        assert rep.temp_mos_c == 25.0
        assert rep.temp_rotor_c == 30.0
        # position ~= 0 rad (浮点 round-trip 误差 < 1 LSB)
        assert abs(rep.position) < 0.001
        assert abs(rep.velocity) < 0.02
        assert abs(rep.torque) < 0.01

    def test_disabled_state(self):
        diag = DamiaoDiagnostic(motor_map={0x14: "FR"}, motor_model="dm4310")
        frame = make_reply_frame(state=0x0, id_low4=0x4,
                                 pos_rad=0.0, vel_rad_s=0.0, torque_nm=0.0,
                                 t_mos=22, t_rotor=22)
        rep = diag.parse_mit_reply(frame, can_id=0x14)
        assert rep.state == 0x0
        assert rep.state_zh == "失能"
        assert rep.severity == "none"
        assert rep.is_fault() is False

    def test_non_zero_position(self):
        """pos=3.14rad (~π), vel=1.5rad/s, tau=2.5Nm."""
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"}, motor_model="dm4310")
        frame = make_reply_frame(state=0x1, id_low4=0x3,
                                 pos_rad=3.14, vel_rad_s=1.5, torque_nm=2.5,
                                 t_mos=40, t_rotor=45)
        rep = diag.parse_mit_reply(frame, can_id=0x13)
        # 16-bit 精度: 12.5/32767 ≈ 0.00038 rad/LSB
        assert abs(rep.position - 3.14) < 0.001
        # 12-bit 速度精度: 30/2047 ≈ 0.0147 rad/s/LSB
        assert abs(rep.velocity - 1.5) < 0.02
        # 12-bit 扭矩精度: 10/2047 ≈ 0.00489 Nm/LSB
        assert abs(rep.torque - 2.5) < 0.01

    def test_negative_position(self):
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"}, motor_model="dm4310")
        frame = make_reply_frame(state=0x1, id_low4=0x3,
                                 pos_rad=-2.0, vel_rad_s=-1.0, torque_nm=-3.0,
                                 t_mos=30, t_rotor=33)
        rep = diag.parse_mit_reply(frame, can_id=0x13)
        assert rep.position < 0
        assert abs(rep.position - (-2.0)) < 0.001
        assert rep.velocity < 0
        assert rep.torque < 0

    def test_boundary_positive(self):
        """pos 在 PMAX 极限 (+12.5 rad)."""
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"}, motor_model="dm4310")
        frame = make_reply_frame(state=0x1, id_low4=0x3,
                                 pos_rad=12.5, vel_rad_s=30.0, torque_nm=10.0,
                                 t_mos=50, t_rotor=55)
        rep = diag.parse_mit_reply(frame, can_id=0x13)
        # 顶值 round-trip 会有 ±1 LSB 误差
        assert abs(rep.position - 12.5) < 0.001
        assert abs(rep.velocity - 30.0) < 0.02
        assert abs(rep.torque - 10.0) < 0.01


# =========================================================================== #
# Section 3: 解码故障帧
# =========================================================================== #
class TestDecodeFaults:
    @pytest.mark.parametrize("state_code,expected_zh_contains", [
        (0x5, "传感器"),
        (0x6, "电机参数"),
        (0x8, "过压"),
        (0x9, "欠压"),
        (0xA, "过电流"),
        (0xB, "MOS"),
        (0xC, "线圈"),
        (0xD, "通讯"),
        (0xE, "过载"),
    ])
    def test_each_fault_code(self, state_code: int, expected_zh_contains: str):
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"}, motor_model="dm4310")
        frame = make_reply_frame(state=state_code, id_low4=0x3,
                                 pos_rad=0.0, vel_rad_s=0.0, torque_nm=0.0,
                                 t_mos=80, t_rotor=85)
        rep = diag.parse_mit_reply(frame, can_id=0x13)
        assert rep.state == state_code
        assert rep.error_code == state_code
        assert expected_zh_contains in rep.state_zh
        assert rep.is_fault() is True

    def test_over_voltage_severity(self):
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"},
                                yaml_path=YAML_PATH,
                                motor_model="dm4310")
        frame = make_reply_frame(state=0x8, id_low4=0x3,
                                 pos_rad=0.0, vel_rad_s=0.0, torque_nm=0.0,
                                 t_mos=30, t_rotor=35)
        rep = diag.parse_mit_reply(frame, can_id=0x13)
        assert rep.severity == "hard"
        assert rep.recoverable is False

    def test_comm_lost_severity(self):
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"},
                                yaml_path=YAML_PATH,
                                motor_model="dm4310")
        frame = make_reply_frame(state=0xD, id_low4=0x3,
                                 pos_rad=0.0, vel_rad_s=0.0, torque_nm=0.0,
                                 t_mos=30, t_rotor=35)
        rep = diag.parse_mit_reply(frame, can_id=0x13)
        assert rep.severity == "soft"   # 通讯丢失可重连
        assert rep.recoverable is True

    def test_mos_overtemp_recoverable(self):
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"},
                                yaml_path=YAML_PATH,
                                motor_model="dm4310")
        frame = make_reply_frame(state=0xB, id_low4=0x3,
                                 pos_rad=0.0, vel_rad_s=0.0, torque_nm=0.0,
                                 t_mos=120, t_rotor=90)
        rep = diag.parse_mit_reply(frame, can_id=0x13)
        assert rep.recoverable is True  # 冷却后可恢复
        assert rep.severity == "hard"
        assert rep.temp_mos_c == 120.0

    def test_unknown_code_conservative(self):
        """未知 state code 应保守按 hard 处理."""
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"}, motor_model="dm4310")
        # 0x7 在两份手册中均未列出
        frame = make_reply_frame(state=0x7, id_low4=0x3,
                                 pos_rad=0.0, vel_rad_s=0.0, torque_nm=0.0,
                                 t_mos=25, t_rotor=25)
        rep = diag.parse_mit_reply(frame, can_id=0x13)
        assert "未知" in rep.state_zh
        assert rep.severity == "hard"
        assert rep.recoverable is False
        assert rep.is_fault() is True


# =========================================================================== #
# Section 4: 温度字段 + temp_c property
# =========================================================================== #
class TestTemperature:
    def test_temp_c_picks_higher(self):
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"}, motor_model="dm4310")
        frame = make_reply_frame(state=0x1, id_low4=0x3,
                                 pos_rad=0.0, vel_rad_s=0.0, torque_nm=0.0,
                                 t_mos=45, t_rotor=70)
        rep = diag.parse_mit_reply(frame, can_id=0x13)
        assert rep.temp_c == 70.0

    def test_temp_c_range(self):
        """温度字段是 uint8, 极端情况 255°C (传感器上限)."""
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"}, motor_model="dm4310")
        frame = make_reply_frame(state=0x1, id_low4=0x3,
                                 pos_rad=0.0, vel_rad_s=0.0, torque_nm=0.0,
                                 t_mos=255, t_rotor=255)
        rep = diag.parse_mit_reply(frame, can_id=0x13)
        assert rep.temp_mos_c == 255.0
        assert rep.temp_rotor_c == 255.0


# =========================================================================== #
# Section 5: 帧长异常处理
# =========================================================================== #
class TestBadFrame:
    def test_short_frame(self):
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"}, motor_model="dm4310")
        rep = diag.parse_mit_reply(bytes([0x13, 0x80]), can_id=0x13)
        assert rep.state == -1
        assert "错误" in rep.state_zh
        assert rep.severity == "hard"
        assert rep.position is None

    def test_long_frame(self):
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"}, motor_model="dm4310")
        rep = diag.parse_mit_reply(bytes(16), can_id=0x13)
        assert rep.state == -1
        assert "错误" in rep.state_zh

    def test_unknown_can_id(self):
        """can_id 不在 motor_map 中, 应当生成 UNKNOWN_0xXX 名字但不崩."""
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"}, motor_model="dm4310")
        frame = make_reply_frame(state=0x1, id_low4=0x9,
                                 pos_rad=0.0, vel_rad_s=0.0, torque_nm=0.0,
                                 t_mos=25, t_rotor=25)
        rep = diag.parse_mit_reply(frame, can_id=0x99)
        assert "UNKNOWN" in rep.motor_name


# =========================================================================== #
# Section 6: 命令帧构造
# =========================================================================== #
class TestCommandFrames:
    def test_enable_frame(self):
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"}, motor_model="dm4310")
        f = diag.build_enable_frame()
        assert f == bytes([0xFF] * 7 + [0xFC])
        assert f == CMD_ENABLE_DATA
        assert len(f) == 8

    def test_disable_frame(self):
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"}, motor_model="dm4310")
        f = diag.build_disable_frame()
        assert f == bytes([0xFF] * 7 + [0xFD])
        assert f == CMD_DISABLE_DATA

    def test_clear_error_frame(self):
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"}, motor_model="dm4310")
        f = diag.build_clear_error_frame()
        assert f == bytes([0xFF] * 7 + [0xFB])
        assert f == CMD_CLEAR_ERROR_DATA

    def test_set_zero_frame(self):
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"}, motor_model="dm4310")
        f = diag.build_set_zero_frame()
        assert f == bytes([0xFF] * 7 + [0xFE])
        assert f == CMD_SET_ZERO_DATA

    def test_cmd_all_distinct(self):
        """四条命令帧最后一字节必须互异."""
        last_bytes = {
            CMD_ENABLE_DATA[-1],
            CMD_DISABLE_DATA[-1],
            CMD_CLEAR_ERROR_DATA[-1],
            CMD_SET_ZERO_DATA[-1],
        }
        assert len(last_bytes) == 4


# =========================================================================== #
# Section 7: 看门狗 / 掉线检测
# =========================================================================== #
class TestOfflineDetection:
    def test_never_seen_is_offline(self):
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"}, motor_model="dm4310")
        now = time.monotonic()
        assert diag.check_offline(0x13, now) is True
        assert diag.last_seen(0x13) is None

    def test_just_received_is_online(self):
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"}, motor_model="dm4310")
        frame = make_reply_frame(state=0x1, id_low4=0x3,
                                 pos_rad=0.0, vel_rad_s=0.0, torque_nm=0.0,
                                 t_mos=25, t_rotor=25)
        diag.parse_mit_reply(frame, can_id=0x13)
        now = time.monotonic()
        assert diag.check_offline(0x13, now) is False

    def test_timeout_elapsed(self):
        """人为把 last_seen 回拨 200ms, 100ms 阈值下应判定掉线."""
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"}, motor_model="dm4310")
        frame = make_reply_frame(state=0x1, id_low4=0x3,
                                 pos_rad=0.0, vel_rad_s=0.0, torque_nm=0.0,
                                 t_mos=25, t_rotor=25)
        diag.parse_mit_reply(frame, can_id=0x13)

        # 模拟 200ms 未再收到帧
        fake_now = diag.last_seen(0x13) + 0.2
        assert diag.check_offline(0x13, fake_now, timeout_ms=100) is True

    def test_custom_timeout(self):
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"}, motor_model="dm4310")
        frame = make_reply_frame(state=0x1, id_low4=0x3,
                                 pos_rad=0.0, vel_rad_s=0.0, torque_nm=0.0,
                                 t_mos=25, t_rotor=25)
        diag.parse_mit_reply(frame, can_id=0x13)
        last = diag.last_seen(0x13)
        # 50ms 后: 100ms 阈值下仍在线, 30ms 阈值下已离线
        assert diag.check_offline(0x13, last + 0.05, timeout_ms=100) is False
        assert diag.check_offline(0x13, last + 0.05, timeout_ms=30) is True

    def test_update_last_seen_manual(self):
        diag = DamiaoDiagnostic(motor_map={0x13: "RR"}, motor_model="dm4310")
        now = time.monotonic()
        diag.update_last_seen(0x13, now=now)
        assert diag.last_seen(0x13) == now
        assert diag.check_offline(0x13, now) is False

    def test_offline_motors_list(self):
        diag = DamiaoDiagnostic(
            motor_map={0x13: "RR", 0x14: "FR", 0x15: "RL", 0x16: "FL"},
            motor_model="dm4310",
        )
        # 只让 RR 收到帧
        frame = make_reply_frame(state=0x1, id_low4=0x3,
                                 pos_rad=0.0, vel_rad_s=0.0, torque_nm=0.0,
                                 t_mos=25, t_rotor=25)
        diag.parse_mit_reply(frame, can_id=0x13)

        offline = diag.offline_motors()
        assert 0x13 not in offline
        assert 0x14 in offline
        assert 0x15 in offline
        assert 0x16 in offline

    def test_snapshot(self):
        diag = DamiaoDiagnostic(
            motor_map={0x13: "RR", 0x14: "FR"},
            motor_model="dm4310",
        )
        frame = make_reply_frame(state=0x1, id_low4=0x3,
                                 pos_rad=1.0, vel_rad_s=0.5, torque_nm=0.1,
                                 t_mos=30, t_rotor=35)
        diag.parse_mit_reply(frame, can_id=0x13)

        snap = diag.snapshot()
        assert 0x13 in snap and 0x14 in snap
        assert snap[0x13]["offline"] is False
        assert snap[0x14]["offline"] is True
        assert snap[0x13]["last_report"] is not None
        assert snap[0x14]["last_report"] is None


# =========================================================================== #
# Section 8: DM3519 vmax=200 差异
# =========================================================================== #
class TestDm3519:
    def test_dm3519_uses_vmax_200(self):
        """
        同一 bit pattern 在 DM4310 (vmax=30) vs DM3519 (vmax=200) 应解出不同速度.
        vel_uint=2047 (满量程) → DM4310: ~30 rad/s; DM3519: ~200 rad/s.
        """
        diag4310 = DamiaoDiagnostic(motor_map={0x13: "X"}, motor_model="dm4310")
        diag3519 = DamiaoDiagnostic(motor_map={0x13: "X"}, motor_model="dm3519")
        # 用 vmax=30 编码 30 rad/s
        frame = make_reply_frame(state=0x1, id_low4=0x3,
                                 pos_rad=0.0, vel_rad_s=30.0, torque_nm=0.0,
                                 t_mos=25, t_rotor=25,
                                 vmax=30.0)
        rep_4310 = diag4310.parse_mit_reply(frame, can_id=0x13)
        rep_3519 = diag3519.parse_mit_reply(frame, can_id=0x13)
        # DM4310 应当近似 30 rad/s
        assert abs(rep_4310.velocity - 30.0) < 0.1
        # DM3519 解同一 bit pattern 得到 ~200 rad/s (因为 vmax 不同)
        assert abs(rep_3519.velocity - 200.0) < 1.0

    def test_dm3519_from_yaml(self):
        diag = DamiaoDiagnostic(
            motor_map={0x21: "HEAD_LIFT"},
            yaml_path=YAML_PATH,
            motor_model="dm3519",
        )
        assert diag._limits["vmax"] == 200.0
        assert diag._limits["pmax"] == 12.5
        assert diag._limits["tmax"] == 10.0


# =========================================================================== #
# Section 9: 线程安全
# =========================================================================== #
class TestThreadSafety:
    def test_concurrent_parse_and_check(self):
        """
        多个生产者线程并发喂帧, 一个消费者线程不停调 check_offline / snapshot.
        不应出现 KeyError 或数据竞态 (RLock 保护 self._last_seen / _last_reports).
        """
        diag = DamiaoDiagnostic(
            motor_map={0x13: "RR", 0x14: "FR", 0x15: "RL", 0x16: "FL"},
            motor_model="dm4310",
        )
        stop_event = threading.Event()
        errors: List[Exception] = []

        def producer(can_id: int) -> None:
            id_low = can_id & 0xF
            while not stop_event.is_set():
                try:
                    frame = make_reply_frame(state=0x1, id_low4=id_low,
                                             pos_rad=0.0, vel_rad_s=0.0, torque_nm=0.0,
                                             t_mos=25, t_rotor=25)
                    diag.parse_mit_reply(frame, can_id=can_id)
                except Exception as e:
                    errors.append(e)

        def consumer() -> None:
            while not stop_event.is_set():
                try:
                    _ = diag.snapshot()
                    for cid in [0x13, 0x14, 0x15, 0x16]:
                        _ = diag.check_offline(cid, time.monotonic())
                except Exception as e:
                    errors.append(e)

        threads = [threading.Thread(target=producer, args=(cid,)) for cid in [0x13, 0x14, 0x15, 0x16]]
        threads.append(threading.Thread(target=consumer))
        for t in threads:
            t.start()
        time.sleep(0.3)  # 并发 300ms
        stop_event.set()
        for t in threads:
            t.join(timeout=2.0)

        assert not errors, f"Thread errors: {errors}"


# =========================================================================== #
# Section 10: decode_mit_reply_standalone 便利函数
# =========================================================================== #
class TestStandaloneDecoder:
    def test_basic(self):
        frame = make_reply_frame(state=0x1, id_low4=0x3,
                                 pos_rad=1.0, vel_rad_s=0.5, torque_nm=0.2,
                                 t_mos=30, t_rotor=35)
        rep = decode_mit_reply_standalone(frame, can_id=0x13, motor_name="TEST")
        assert rep.motor_name == "TEST"
        assert rep.state == 0x1
        assert abs(rep.position - 1.0) < 0.001

    def test_override_limits(self):
        """DM3519 的 vmax=200."""
        frame = make_reply_frame(state=0x1, id_low4=0x3,
                                 pos_rad=0.0, vel_rad_s=100.0, torque_nm=0.0,
                                 t_mos=25, t_rotor=25,
                                 vmax=200.0)
        rep = decode_mit_reply_standalone(frame, can_id=0x13,
                                          motor_name="LIFT",
                                          vmax=200.0)
        assert abs(rep.velocity - 100.0) < 0.1


# =========================================================================== #
# __main__ 便携模式 - 不需 pytest 也能跑一遍
# =========================================================================== #
def _run_all_standalone() -> Tuple[int, int]:
    """遍历本文件所有 TestXxx.test_yyy, 独立运行. 返回 (passed, failed)."""
    import inspect

    passed = 0
    failed = 0
    failures: List[str] = []

    module = sys.modules[__name__]
    for cls_name, cls in inspect.getmembers(module, inspect.isclass):
        if not cls_name.startswith("Test"):
            continue
        # 实例化 test class (pytest 允许 no-arg __init__)
        try:
            inst = cls()
        except Exception as e:
            failures.append(f"  [class {cls_name}] __init__ failed: {e}")
            failed += 1
            continue
        for m_name, method in inspect.getmembers(cls, predicate=inspect.isfunction):
            if not m_name.startswith("test_"):
                continue
            # 跳过 parametrize 方法 (pytest 专用)
            if hasattr(method, "pytestmark"):
                # parametrize - 简化: 手动展开. 这里只印跳过.
                print(f"  [SKIP] {cls_name}.{m_name} (parametrize, run via pytest)")
                continue
            try:
                method(inst)
                print(f"  [PASS] {cls_name}.{m_name}")
                passed += 1
            except Exception as e:
                failures.append(f"  [FAIL] {cls_name}.{m_name}: {e}")
                failed += 1

    print("\n" + "=" * 60)
    print(f"  Passed: {passed}, Failed: {failed}")
    print("=" * 60)
    for f in failures:
        print(f)
    return passed, failed


if __name__ == "__main__":
    print("=" * 60)
    print("  damiao_diag_test.py - 独立运行模式")
    print("=" * 60)
    passed, failed = _run_all_standalone()
    sys.exit(0 if failed == 0 else 1)
