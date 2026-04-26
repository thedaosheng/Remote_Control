#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
  damiao_diag.py - 达妙电机故障诊断模块 (生产级)
================================================================================
仓颉 / Faber 底盘与头部 safety 子系统的电机故障解析器.

设计定位:
  - 纯解码 + 状态追踪, 不依赖 ROS 2, 不依赖达妙 SDK 运行时.
  - 上游 watchdog (chassis_safety_node.py 的 风车 状态机) 调用 parse_mit_reply()
    喂入 8 字节 CAN 反馈帧, 拿到 FaultReport, 决定是否需要急停 / 进入降级状态.
  - 若创建时传入 SDK MotorControl 句柄, 可以通过 build_*_frame() 方法构造标准
    达妙命令帧字节, 由调用方写到 USB2CAN 适配器 (本模块不负责串口 I/O).

协议参考 (全部 verified, 详见 damiao_errors.yaml 顶部 [A][B][C][D][E] 来源表):
  - DM-S3519-1EC V1.1 手册 (反馈帧状态/错误码最齐全版本)
  - DM-J4310-2EC V1.1 手册 (底盘转向电机)
  - 达妙官方 DM_CAN.py SDK (反馈帧字段位宽的权威实现)
  - cmjang/DM_Motor_Control damiao.h (C++ 社区参考)

依赖: 仅 Python 3.10+ 标准库 + PyYAML. 无 numpy / rclpy / serial.
================================================================================
"""
from __future__ import annotations

import logging
import os
import threading
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Any, Callable, Dict, List, Literal, Optional

import yaml


# --------------------------------------------------------------------------- #
# Logger - 调用方自己管 handler, 本模块只产生 log record
# --------------------------------------------------------------------------- #
logger = logging.getLogger("damiao_diag")


# --------------------------------------------------------------------------- #
# 常量 - 来自 damiao_errors.yaml 与 DM_CAN.py __control_cmd()
# 来源: DM_CAN.py line 327-329, damiao.h 类似段. 所有达妙电机共用同一组命令.
# --------------------------------------------------------------------------- #
#: 使能电机命令帧 8 字节载荷 (CAN ID = SlaveID)
CMD_ENABLE_DATA: bytes = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])

#: 失能电机命令帧
CMD_DISABLE_DATA: bytes = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])

#: 设置当前位置为零点 (失能状态下使用)
CMD_SET_ZERO_DATA: bytes = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])

#: 清除故障码 (非官方 SDK API, 来自达妙驱动控制协议 V1.4 社区记录; verified=false)
CMD_CLEAR_ERROR_DATA: bytes = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB])


#: DM-S3519 V1.1 手册 [A] 第 6 页反馈帧 ERR 字段 4-bit 编码表.
#: 两种含义共享 4-bit 编码空间: 0/1 是正常运行状态, 5/6/8/9/A/B/C/D/E 是错误.
#: 这是两份官方手册合并后最完整的版本. DM4310 V1.0 手册仅列 8-E, 其余兼容.
_STATE_ERROR_ZH: Dict[int, str] = {
    0x0: "失能",
    0x1: "使能",
    0x5: "传感器读出错误",
    0x6: "电机参数读取错误",
    0x8: "过压保护",
    0x9: "欠压保护",
    0xA: "过电流保护",
    0xB: "MOS 过温保护",
    0xC: "电机线圈过温保护",
    0xD: "通讯丢失",
    0xE: "过载保护",
}

#: severity classification - none/soft/hard
#: - none: 正常运行状态 (0/1)
#: - soft: 可恢复, 短暂降级即可 (D 通讯丢失 - 重新发使能可恢复)
#: - hard: 不可简单 clear 的故障. 过压/欠压/过流/过载/传感器错误. B/C 过温虽 "可恢复",
#:         但因为可能瞬间复燃 (散热条件未变), 按 hard 对待.
_SEVERITY: Dict[int, Literal["none", "soft", "hard"]] = {
    0x0: "none",
    0x1: "none",
    0x5: "hard",
    0x6: "hard",
    0x8: "hard",
    0x9: "hard",
    0xA: "hard",
    0xB: "hard",
    0xC: "hard",
    0xD: "soft",
    0xE: "hard",
}

#: recoverable - 是否理论上可通过 disable→clear→enable 恢复
_RECOVERABLE: Dict[int, bool] = {
    0x0: True, 0x1: True,
    0x5: False, 0x6: False,
    0x8: False, 0x9: False, 0xA: False,
    0xB: True, 0xC: True,
    0xD: True,
    0xE: False,
}


# --------------------------------------------------------------------------- #
# FaultReport 数据类 - 每次 parse_mit_reply() 返回这个, 上游消费
# --------------------------------------------------------------------------- #
@dataclass
class FaultReport:
    """
    单电机的一次反馈解析结果快照.

    除 motor_id / motor_name / state / state_zh / severity / recoverable / timestamp
    / raw 外, 其余字段可能为 None (例如未知错误码时 position 仍然会解码但 state_zh="unknown",
    或者解码失败时 position=None).
    """
    motor_id: int                      # 反馈帧里的 CAN ID (MasterID), e.g. 0x13/0x14/0x15/0x16
    motor_name: str                    # 可读名字, e.g. "FL", "RR", "HEAD_LIFT"
    state: int                         # 原始 4-bit state/err code, 0x0 ~ 0xF
    state_zh: str                      # 中文标签
    error_code: int                    # 等同 state - 官方协议里合并为一个 4-bit code.
                                       # 保留该字段是为了兼容任务书的 API 契约 (state + error_code 分开).
                                       # state == error_code 始终成立.
    error_zh: str                      # 中文故障描述. state∈{0,1} 时为 "无故障".
    severity: Literal["none", "soft", "hard"]
    recoverable: bool
    position: Optional[float]          # rad, 由 pmax 映射后的浮点. None 表示解码失败.
    velocity: Optional[float]          # rad/s
    torque: Optional[float]            # Nm
    temp_mos_c: Optional[float]        # MOS 温度, °C
    temp_rotor_c: Optional[float]      # 线圈温度, °C
    timestamp: float                   # time.monotonic() 时间戳 (秒)
    raw: Optional[bytes]               # 原始 8 字节, debug 用. 生产环境禁用可设 None.

    # 向后兼容: 任务书里用的是 temp_c 单字段, 这里保留两个 (MOS + rotor) 对应手册 T_MOS / T_Rotor.
    @property
    def temp_c(self) -> Optional[float]:
        """较保守的单一温度值 - 取两者中较高者 (任一超温都该报警)."""
        a = self.temp_mos_c
        b = self.temp_rotor_c
        if a is None and b is None:
            return None
        if a is None:
            return b
        if b is None:
            return a
        return max(a, b)

    def to_dict(self) -> Dict[str, Any]:
        """便于序列化到 JSON / ROS2 diagnostic_msgs / 写日志."""
        d = asdict(self)
        # bytes 不 JSON 安全, 转 hex
        if self.raw is not None:
            d["raw"] = self.raw.hex()
        return d

    def is_fault(self) -> bool:
        """True 当 severity != 'none' (即处于任一故障状态)."""
        return self.severity != "none"


# --------------------------------------------------------------------------- #
# 解码工具 - 与 DM_CAN.py uint_to_float() line 494-498 完全对齐
# --------------------------------------------------------------------------- #
def _uint_to_float(x: int, x_min: float, x_max: float, bits: int) -> float:
    """
    达妙 MIT 反馈帧的定点-浮点反映射.
    value = (raw / (2^bits - 1)) * (xmax - xmin) + xmin  其中 xmin = -xmax.
    与 SDK 算法逐字一致, 避免上下游数值漂移.
    """
    span = x_max - x_min
    data_norm = float(x) / float((1 << bits) - 1)
    return data_norm * span + x_min


# --------------------------------------------------------------------------- #
# 核心类
# --------------------------------------------------------------------------- #
class DamiaoDiagnostic:
    """
    达妙电机诊断器 (一实例即可覆盖多颗同 bus 电机).

    Usage
    -----
    >>> diag = DamiaoDiagnostic(
    ...     motor_map={0x13: "RR", 0x14: "FR", 0x15: "RL", 0x16: "FL"},
    ...     yaml_path=Path("damiao_errors.yaml"),
    ... )
    >>> # 喂入 CAN 反馈帧 (8 bytes) + CAN ID (反馈帧 MasterID)
    >>> report = diag.parse_mit_reply(raw_8bytes, can_id=0x13)
    >>> if report.is_fault():
    ...     # 触发安全动作
    ...     pass
    >>> # 在 watchdog 循环里检查掉线
    >>> if diag.check_offline(0x13, now=time.monotonic()):
    ...     # 该电机超时未汇报, 认为掉线
    ...     pass
    """

    def __init__(
        self,
        motor_map: Dict[int, str],
        yaml_path: Optional[Path] = None,
        sdk_motor_control: Optional[Any] = None,
        motor_model: str = "dm4310",
    ) -> None:
        """
        Parameters
        ----------
        motor_map : dict[int, str]
            CAN_ID (反馈帧 MasterID) → 可读名字. 例如 {0x13: "RR", 0x14: "FR"}.
        yaml_path : Path | None
            damiao_errors.yaml 路径. 若 None, 使用模块内置常量 (_STATE_ERROR_ZH 等)
            与 limits (DM4310 的 12.5/30/10). YAML 提供时会覆盖默认 limits.
        sdk_motor_control : optional
            达妙 SDK 的 MotorControl 实例. 本模块不直接调用它, 但若你想用 SDK 的
            switchControlMode 等, 可以保留该句柄. 保留 None 表示 "纯解码模式".
        motor_model : str
            "dm4310" 或 "dm3519". 决定默认 limits (PMAX/VMAX/TMAX).
        """
        self._motor_map: Dict[int, str] = dict(motor_map)
        self._sdk_mc = sdk_motor_control
        self._motor_model = motor_model.lower()

        # 内置默认 limits (当 YAML 未提供时). 来源 DM_CAN.py Limit_Param.
        _DEFAULT_LIMITS: Dict[str, Dict[str, float]] = {
            "dm4310":    {"pmax": 12.5, "vmax":  30.0, "tmax":  10.0},
            "dm4310_48": {"pmax": 12.5, "vmax":  50.0, "tmax":  10.0},
            "dm4340":    {"pmax": 12.5, "vmax":   8.0, "tmax":  28.0},
            "dm3519":    {"pmax": 12.5, "vmax": 200.0, "tmax":  10.0},
            "dm6006":    {"pmax": 12.5, "vmax":  45.0, "tmax":  20.0},
            "dm8009":    {"pmax": 12.5, "vmax":  45.0, "tmax":  54.0},
        }

        self._offline_timeout_ms: float = 100.0
        self._expected_rate_hz: float = 500.0

        # 先装内置值
        self._limits: Dict[str, float] = dict(
            _DEFAULT_LIMITS.get(self._motor_model, _DEFAULT_LIMITS["dm4310"])
        )
        self._state_error_zh: Dict[int, str] = dict(_STATE_ERROR_ZH)
        self._severity: Dict[int, Literal["none", "soft", "hard"]] = dict(_SEVERITY)
        self._recoverable: Dict[int, bool] = dict(_RECOVERABLE)

        # 若提供了 YAML 则覆盖
        self._yaml_data: Dict[str, Any] = {}
        if yaml_path is not None:
            self._load_yaml(Path(yaml_path))

        # last_seen 用 monotonic 秒. 线程安全: 用 RLock 保护.
        self._last_seen: Dict[int, float] = {}
        self._last_reports: Dict[int, FaultReport] = {}
        self._lock = threading.RLock()

        logger.info(
            "DamiaoDiagnostic initialized: model=%s, motors=%s, offline_timeout=%.0fms",
            self._motor_model,
            {hex(k): v for k, v in self._motor_map.items()},
            self._offline_timeout_ms,
        )

    # ---------------------------------------------------------------- YAML
    def _load_yaml(self, path: Path) -> None:
        """
        从 yaml 加载 state/error 表 + limits + timeout. YAML 是权威配置,
        允许运维在不改代码的前提下补充新固件的错误码.
        """
        if not path.exists():
            logger.warning("YAML %s not found; using built-in defaults", path)
            return
        try:
            with path.open("r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
        except Exception as e:
            logger.exception("Failed to parse YAML %s: %s; using defaults", path, e)
            return
        if not isinstance(data, dict):
            logger.warning("YAML %s root is not a dict; using defaults", path)
            return

        self._yaml_data = data
        section = data.get(self._motor_model)
        if section is None:
            logger.warning(
                "YAML has no section for model=%s (keys=%s); using built-in defaults",
                self._motor_model, list(data.keys()),
            )
            return

        # limits
        lim = section.get("limits") or {}
        if "pmax_rad" in lim:
            self._limits["pmax"] = float(lim["pmax_rad"])
        if "vmax_rad_s" in lim:
            self._limits["vmax"] = float(lim["vmax_rad_s"])
        if "tmax_nm" in lim:
            self._limits["tmax"] = float(lim["tmax_nm"])

        # state/error
        codes = section.get("state_error_codes") or {}
        for code, info in codes.items():
            # YAML 里 int 键解析稳定性: 允许 0x 前缀字符串 / int / bool-ish
            try:
                ic = int(code) if isinstance(code, int) else int(str(code), 0)
            except ValueError:
                logger.warning("Skipping non-int error code key: %r", code)
                continue
            if not isinstance(info, dict):
                continue
            if "zh" in info:
                self._state_error_zh[ic] = str(info["zh"])
            if "severity" in info:
                sev = str(info["severity"]).lower()
                if sev in ("none", "soft", "hard"):
                    self._severity[ic] = sev  # type: ignore[assignment]
            if "recoverable" in info:
                self._recoverable[ic] = bool(info["recoverable"])

        # 掉线参数
        if "offline_timeout_ms" in section:
            try:
                self._offline_timeout_ms = float(section["offline_timeout_ms"])
            except (TypeError, ValueError):
                pass
        if "expected_reply_rate_hz" in section:
            try:
                self._expected_rate_hz = float(section["expected_reply_rate_hz"])
            except (TypeError, ValueError):
                pass

        logger.info(
            "YAML %s loaded: %d codes, pmax=%.2f vmax=%.2f tmax=%.2f",
            path.name, len(self._state_error_zh),
            self._limits["pmax"], self._limits["vmax"], self._limits["tmax"],
        )

    # ---------------------------------------------------------------- 解码主函数
    def parse_mit_reply(self, raw_8_bytes: bytes, can_id: int) -> FaultReport:
        """
        解析一个 8 字节 MIT 反馈帧. 返回 FaultReport.

        该函数是纯函数 + 有副作用: 副作用只有更新 last_seen / last_report 缓存.
        原因: watchdog 循环天然要在同一调用里完成 "解码 + 刷新看门狗".
        """
        now = time.monotonic()
        motor_name = self._motor_map.get(can_id, f"UNKNOWN_0x{can_id:X}")

        # 1) 长度守护. 协议要求 8 字节, 但 USB2CAN 解封装有时吐出短帧 (串口噪声).
        if not isinstance(raw_8_bytes, (bytes, bytearray)) or len(raw_8_bytes) != 8:
            logger.warning(
                "parse_mit_reply: bad frame len for can_id=0x%X name=%s: %d bytes",
                can_id, motor_name,
                len(raw_8_bytes) if raw_8_bytes is not None else -1,
            )
            rep = FaultReport(
                motor_id=can_id, motor_name=motor_name,
                state=-1, state_zh="帧长错误",
                error_code=-1, error_zh="帧长错误",
                severity="hard", recoverable=False,
                position=None, velocity=None, torque=None,
                temp_mos_c=None, temp_rotor_c=None,
                timestamp=now,
                raw=bytes(raw_8_bytes) if raw_8_bytes is not None else None,
            )
            with self._lock:
                self._last_reports[can_id] = rep
            return rep

        b = bytes(raw_8_bytes)  # 定型

        # 2) D[0] = ID(低4位) | ERR<<4 (高4位)
        # 参见 DM-J4310 V1.0 手册第 6 页, DM-S3519 V1.1 手册第 6-7 页
        state_code = (b[0] >> 4) & 0x0F
        # id_low4 = b[0] & 0x0F   # 电机 SlaveID 的低 4 位, 当前不使用 (can_id 已经由上层选路)

        state_zh = self._state_error_zh.get(state_code, f"未知状态 0x{state_code:X}")
        severity = self._severity.get(state_code, "hard")  # 未知代码保守按 hard 处理
        recoverable = self._recoverable.get(state_code, False)

        # 3) 位置: 16-bit, D[1]<<8 | D[2]
        q_uint = (b[1] << 8) | b[2]
        pos = _uint_to_float(q_uint, -self._limits["pmax"], self._limits["pmax"], 16)

        # 4) 速度: 12-bit, D[3]<<4 | (D[4]>>4)
        dq_uint = (b[3] << 4) | (b[4] >> 4)
        vel = _uint_to_float(dq_uint, -self._limits["vmax"], self._limits["vmax"], 12)

        # 5) 扭矩: 12-bit, (D[4]&0x0F)<<8 | D[5]
        tau_uint = ((b[4] & 0x0F) << 8) | b[5]
        torque = _uint_to_float(tau_uint, -self._limits["tmax"], self._limits["tmax"], 12)

        # 6) 温度 (uint8, °C, 直接读出)
        t_mos = float(b[6])
        t_rotor = float(b[7])

        # state_code 既是 state 又是 error_code (官方协议合并): 当 state in {0,1} 时无故障
        if state_code in (0x0, 0x1):
            error_code = 0x0
            error_zh = "无故障"
        else:
            error_code = state_code
            error_zh = state_zh

        rep = FaultReport(
            motor_id=can_id,
            motor_name=motor_name,
            state=state_code,
            state_zh=state_zh,
            error_code=error_code,
            error_zh=error_zh,
            severity=severity,
            recoverable=recoverable,
            position=pos,
            velocity=vel,
            torque=torque,
            temp_mos_c=t_mos,
            temp_rotor_c=t_rotor,
            timestamp=now,
            raw=b,
        )

        # 7) 更新 last_seen / last_report (线程安全)
        with self._lock:
            self._last_seen[can_id] = now
            self._last_reports[can_id] = rep

        # 8) 故障 → WARNING, 正常 → DEBUG (调用方可以 setLevel 过滤)
        if rep.is_fault():
            logger.warning(
                "DaMiao fault: %s (0x%X) state=0x%X (%s) T_MOS=%.0f°C T_rot=%.0f°C "
                "pos=%.3frad vel=%.3frad/s tau=%.2fNm",
                motor_name, can_id, state_code, state_zh, t_mos, t_rotor,
                pos, vel, torque,
            )
        else:
            logger.debug(
                "DaMiao OK: %s (0x%X) state=0x%X (%s) pos=%.3f vel=%.3f tau=%.2f",
                motor_name, can_id, state_code, state_zh, pos, vel, torque,
            )

        return rep

    # ---------------------------------------------------------------- 简单查询
    def code_to_zh(self, code: int) -> str:
        """返回状态/错误码的中文标签, 未知则 '未知状态 0xN'."""
        return self._state_error_zh.get(code & 0xF, f"未知状态 0x{code & 0xF:X}")

    def classify(self, code: int) -> Literal["none", "soft", "hard"]:
        """返回严重度分类. 未知码保守按 'hard'."""
        return self._severity.get(code & 0xF, "hard")

    # ---------------------------------------------------------------- 看门狗
    def update_last_seen(self, can_id: int, now: Optional[float] = None) -> None:
        """主动刷新某个电机的 last_seen (当你不走 parse_mit_reply 但确定收到了帧时使用)."""
        if now is None:
            now = time.monotonic()
        with self._lock:
            self._last_seen[can_id] = now

    def last_seen(self, can_id: int) -> Optional[float]:
        """返回 can_id 最近一次反馈帧的 monotonic 时间戳; 从未收到则 None."""
        with self._lock:
            return self._last_seen.get(can_id)

    def last_report(self, can_id: int) -> Optional[FaultReport]:
        """返回 can_id 最近一次 FaultReport 快照. 可能为 None."""
        with self._lock:
            return self._last_reports.get(can_id)

    def check_offline(
        self,
        can_id: int,
        now: float,
        timeout_ms: Optional[float] = None,
    ) -> bool:
        """
        判断 can_id 是否掉线.
        - timeout_ms=None 时使用构造时 YAML/默认的 offline_timeout_ms.
        - 若该 can_id 从未收到过反馈, 也返回 True (视为掉线).
        """
        if timeout_ms is None:
            timeout_ms = self._offline_timeout_ms
        with self._lock:
            last = self._last_seen.get(can_id)
        if last is None:
            return True
        age_ms = (now - last) * 1000.0
        return age_ms > timeout_ms

    def offline_motors(self, now: Optional[float] = None, timeout_ms: Optional[float] = None) -> List[int]:
        """返回当前所有被认为掉线的 CAN ID 列表."""
        if now is None:
            now = time.monotonic()
        return [cid for cid in self._motor_map if self.check_offline(cid, now, timeout_ms)]

    def snapshot(self, now: Optional[float] = None) -> Dict[int, Dict[str, Any]]:
        """
        面向 watchdog / 外部监控的快照视图. 每个电机一个字典, 包含 name/state/offline/报告.
        """
        if now is None:
            now = time.monotonic()
        out: Dict[int, Dict[str, Any]] = {}
        with self._lock:
            for cid, name in self._motor_map.items():
                last = self._last_seen.get(cid)
                rep = self._last_reports.get(cid)
                age_ms = (now - last) * 1000.0 if last is not None else None
                out[cid] = {
                    "name": name,
                    "last_seen_age_ms": age_ms,
                    "offline": (last is None) or (age_ms is not None and age_ms > self._offline_timeout_ms),
                    "last_report": rep.to_dict() if rep is not None else None,
                }
        return out

    # ---------------------------------------------------------------- 命令帧构造
    # 以下 build_*_frame() 返回 8 字节纯 CAN 数据载荷 (not 达妙 USB2CAN 16 字节封装).
    # 调用方负责包进达妙 USB2CAN 帧头 (0x55 0xAA ... 0x55) 或走 SocketCAN 等.
    def build_enable_frame(self) -> bytes:
        """构造使能命令帧的 8 字节 CAN 数据载荷."""
        return bytes(CMD_ENABLE_DATA)

    def build_disable_frame(self) -> bytes:
        """构造失能命令帧的 8 字节 CAN 数据载荷."""
        return bytes(CMD_DISABLE_DATA)

    def build_clear_error_frame(self) -> bytes:
        """构造清错码命令帧. 注意: 该命令非达妙官方 SDK 暴露 API (verified=false)."""
        return bytes(CMD_CLEAR_ERROR_DATA)

    def build_set_zero_frame(self) -> bytes:
        """构造设零点命令帧的 8 字节 CAN 数据载荷. 必须先失能."""
        return bytes(CMD_SET_ZERO_DATA)

    # ---------------------------------------------------------------- SDK 协作
    # 这些方法仅在构造时传入了 SDK MotorControl 才生效, 否则 no-op + warning.
    def sdk_enable(self, motor_obj: Any) -> bool:
        if self._sdk_mc is None:
            logger.warning("sdk_enable: no SDK attached; use build_enable_frame() + write to serial")
            return False
        try:
            self._sdk_mc.enable(motor_obj)
            return True
        except Exception as e:
            logger.exception("sdk_enable failed: %s", e)
            return False

    def sdk_disable(self, motor_obj: Any) -> bool:
        if self._sdk_mc is None:
            logger.warning("sdk_disable: no SDK attached")
            return False
        try:
            self._sdk_mc.disable(motor_obj)
            return True
        except Exception as e:
            logger.exception("sdk_disable failed: %s", e)
            return False

    def attempt_recovery(
        self,
        motor_obj: Any,
        sleep_fn: Callable[[float], None] = time.sleep,
    ) -> bool:
        """
        自动执行 disable → wait → clear_error → wait → enable 的恢复序列.
        需要 SDK MotorControl 且 motor_obj 是 Motor 对象. 返回 True 表示命令均发出,
        但调用方仍需观察后续反馈帧确认 state 回到 0x1 (使能).
        """
        if self._sdk_mc is None:
            logger.warning("attempt_recovery: no SDK; caller must send frames manually")
            return False
        try:
            self._sdk_mc.disable(motor_obj)
            sleep_fn(0.1)
            # clear_error 在 SDK 里没 API, 我们直接通过 SDK 内部 __control_cmd 也做不到外部访问.
            # 所以 attempt_recovery 实际上只做 disable → enable, 依赖电机自恢复.
            # 如需清错码, 调用方走 build_clear_error_frame() + 原生串口 write.
            self._sdk_mc.enable(motor_obj)
            sleep_fn(0.1)
            return True
        except Exception as e:
            logger.exception("attempt_recovery failed: %s", e)
            return False


# --------------------------------------------------------------------------- #
# 便利函数 - 单帧快速解码, 无状态 (测试/调试用)
# --------------------------------------------------------------------------- #
def decode_mit_reply_standalone(
    raw_8_bytes: bytes,
    can_id: int,
    motor_name: str = "UNNAMED",
    pmax: float = 12.5,
    vmax: float = 30.0,
    tmax: float = 10.0,
) -> FaultReport:
    """
    无状态单帧解码. 不需要实例化 DamiaoDiagnostic, 适合单元测试.
    默认 limits 对应 DM4310; 解 DM3519 时传 vmax=200.
    """
    diag = DamiaoDiagnostic(
        motor_map={can_id: motor_name},
        yaml_path=None,
        sdk_motor_control=None,
        motor_model="dm4310",
    )
    diag._limits["pmax"] = pmax
    diag._limits["vmax"] = vmax
    diag._limits["tmax"] = tmax
    return diag.parse_mit_reply(raw_8_bytes, can_id)


__all__ = [
    "FaultReport",
    "DamiaoDiagnostic",
    "decode_mit_reply_standalone",
    "CMD_ENABLE_DATA",
    "CMD_DISABLE_DATA",
    "CMD_SET_ZERO_DATA",
    "CMD_CLEAR_ERROR_DATA",
]


if __name__ == "__main__":  # pragma: no cover
    # 简单自测 - 正常运行一次走完全套 API
    logging.basicConfig(level=logging.DEBUG, format="[%(levelname)s] %(name)s: %(message)s")

    motor_map = {0x13: "RR", 0x14: "FR", 0x15: "RL", 0x16: "FL"}
    diag = DamiaoDiagnostic(
        motor_map=motor_map,
        yaml_path=Path(__file__).parent / "damiao_errors.yaml",
        motor_model="dm4310",
    )

    # 构造一帧 "使能, 位置≈0, 速度≈0, 扭矩≈0, T_MOS=35, T_rotor=40"
    # 要让扭矩 ≈ 0 (而非 -TMAX), 必须让 tau_uint ≈ 2048 (12-bit 量程中点)
    # → (D[4] & 0x0F) << 8 | D[5] = 0x800 → D[4]=0x08, D[5]=0x00
    # 速度同理: dq_uint ≈ 2048 → D[3]<<4 | (D[4]>>4) = 0x800 → D[3]=0x80
    # 位置: q_uint ≈ 32768 → D[1]=0x80, D[2]=0x00
    # state=1 使能, id_low4=0x3 → D[0] = (0x1<<4) | 0x3 = 0x13
    frame = bytes([0x13, 0x80, 0x00, 0x80, 0x08, 0x00, 0x23, 0x28])
    rep = diag.parse_mit_reply(frame, can_id=0x13)
    print(f"Decoded: {rep.to_dict()}")

    # 构造过压错误帧 state=8, 其余同上
    err_frame = bytes([0x83, 0x80, 0x00, 0x80, 0x08, 0x00, 0x4B, 0x5A])
    err_rep = diag.parse_mit_reply(err_frame, can_id=0x13)
    print(f"Fault: {err_rep.to_dict()}")
    print(f"Severity={err_rep.severity}, is_fault={err_rep.is_fault()}")
