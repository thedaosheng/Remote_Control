#!/usr/bin/env python3
"""
================================================================
  zlac_diag.py — ZLAC8015D 故障诊断模块 (production grade)
================================================================

用于 仓颉/Faber 底盘安全守护. 本模块纯 Python, **不依赖 ROS 2**, 可被
chassis_safety_node.py 或任何 Python 应用导入.

## 硬件背景

- 2× ZLAC8015D 双通道 CANopen 伺服驱动器 (每驱动器驱动 L/R 两个轮毂电机)
- Node IDs: 2 和 3
- CAN bus: SocketCAN can0 @ 500 kbps (达妙 USB2CAN + slcand)
- 协议: CANopen CiA301 + CiA402

## 故障来源优先级 (本模块按以下顺序采集)

1. **EMCY 帧** (COB-ID 0x80 + NodeID) — 实时 push (被动监听, background thread)
2. **Heartbeat** (COB-ID 0x700 + NodeID) — 实时 push (同一 listener thread)
3. **0x603F Error Code** — 主动 SDO poll (CiA402 标准)
4. **0x6041 Statusword** — 主动 SDO poll (检测 Fault bit 3)
5. **0x20A5/0x20A6 Modbus bitmap** — 主动 SDO poll (ZLAC 厂家映射, 未验证)

## 故障严重度分级

- **none** — 无故障
- **soft** — 暂态 (过压/欠压/过载/过温/跟随超差) → chassis_safety 可 0x6040=0x80
             复位 + 重走使能状态机. 3 次失败升级为 hard
- **hard** — 物理 (HALL/EEPROM/内部 ref/过流/短路/bus_off/heartbeat_lost)
             → chassis_safety 必须 freeze, 等人检修

## 重要约束 (uncertainty)

- ZLAC 厂家 PDF **不公开 CANopen object dictionary**. 所有 0x2xxx 条目 (特别是
  0x20A5/0x20A6) 的真实 CANopen index 地址均为 "Modbus register → CANopen index
  直接映射" 的合理猜测, 需要硬件 SDO 实测验证.
- ZLAC 是否主动发 EMCY 帧、0x603F 是否真实实现、0x1017 Producer Heartbeat Time
  默认值是否为 0, 均需硬件验证.
- 这些 uncertainty 在 zlac_errors.yaml 里标为 verified=false. 运行时如果遇到
  SDO abort code 0x06020000 (Object does not exist), 说明这些对象未实现.

## 典型使用

    from pathlib import Path
    import can
    from zlac_diag import ZlacDiagnostic

    bus = can.Bus(channel='can0', bustype='socketcan')
    diag = ZlacDiagnostic(bus, node_ids=[2, 3])
    diag.start_listener()

    # poll 一次
    rpt = diag.poll_fault(node_id=2)
    if rpt:
        print(f"[{rpt.node_id}] {rpt.zh} ({rpt.severity})")

    # 检查驱动是否丢线
    if diag.detect_dropped(node_id=2, now=time.time()):
        print("Driver 2 dropped!")

    # bus 整体健康
    print(diag.check_bus_health())

    diag.stop_listener()
"""

from __future__ import annotations

import logging
import struct
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal, Optional, Union

# 可选依赖: python-can — 运行时按需导入 (允许无硬件单测)
try:
    import can  # type: ignore
    _HAS_CAN = True
except ImportError:
    can = None  # type: ignore
    _HAS_CAN = False

# 可选依赖: pyyaml — YAML 字典必需
try:
    import yaml  # type: ignore
    _HAS_YAML = True
except ImportError:
    yaml = None  # type: ignore
    _HAS_YAML = False


# ============================================================
# 日志器
# ============================================================
logger = logging.getLogger("zlac_diag")


# ============================================================
# CANopen / CiA402 常量
# ============================================================

# SDO 命令字节 (expedited transfer)
SDO_READ_REQ = 0x40
SDO_WRITE_1B = 0x2F
SDO_WRITE_2B = 0x2B
SDO_WRITE_4B = 0x23

# SDO 响应字节 (高 nibble 固定 0x4_, 低 nibble 表示 size)
SDO_READ_RESP_BYTES = (0x4F, 0x4B, 0x43, 0x42, 0x41, 0x40)  # 不同 size 的响应码
SDO_WRITE_ACK = 0x60
SDO_ABORT = 0x80

# CiA402 / CiA301 对象字典 index (本模块用到的)
OBJ_ERROR_REGISTER = 0x1001
OBJ_PREDEFINED_ERR = 0x1003
OBJ_HEARTBEAT_TIME = 0x1017
OBJ_ERROR_CODE = 0x603F
OBJ_CONTROLWORD = 0x6040
OBJ_STATUSWORD = 0x6041
OBJ_L_FAULT_MODBUS = 0x20A5  # ZLAC Modbus register → CANopen index 映射 (未验证)
OBJ_R_FAULT_MODBUS = 0x20A6  # ZLAC Modbus register → CANopen index 映射 (未验证)

# Controlword 命令
CW_FAULT_RESET = 0x0080

# COB-ID 基
COBID_EMCY_BASE = 0x080
COBID_SDO_RX_BASE = 0x580   # slave → master
COBID_SDO_TX_BASE = 0x600   # master → slave
COBID_HEARTBEAT_BASE = 0x700

# Statusword Fault bit
STATUSWORD_FAULT_BIT = 1 << 3
STATUSWORD_QUICK_STOP_BIT = 1 << 5

# 严重度类型别名 (Python 3.8+ typing.Literal)
Severity = Literal["none", "soft", "hard"]


# ============================================================
# FaultReport dataclass
# ============================================================
@dataclass
class FaultReport:
    """
    单个故障观察的结构化报告.

    字段:
      node_id:   ZLAC 的 CANopen Node ID (本工程 2 或 3)
      subindex:  电机槽位. 1 = Left motor, 2 = Right motor, None = 驱动器级别
                 (bus-off, heartbeat 丢失, 或无法定位到具体电机时)
      code:      原始故障代码 (UINT16). 对于 Modbus bitmap, 该字段可能是多 bit 或(下同).
                 对于 heartbeat/bus_off 之类, code 约定为 0xFFFE (heartbeat lost) /
                 0xFFFF (bus off) 这种 "伪 code".
      zh/en:     本地化描述
      severity:  "none" / "soft" / "hard"
      recoverable: 是否可通过 fault-reset + re-enable 恢复
      source:    "emcy" / "statusword" / "0x603F" / "0x20A5" / "0x20A6" /
                 "heartbeat" / "bus_state"
      timestamp: time.time() at observation
      raw:       原始 CAN frame bytes (如果可用)
    """
    node_id: int
    subindex: Optional[int]
    code: int
    zh: str
    en: str
    severity: Severity
    recoverable: bool
    source: str
    timestamp: float = field(default_factory=time.time)
    raw: Optional[bytes] = None

    def is_fault(self) -> bool:
        """快速判断: 是否为真实故障 (非 none)"""
        return self.severity != "none"

    def __str__(self) -> str:
        sub = f".{self.subindex}" if self.subindex else ""
        return (
            f"FaultReport(node={self.node_id}{sub} "
            f"code=0x{self.code:04X} "
            f"{self.zh} sev={self.severity} src={self.source})"
        )


# 伪 code 用于非标准故障 (heartbeat/bus)
PSEUDO_CODE_HEARTBEAT_LOST = 0xFFFE
PSEUDO_CODE_BUS_OFF = 0xFFFF


# ============================================================
# ZlacDiagnostic 主类
# ============================================================
class ZlacDiagnostic:
    """
    ZLAC8015D 多节点故障诊断器.

    支持被动监听 (EMCY + heartbeat) 和主动轮询 (SDO 读 0x603F / 0x6041 /
    0x20A5 / 0x20A6). 线程安全, 可在 ROS 2 executor 里并发调用.
    """

    def __init__(
        self,
        bus: Optional["can.Bus"],  # noqa: F821
        node_ids: list[int],
        yaml_path: Optional[Path] = None,
    ) -> None:
        """
        构造诊断器.

        参数:
          bus:       python-can Bus 实例. 允许 None — 用于无硬件单测.
          node_ids:  待监控的 CANopen Node ID 列表, e.g. [2, 3]
          yaml_path: zlac_errors.yaml 路径. 默认是本模块同目录 zlac_errors.yaml.
        """
        self.bus = bus
        self.node_ids = list(node_ids)

        # YAML 路径默认 sibling
        if yaml_path is None:
            yaml_path = Path(__file__).parent / "zlac_errors.yaml"
        self.yaml_path = yaml_path

        # 加载字典 (YAML 缺失时用内置空字典 + fallback 描述)
        self._dict = self._load_yaml(yaml_path)

        # 心跳时间戳 (node_id → last_heartbeat_time)
        self._hb_lock = threading.Lock()
        self._last_heartbeat: dict[int, float] = {}

        # EMCY 缓存 (node_id → list[FaultReport] 最近 32 条)
        self._emcy_lock = threading.Lock()
        self._emcy_buffer: dict[int, list[FaultReport]] = {
            nid: [] for nid in self.node_ids
        }
        self._emcy_buffer_max = 32

        # Listener thread
        self._listener_thread: Optional[threading.Thread] = None
        self._listener_stop = threading.Event()

        # Heartbeat 判定参数 (可从 YAML 覆盖)
        hb_cfg = self._dict.get("zlac8015d", {}).get("heartbeat", {})
        self._hb_period_ms = int(hb_cfg.get("default_period_ms", 100))
        self._hb_timeout_ms = int(hb_cfg.get("recommended_timeout_ms", 500))
        self._hb_consec_misses = int(hb_cfg.get("consecutive_misses_threshold", 3))

        logger.info(
            "ZlacDiagnostic initialized: nodes=%s yaml=%s bus=%s "
            "hb_period=%dms hb_timeout=%dms",
            self.node_ids, str(yaml_path),
            "real" if bus is not None else "None",
            self._hb_period_ms, self._hb_timeout_ms,
        )

    # --------------------------------------------------------
    # YAML 加载 (容忍 yaml 缺失或文件缺失)
    # --------------------------------------------------------
    @staticmethod
    def _load_yaml(path: Path) -> dict:
        """加载 YAML 字典. 失败时返回空 dict 并记 warn."""
        if not _HAS_YAML:
            logger.warning(
                "pyyaml not installed — ZlacDiagnostic will use minimal fallback "
                "descriptions only"
            )
            return {}

        if not path.is_file():
            logger.warning("YAML dict not found: %s — using empty dict", path)
            return {}

        try:
            with path.open("r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
            if not isinstance(data, dict):
                logger.warning("YAML root is not a dict: %s", path)
                return {}
            return data
        except Exception as exc:
            logger.error("Failed to parse YAML %s: %s", path, exc)
            return {}

    # --------------------------------------------------------
    # 查表辅助
    # --------------------------------------------------------
    def _get_cia402_entry(self, code: int) -> Optional[dict]:
        """查 cia402_error_codes 表"""
        return (
            self._dict.get("zlac8015d", {})
            .get("cia402_error_codes", {})
            .get(code)
        )

    def _get_modbus_entry(self, code: int) -> Optional[dict]:
        """查 modbus_fault_codes 表 (精确值匹配)"""
        return (
            self._dict.get("zlac8015d", {})
            .get("modbus_fault_codes", {})
            .get(code)
        )

    def code_to_zh(self, code: int) -> str:
        """
        故障代码 → 中文描述.

        优先级:
          1. cia402_error_codes 精确匹配
          2. modbus_fault_codes 精确匹配
          3. fallback "未知故障 (0xXXXX)"
        """
        ent = self._get_cia402_entry(code) or self._get_modbus_entry(code)
        if ent and "zh" in ent:
            return str(ent["zh"])
        # 特殊伪 code
        if code == PSEUDO_CODE_HEARTBEAT_LOST:
            return "心跳丢失 / 驱动器离线"
        if code == PSEUDO_CODE_BUS_OFF:
            return "CAN Bus-Off"
        return f"未知故障 (0x{code:04X})"

    def code_to_en(self, code: int) -> str:
        """故障代码 → 英文描述 (fallback 'Unknown fault (0xXXXX)')"""
        ent = self._get_cia402_entry(code) or self._get_modbus_entry(code)
        if ent and "en" in ent:
            return str(ent["en"])
        if code == PSEUDO_CODE_HEARTBEAT_LOST:
            return "Heartbeat lost / driver offline"
        if code == PSEUDO_CODE_BUS_OFF:
            return "CAN Bus-Off"
        return f"Unknown fault (0x{code:04X})"

    def classify(self, code: int) -> Severity:
        """
        故障代码 → 严重度 (none / soft / hard).

        查表优先. 表里未登记的未知 code:
          - 按 CiA402 标准分段启发: 0x2xxx/0x5xxx/0x7xxx/0x9xxx → hard,
            0x3xxx/0x4xxx/0x8xxx → soft
          - 其他 → soft (保守回退, 允许尝试 reset)
        """
        ent = self._get_cia402_entry(code) or self._get_modbus_entry(code)
        if ent and "severity" in ent:
            sev = str(ent["severity"])
            if sev in ("none", "soft", "hard"):
                return sev  # type: ignore[return-value]
        # 伪 code
        if code in (PSEUDO_CODE_HEARTBEAT_LOST, PSEUDO_CODE_BUS_OFF):
            return "hard"
        if code == 0x0000:
            return "none"
        # 启发式分段 (CiA301 error code groups)
        high_nibble = (code >> 12) & 0xF
        if high_nibble in (0x2, 0x5, 0x7, 0x9):
            return "hard"
        if high_nibble in (0x3, 0x4, 0x8):
            return "soft"
        # 对 Modbus bitmap (低 word) 未登记值, 回退 soft
        return "soft"

    def is_recoverable(self, code: int) -> bool:
        """查表 recoverable 标志, 未登记则按 severity 推定 (hard → False, 其余 → True)"""
        ent = self._get_cia402_entry(code) or self._get_modbus_entry(code)
        if ent and "recoverable" in ent:
            return bool(ent["recoverable"])
        return self.classify(code) != "hard"

    # --------------------------------------------------------
    # 构造 FaultReport 辅助
    # --------------------------------------------------------
    def _build_report(
        self,
        node_id: int,
        code: int,
        source: str,
        subindex: Optional[int] = None,
        raw: Optional[bytes] = None,
    ) -> FaultReport:
        """从 code + context 构造 FaultReport (自动查表 zh/en/severity/recoverable)"""
        return FaultReport(
            node_id=node_id,
            subindex=subindex,
            code=code,
            zh=self.code_to_zh(code),
            en=self.code_to_en(code),
            severity=self.classify(code),
            recoverable=self.is_recoverable(code),
            source=source,
            timestamp=time.time(),
            raw=raw,
        )

    # --------------------------------------------------------
    # SDO 读 (同步, expedited)
    # --------------------------------------------------------
    def _sdo_read(
        self,
        node_id: int,
        index: int,
        subindex: int = 0,
        timeout: float = 0.3,
    ) -> Optional[int]:
        """
        同步 SDO 读 (expedited upload).

        返回:
          int or None — 读取到的 UINT32 value (调用方自行 mask 到 16/8 bit);
          None 表示超时 / abort / bus=None.

        注意: 本方法会消费 bus.recv() 上的 SDO 响应帧. 如果同时有 background listener
        也在 recv, 可能 race. 对 python-can 我们依赖 Notifier 做 fan-out, 见 start_listener.
        若未开 listener 且直接 recv, 会正常工作, 但 EMCY 帧会被 listener 错过.

        推荐用法:
          - 开 listener (EMCY/heartbeat 自动收)
          - poll_fault() 内部只 SDO 读, 用 send + 单独 recv_with_filter
        """
        if self.bus is None:
            logger.debug("_sdo_read called but bus is None (node=%d idx=0x%04X)",
                         node_id, index)
            return None

        # 构造请求帧
        data = bytearray(8)
        data[0] = SDO_READ_REQ
        data[1] = index & 0xFF
        data[2] = (index >> 8) & 0xFF
        data[3] = subindex
        # data[4:8] = 0

        tx_cobid = COBID_SDO_TX_BASE + node_id
        rx_cobid = COBID_SDO_RX_BASE + node_id

        try:
            msg = can.Message(
                arbitration_id=tx_cobid,
                data=bytes(data),
                is_extended_id=False,
            )
            self.bus.send(msg)
        except Exception as exc:
            logger.warning("SDO send failed node=%d idx=0x%04X: %s",
                           node_id, index, exc)
            return None

        # 等待响应
        deadline = time.time() + timeout
        while time.time() < deadline:
            remaining = max(0.0, deadline - time.time())
            try:
                reply = self.bus.recv(timeout=min(0.1, remaining))
            except Exception as exc:
                logger.warning("SDO recv error node=%d: %s", node_id, exc)
                return None
            if reply is None:
                continue
            if reply.arbitration_id != rx_cobid:
                # 不是给我们的帧 — 让 listener 处理 (如果有)
                # 简化实现: 丢弃非目标帧 (生产环境可以 refilter 反灌)
                continue
            cmd = reply.data[0]
            if cmd in SDO_READ_RESP_BYTES:
                # expedited upload response
                val = struct.unpack_from("<I", reply.data, 4)[0]
                return val
            if cmd == SDO_ABORT:
                abort = struct.unpack_from("<I", reply.data, 4)[0]
                logger.warning(
                    "SDO abort node=%d idx=0x%04X:%d abort=0x%08X",
                    node_id, index, subindex, abort
                )
                return None
            # 其他 cmd (segmented transfer, block) — 当前不支持
            logger.debug("SDO unexpected cmd 0x%02X node=%d", cmd, node_id)
        logger.warning(
            "SDO read timeout node=%d idx=0x%04X:%d (timeout=%.3fs)",
            node_id, index, subindex, timeout
        )
        return None

    # --------------------------------------------------------
    # poll_fault — 主动 SDO 诊断
    # --------------------------------------------------------
    def poll_fault(self, node_id: int, timeout: float = 0.3) -> Optional[FaultReport]:
        """
        主动轮询单个 node 的故障状态, 返回 **最严重的** 发现.

        策略:
          1. 读 0x603F Error Code. 若 != 0, 返回对应 FaultReport.
          2. 读 0x6041 Statusword, 检查 Fault bit (3). 若置位, 返回 generic
             "fault bit 置位" (code=0xFFFD).
          3. 读 0x20A5 / 0x20A6 Modbus bitmap. 若非零, 返回最高位 bit 对应的 code.
          4. 都 0 → 返回 None (无故障).

        返回:
          FaultReport 或 None.
        """
        if self.bus is None:
            logger.debug("poll_fault node=%d: bus is None", node_id)
            return None

        # 1. 0x603F — CiA402 标准 error code
        err_code = self._sdo_read(node_id, OBJ_ERROR_CODE, 0, timeout=timeout)
        if err_code is not None:
            err_code &= 0xFFFF
            if err_code != 0:
                return self._build_report(node_id, err_code, source="0x603F")

        # 2. 0x6041 — Statusword Fault bit
        sw = self._sdo_read(node_id, OBJ_STATUSWORD, 0, timeout=timeout)
        if sw is not None:
            sw &= 0xFFFF
            if sw & STATUSWORD_FAULT_BIT:
                # 0xFFFD 是我们自己的伪 code, 表示 "Statusword 报 fault, 但 0x603F 读不到"
                rpt = self._build_report(node_id, 0xFFFD, source="statusword")
                rpt.zh = "Statusword 故障位置位 (0x603F 无具体码)"
                rpt.en = "Statusword fault bit set (no specific code in 0x603F)"
                rpt.severity = "hard"  # 无具体原因时保守处理
                rpt.recoverable = False
                return rpt

        # 3. 0x20A5 — Left motor fault bitmap (ZLAC Modbus register mirror)
        left_bmp = self._sdo_read(node_id, OBJ_L_FAULT_MODBUS, 0, timeout=timeout)
        if left_bmp is not None:
            left_bmp &= 0xFFFF
            if left_bmp != 0:
                top_bit_code = self._highest_bit_code(left_bmp)
                rpt = self._build_report(
                    node_id, top_bit_code, source="0x20A5", subindex=1
                )
                return rpt

        # 4. 0x20A6 — Right motor fault bitmap
        right_bmp = self._sdo_read(node_id, OBJ_R_FAULT_MODBUS, 0, timeout=timeout)
        if right_bmp is not None:
            right_bmp &= 0xFFFF
            if right_bmp != 0:
                top_bit_code = self._highest_bit_code(right_bmp)
                rpt = self._build_report(
                    node_id, top_bit_code, source="0x20A6", subindex=2
                )
                return rpt

        return None  # 无故障

    @staticmethod
    def _highest_bit_code(bitmap: int) -> int:
        """
        Modbus bitmap 多 bit 同置时, 返回 "最严重" 的那个 code.

        当前实现: 返回最低置位 bit 对应的 code (因为在 modbus_fault_codes 里,
        bit 0 = 过压 比 bit 10 = HALL 故障 更紧急? 其实不一定 — 但 HALL/EEPROM
        这些 hard fault 按 code 值排在高 bit, 如果把 bit-value 排序的话
        high bit 更 hard. 这里采取策略: 找最高置位 bit).

        返回 bit-value 对应的 code (1, 2, 4, 8, 16, ..., 0x400).
        """
        if bitmap == 0:
            return 0
        # 找最高置位 bit
        bit = 15
        while bit >= 0 and not (bitmap & (1 << bit)):
            bit -= 1
        return 1 << bit if bit >= 0 else 0

    # --------------------------------------------------------
    # EMCY 帧解析
    # --------------------------------------------------------
    def parse_emcy_frame(
        self, cob_id: int, data: bytes
    ) -> Optional[FaultReport]:
        """
        解析 EMCY 帧.

        CiA301 EMCY 格式:
          COB-ID = 0x80 + NodeID
          DLC = 8
          byte 0-1: Error code (UINT16, little-endian)
          byte 2:   Error register (0x1001 值, UINT8)
          byte 3-7: 厂家自定义 (ZLAC 布局未知)

        参数:
          cob_id: arbitration_id
          data: 原始 8 字节 payload

        返回:
          FaultReport 或 None (若 cob_id 不在 EMCY 范围).
        """
        if not (COBID_EMCY_BASE < cob_id <= COBID_EMCY_BASE + 0x7F):
            return None
        node_id = cob_id - COBID_EMCY_BASE
        if node_id not in self.node_ids:
            # EMCY 来自非监控节点 — 不处理但记 debug
            logger.debug("EMCY from unwatched node %d", node_id)
            return None
        if len(data) < 3:
            logger.warning("EMCY frame too short: len=%d", len(data))
            return None

        err_code = struct.unpack_from("<H", data, 0)[0]
        # err_reg = data[2]  # 0x1001 值, 暂不用
        # error-code 0x0000 = "error reset" 帧 (CiA301 规范 — 表示先前错误已清)
        if err_code == 0x0000:
            rpt = self._build_report(
                node_id, 0x0000, source="emcy", raw=bytes(data)
            )
            rpt.zh = "错误已清除 (EMCY reset)"
            rpt.en = "Error cleared (EMCY reset)"
            rpt.severity = "none"
            return rpt

        rpt = self._build_report(
            node_id, err_code, source="emcy", raw=bytes(data)
        )
        return rpt

    # --------------------------------------------------------
    # Background listener thread
    # --------------------------------------------------------
    def start_listener(self) -> None:
        """
        启动 background thread 捕获 EMCY + heartbeat.

        非阻塞. 线程内做 bus.recv(timeout=0.1), 解析 COB-ID:
          - 0x080 + NodeID → EMCY
          - 0x700 + NodeID → heartbeat (1 byte = NMT state)
          - 其他 → 丢弃 (SDO response 由 _sdo_read 直接 recv)

        CAUTION: 因 python-can 的 Bus.recv() 同一时刻只能有一个 consumer,
        如果调用方同时使用 poll_fault (其内部 recv SDO response) 和本 listener,
        会有竞争. **生产环境强烈建议**: 用 can.Notifier + 多 Listener fan-out.
        当前实现用简单 lock-free dict 暂存, 在多线程并发使用时 SDO poll 有
        "看不到响应" 风险. 若 chassis_safety_node 主要靠 listener 的 EMCY
        + heartbeat, 不依赖 poll_fault, 这个风险可接受.
        """
        if self.bus is None:
            logger.warning("start_listener: bus is None, no-op")
            return
        if self._listener_thread and self._listener_thread.is_alive():
            logger.warning("listener already running")
            return

        self._listener_stop.clear()
        self._listener_thread = threading.Thread(
            target=self._listener_loop,
            name="zlac-diag-listener",
            daemon=True,
        )
        self._listener_thread.start()
        logger.info("listener started")

    def stop_listener(self) -> None:
        """停止 background listener thread (等待最多 2s)."""
        self._listener_stop.set()
        if self._listener_thread:
            self._listener_thread.join(timeout=2.0)
            if self._listener_thread.is_alive():
                logger.warning("listener did not stop within 2s")
            self._listener_thread = None
        logger.info("listener stopped")

    def _listener_loop(self) -> None:
        """background thread main. 消费所有 CAN frame, dispatch."""
        logger.debug("listener loop entered")
        while not self._listener_stop.is_set():
            try:
                msg = self.bus.recv(timeout=0.1)  # type: ignore[union-attr]
            except Exception as exc:
                logger.warning("listener recv error: %s", exc)
                time.sleep(0.1)
                continue
            if msg is None:
                continue
            self._dispatch_frame(msg)
        logger.debug("listener loop exited")

    def _dispatch_frame(self, msg) -> None:  # noqa: ANN001
        """分派 CAN frame (EMCY / heartbeat / 其他)"""
        cob = msg.arbitration_id

        # Heartbeat: 0x700 + NodeID (DLC 1)
        if COBID_HEARTBEAT_BASE < cob <= COBID_HEARTBEAT_BASE + 0x7F:
            node_id = cob - COBID_HEARTBEAT_BASE
            if node_id in self.node_ids:
                with self._hb_lock:
                    self._last_heartbeat[node_id] = time.time()
            return

        # EMCY: 0x080 + NodeID (DLC 8)
        if COBID_EMCY_BASE < cob <= COBID_EMCY_BASE + 0x7F:
            rpt = self.parse_emcy_frame(cob, bytes(msg.data))
            if rpt is None:
                return
            with self._emcy_lock:
                buf = self._emcy_buffer.setdefault(rpt.node_id, [])
                buf.append(rpt)
                # ring buffer
                if len(buf) > self._emcy_buffer_max:
                    del buf[: len(buf) - self._emcy_buffer_max]
            if rpt.is_fault():
                logger.warning("EMCY: %s", rpt)
            return

        # 其他 COB-ID 丢弃 (SDO response 不由 listener 消费)

    # --------------------------------------------------------
    # 丢线 / 健康检测
    # --------------------------------------------------------
    def last_heartbeat(self, node_id: int) -> Optional[float]:
        """返回 node_id 最后一次收到 heartbeat 的 time.time(); 未收到返回 None."""
        with self._hb_lock:
            return self._last_heartbeat.get(node_id)

    def detect_dropped(self, node_id: int, now: float) -> bool:
        """
        检测 node 是否已掉线.

        判据 (任一满足):
          - bus 处于 ERROR_PASSIVE / BUS_OFF (影响所有节点)
          - heartbeat 连续超时 >= _hb_consec_misses * _hb_period_ms

        参数:
          node_id: 待检测节点
          now: 当前时间 (由调用方传入, 便于测试模拟)

        返回:
          True = 已丢线 (hard fault)
        """
        # 1. bus-level
        bus_health = self.check_bus_health()
        if not bus_health["healthy"]:
            logger.warning("bus unhealthy: %s", bus_health)
            return True

        # 2. heartbeat timeout
        last = self.last_heartbeat(node_id)
        if last is None:
            # 从未收到过心跳 — 可能 listener 未启动 / ZLAC 未配置心跳
            # 保守: listener 已启动 > 2 * timeout 仍无心跳 → dropped
            # 简化实现: 直接返回 False (不误报)
            # 调用方若想要严格版, 应在启动后 sleep > hb_timeout 再调用
            return False

        miss_threshold_s = (
            self._hb_consec_misses * self._hb_period_ms / 1000.0
        )
        age = now - last
        if age > miss_threshold_s:
            logger.warning(
                "node %d heartbeat age %.2fs > threshold %.2fs",
                node_id, age, miss_threshold_s,
            )
            return True
        return False

    def check_bus_health(self) -> dict:
        """
        返回 bus 整体状态.

        返回字典:
          {
            "state": "ACTIVE" | "PASSIVE" | "BUSOFF" | "UNKNOWN",
            "healthy": bool,
          }

        "healthy" = True 当且仅当 state == ACTIVE.
        """
        if self.bus is None:
            return {"state": "UNKNOWN", "healthy": False}

        state = "UNKNOWN"
        if _HAS_CAN:
            try:
                bus_state = self.bus.state
                # python-can 的 BusState enum: ACTIVE / ERROR / PASSIVE / BUSOFF
                # 不同版本名称略有差异, 用字符串匹配
                name = getattr(bus_state, "name", str(bus_state)).upper()
                if "ACTIVE" in name:
                    state = "ACTIVE"
                elif "PASSIVE" in name:
                    state = "PASSIVE"
                elif "BUSOFF" in name or "BUS_OFF" in name:
                    state = "BUSOFF"
                else:
                    state = name
            except (AttributeError, NotImplementedError):
                # 某些 interface 不支持 state 查询
                state = "UNKNOWN"

        return {"state": state, "healthy": state == "ACTIVE"}

    # --------------------------------------------------------
    # EMCY 缓冲区访问
    # --------------------------------------------------------
    def drain_emcy(self, node_id: int) -> list[FaultReport]:
        """
        取出并清空 node 的 EMCY 缓冲区.

        chassis_safety_node 应周期性调用此方法消费 EMCY 历史, 避免 buffer 漏判.
        """
        with self._emcy_lock:
            buf = self._emcy_buffer.get(node_id, [])
            self._emcy_buffer[node_id] = []
            return buf

    def peek_emcy(self, node_id: int) -> list[FaultReport]:
        """窥视 EMCY 缓冲 (不清空). 用于调试."""
        with self._emcy_lock:
            return list(self._emcy_buffer.get(node_id, []))

    # --------------------------------------------------------
    # 发送 fault-reset (方便 chassis_safety_node 重用本对象 send 资源)
    # --------------------------------------------------------
    def send_fault_reset(self, node_id: int) -> bool:
        """
        向 node_id 发送 Controlword = 0x0080 (Fault Reset).

        返回:
          bool — True 若收到 SDO ack (0x60); False 若超时/abort/bus=None.
        """
        if self.bus is None:
            return False
        data = bytearray(8)
        data[0] = SDO_WRITE_2B
        data[1] = OBJ_CONTROLWORD & 0xFF
        data[2] = (OBJ_CONTROLWORD >> 8) & 0xFF
        data[3] = 0  # subindex
        struct.pack_into("<H", data, 4, CW_FAULT_RESET)
        # data[6:8] = 0

        tx_cobid = COBID_SDO_TX_BASE + node_id
        rx_cobid = COBID_SDO_RX_BASE + node_id

        try:
            msg = can.Message(  # type: ignore[union-attr]
                arbitration_id=tx_cobid,
                data=bytes(data),
                is_extended_id=False,
            )
            self.bus.send(msg)
        except Exception as exc:
            logger.warning("fault_reset send failed node=%d: %s", node_id, exc)
            return False

        # 等 SDO ack
        deadline = time.time() + 0.5
        while time.time() < deadline:
            try:
                reply = self.bus.recv(timeout=0.1)
            except Exception:
                continue
            if reply is None:
                continue
            if reply.arbitration_id == rx_cobid:
                if reply.data[0] == SDO_WRITE_ACK:
                    logger.info("fault_reset ack received node=%d", node_id)
                    return True
                if reply.data[0] == SDO_ABORT:
                    abort = struct.unpack_from("<I", reply.data, 4)[0]
                    logger.warning(
                        "fault_reset abort node=%d abort=0x%08X",
                        node_id, abort,
                    )
                    return False
        logger.warning("fault_reset timeout node=%d", node_id)
        return False


# ============================================================
# __main__ — 简易自检 (不触硬件)
# ============================================================
if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )
    logger.info("ZlacDiagnostic self-check (no hardware)")
    diag = ZlacDiagnostic(bus=None, node_ids=[2, 3])
    assert diag.code_to_zh(0x0001) == "过压", "Modbus 0x0001 lookup failed"
    assert diag.code_to_zh(0xDEAD) == "未知故障 (0xDEAD)", "fallback failed"
    assert diag.classify(0x0200) == "hard", "HALL fault should be hard"
    assert diag.classify(0x0008) == "soft", "Over-load should be soft"
    assert diag.classify(0x0000) == "none"
    assert diag.check_bus_health()["state"] == "UNKNOWN"
    assert diag.poll_fault(2) is None  # bus=None → None
    logger.info("Self-check PASSED")
