#!/usr/bin/env python3
"""
20260331-cc-delay_perception_test.py
=====================================
力反馈延迟感知实验脚本

实验设计：
  - 在 Touch 笔末端模拟一个"虚拟接触墙"
  - 当笔的 Z 轴位置超过基准点 + 触发阈值时，判定为"接触"
  - 将此时的弹簧反力写入延迟缓冲区
  - 经过当前档位的延迟后，才把力发给笔
  - 依次遍历 10 个延迟档：10s / 20s / ... / 100s
  - 每档持续 25 秒，供操作者反复感受

运行方式：
  GTDD_HOME=/home/rhz/.3dsystems \
  LD_LIBRARY_PATH=/tmp/patched_lib:/tmp/fakelibs:/usr/lib \
  python3 20260331-cc-delay_perception_test.py

按 Ctrl-C 提前退出，或等待全部档位完成后自动退出。
"""

import os
import sys
import time
import signal
import importlib.util
import threading
from collections import deque

import numpy as np

# ── 自动设置 DISPLAY（VNC 环境）────────────────────────────────────────────
if "DISPLAY" not in os.environ:
    os.environ["DISPLAY"] = ":0"

# ============================================================================
# 延迟队列（精确时间戳版）
# ============================================================================

class TimestampDelayQueue:
    """
    基于时间戳的精确延迟队列。

    每个元素携带入队时间戳，只有当 (now - enqueue_time) >= delay_s 时才允许出队。
    delay_s 可在运行时动态修改。
    """

    def __init__(self, delay_s: float = 0.0):
        self.delay_s = delay_s          # 当前延迟（秒），可动态改
        self._buf: deque = deque()      # 元素：(enqueue_timestamp, data)
        self._lock = threading.Lock()

    def push(self, data):
        """推入数据，附带当前时间戳。"""
        with self._lock:
            self._buf.append((time.time(), data))

    def pop(self):
        """
        取出最早的已成熟数据（等待时间 >= delay_s）。
        返回最新的已成熟帧，同时丢弃中间所有已成熟的旧帧。
        若无成熟帧则返回 None。
        """
        now = time.time()
        result = None
        with self._lock:
            while self._buf and (now - self._buf[0][0]) >= self.delay_s:
                _, result = self._buf.popleft()
        return result

    def flush(self):
        """清空队列（切换延迟档位时调用）。"""
        with self._lock:
            self._buf.clear()


# ============================================================================
# 实验控制器
# ============================================================================

class DelayPerceptionTest:
    """
    力反馈延迟感知实验主体。

    实验逻辑（每帧，约 1kHz）：
      1. 读取笔的当前 Z 位置
      2. 若 Z > 基准 Z + contact_threshold_mm → 接触激活，产生弹簧力
      3. 将弹簧力推入延迟队列
      4. 从延迟队列取出成熟的力 → 发给笔
      5. 定期更新控制台状态
    """

    # 延迟档位列表（秒）
    DELAY_STAGES = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
    # 每档持续时间（秒）
    STAGE_DURATION_S = 25.0
    # 虚拟墙弹簧刚度（N/m → 换算到 mm：N/mm * 1000 = N/m）
    # 笔推入 10mm → 反力 = 0.5N
    WALL_STIFFNESS_N_per_mm = 0.05
    # 触发接触的最小穿入深度（mm）
    CONTACT_THRESHOLD_MM = 2.0
    # 发给笔的最大力（N，保护电机）
    MAX_FORCE_N = 2.5
    # 打印间隔（秒）
    PRINT_INTERVAL_S = 0.2

    def __init__(self):
        self._running = False
        self._delay_queue = TimestampDelayQueue(delay_s=0.0)

        # 加载 haptic_driver（复用已有驱动）
        driver_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "20260328-cc-haptic_driver.py"
        )
        spec = importlib.util.spec_from_file_location("haptic_driver", driver_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self._device = module.HapticDevice()

        # 运行时状态
        self._stage_idx = 0          # 当前档位索引
        self._stage_start = 0.0      # 当前档位开始时间
        self._base_z_mm = None       # 笔的 Z 轴基准位置（首帧自动标定）
        self._contact_active = False # 当前是否处于"接触"状态
        self._last_print = 0.0

        # 统计（每档重置）
        self._contact_count = 0      # 本档接触次数
        self._was_in_contact = False # 用于边沿检测

    # ── 初始化 ──────────────────────────────────────────────────────────────

    def init(self):
        """初始化 Touch 设备，打印欢迎信息。"""
        self._device.initialize()
        self._device.start_scheduler()
        self._running = True

        print("\n" + "=" * 60)
        print("  力反馈延迟感知实验")
        print("=" * 60)
        print(f"  延迟档位: {self.DELAY_STAGES} 秒")
        print(f"  每档时长: {self.STAGE_DURATION_S} 秒")
        print(f"  虚拟墙刚度: {self.WALL_STIFFNESS_N_per_mm * 1000:.0f} N/m")
        print(f"  最大反馈力: {self.MAX_FORCE_N} N")
        print("=" * 60)
        print()
        print("  操作说明：")
        print("  1. 笔静置 1-2 秒，系统自动记录基准 Z 位置")
        print("  2. 沿 Z 轴方向（朝向设备底座）推入超过 2mm 触发接触")
        print("  3. 感受延迟后的弹簧反力")
        print("  4. 每档 25 秒后自动切换到下一延迟")
        print("  5. Ctrl+C 随时退出")
        print()

        signal.signal(signal.SIGINT, self._on_sigint)

    # ── 主循环 ───────────────────────────────────────────────────────────────

    def run(self):
        """主循环：1kHz 采样，推入/取出延迟队列，发力。"""
        self._stage_start = time.time()
        self._stage_idx = 0
        self._reset_stage_stats()

        while self._running and self._stage_idx < len(self.DELAY_STAGES):

            loop_start = time.time()

            # ── 1. 读设备状态 ──
            state = self._device.get_state()
            if not state:
                time.sleep(0.001)
                continue

            pos = np.array(state.position)   # [X, Y, Z] mm

            # ── 2. 自动标定基准 Z（头 2 秒平均）──
            if self._base_z_mm is None:
                if not hasattr(self, "_calib_buf"):
                    self._calib_buf = []
                    self._calib_start = time.time()
                self._calib_buf.append(pos[2])
                if time.time() - self._calib_start >= 2.0:
                    self._base_z_mm = float(np.mean(self._calib_buf))
                    print(f"[标定] 基准 Z = {self._base_z_mm:.2f} mm，实验开始！\n")
                    del self._calib_buf, self._calib_start
                time.sleep(0.001)
                continue

            # ── 3. 计算穿入深度 & 弹簧力 ──
            # Touch 坐标系：笔向前（朝底座）推，Z 变负（减小）
            # 用 |delta_z| 判断穿入深度
            delta_z = self._base_z_mm - pos[2]   # 正值 = 往接触方向推
            penetration_mm = max(0.0, delta_z - self.CONTACT_THRESHOLD_MM)
            spring_force_n = min(
                penetration_mm * self.WALL_STIFFNESS_N_per_mm,
                self.MAX_FORCE_N
            )

            # 接触判定（用于统计）
            in_contact = (delta_z > self.CONTACT_THRESHOLD_MM)
            if in_contact and not self._was_in_contact:
                self._contact_count += 1   # 上升沿 → 新一次接触
            self._was_in_contact = in_contact
            self._contact_active = in_contact

            # ── 4. 推入延迟队列（Z 方向的推回力）──
            # Touch Z 轴：反力方向 = +Z（推回用户）
            force_z = spring_force_n if in_contact else 0.0
            self._delay_queue.push(force_z)

            # ── 5. 取出成熟力帧 → 发给笔 ──
            delayed_fz = self._delay_queue.pop()
            if delayed_fz is not None:
                # 只在 Z 方向施力（最简洁，避免轴映射带来的干扰）
                self._device.set_force(0.0, 0.0, float(delayed_fz))
            else:
                self._device.set_force(0.0, 0.0, 0.0)

            # ── 6. 切换档位逻辑 ──
            elapsed = time.time() - self._stage_start
            if elapsed >= self.STAGE_DURATION_S:
                self._next_stage()

            # ── 7. 定期打印状态 ──
            now = time.time()
            if now - self._last_print >= self.PRINT_INTERVAL_S:
                self._print_status(pos, delta_z, spring_force_n,
                                   delayed_fz or 0.0, elapsed)
                self._last_print = now

            # ── 8. 定时（1kHz）──
            sleep_t = 0.001 - (time.time() - loop_start)
            if sleep_t > 0:
                time.sleep(sleep_t)

        self._finish()

    # ── 档位切换 ─────────────────────────────────────────────────────────────

    def _next_stage(self):
        """进入下一延迟档位。"""
        old_delay = self.DELAY_STAGES[self._stage_idx]
        print(f"\n\n{'─'*60}")
        print(f"  档位 {self._stage_idx + 1} 结束  |  延迟 = {old_delay}s  "
              f"|  本档接触 {self._contact_count} 次")
        print(f"{'─'*60}\n")

        self._stage_idx += 1
        if self._stage_idx >= len(self.DELAY_STAGES):
            return

        new_delay = self.DELAY_STAGES[self._stage_idx]
        self._delay_queue.delay_s = new_delay  # 切换延迟
        self._delay_queue.flush()              # 清空旧队列，避免串扰
        self._stage_start = time.time()
        self._reset_stage_stats()

        print(f"  ▶ 档位 {self._stage_idx + 1} / {len(self.DELAY_STAGES)}"
              f"   延迟 = {new_delay}s   剩余 {self.STAGE_DURATION_S:.0f}s\n")

    def _reset_stage_stats(self):
        """重置每档统计数据。"""
        self._contact_count = 0
        self._was_in_contact = False
        # 首档开始时打印提示
        if self._stage_idx < len(self.DELAY_STAGES):
            delay = self.DELAY_STAGES[self._stage_idx]
            self._delay_queue.delay_s = delay
            self._delay_queue.flush()
            print(f"  ▶ 档位 {self._stage_idx + 1} / {len(self.DELAY_STAGES)}"
                  f"   延迟 = {delay}s   持续 {self.STAGE_DURATION_S:.0f}s\n")

    # ── 状态打印 ──────────────────────────────────────────────────────────────

    def _print_status(self, pos, delta_z, raw_force, delayed_force, elapsed):
        """实时覆盖打印当前状态。"""
        stage = self._stage_idx + 1
        total = len(self.DELAY_STAGES)
        delay = self.DELAY_STAGES[self._stage_idx]
        remaining = max(0.0, self.STAGE_DURATION_S - elapsed)

        contact_str = f"【接触 {delta_z:.1f}mm  原始力 {raw_force:.2f}N】" \
                      if self._contact_active else "  无接触            "
        delayed_str = f"延迟输出: {delayed_force:.2f}N" \
                      if abs(delayed_force) > 0.01 else "延迟输出: 0.00N"

        sys.stdout.write(
            f"\r  档{stage}/{total}  延迟={delay:3d}s  "
            f"剩余={remaining:5.1f}s  "
            f"Z={pos[2]:7.2f}mm  {contact_str}  {delayed_str}  "
            f"接触次数={self._contact_count}"
        )
        sys.stdout.flush()

    # ── 结束 ──────────────────────────────────────────────────────────────────

    def _finish(self):
        """实验结束，清零力输出。"""
        print(f"\n\n{'=' * 60}")
        print("  实验结束！全部 10 个延迟档位已完成。")
        print(f"{'=' * 60}\n")
        self._device.set_force(0.0, 0.0, 0.0)
        time.sleep(0.1)
        self._device.stop()

    def _on_sigint(self, sig, frame):
        """Ctrl-C 优雅退出。"""
        print("\n\n[实验] 收到退出信号，正在停止...")
        self._running = False
        self._device.set_force(0.0, 0.0, 0.0)
        time.sleep(0.1)
        self._device.stop()
        sys.exit(0)


# ============================================================================
# 入口
# ============================================================================

if __name__ == "__main__":
    test = DelayPerceptionTest()
    test.init()
    test.run()
