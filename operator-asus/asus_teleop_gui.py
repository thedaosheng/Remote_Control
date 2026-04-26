#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
====================================================================
  asus_teleop_gui — ASUS 上跑的 Tkinter 遥控面板
====================================================================

数据链路 (chassis):
  本脚本 → LiveKit data {"type":"cmd_vel"} → Aliyun SFU
       → Orin cloud_to_ros_bridge → ROS2 /cmd_vel
       → CangJie chassis_controller (swerve IK)
       ← /chassis_state ← Orin bridge ← Aliyun ← 本脚本

数据链路 (lift, PR-4 新增):
  G/H 键 → LiveKit data {"type":"lift_cmd","speed":±X}
       → Orin cloud_to_ros_bridge → ROS2 /lift_cmd (Float32 ∈ [-1,+1])
       → CangJie chassis_controller v3 lift section → DM3519 一拖四

UI:
  - 大字号 (vx / vy / wz / lift 当前值)
  - WASD / QE / GH 键盘 + 点击按钮都可以
    - WASD: 平移   QE: 偏航   G: 抬升上升   H: 抬升下降
  - 连接状态 / 发送计数 / 最近回包 / 往返延时
  - 4 轮 IK 解算实时显示

运行:
  GUI:  DISPLAY=:0 python3 asus_teleop_gui.py
  CLI:  python3 asus_teleop_gui.py --cli        # 无显示自测

依赖:
  pip install --user -i https://pypi.tuna.tsinghua.edu.cn/simple livekit livekit-api

环境变量 (必填; 见 ../.env.example):
  LIVEKIT_API_KEY, LIVEKIT_API_SECRET
"""

import argparse
import asyncio
import json
import os
import queue
import sys
import threading
import time

from livekit import api, rtc


# ---------- LiveKit 凭据 ----------
# Per SECURITY.md: API key/secret env-required, no default fallback (avoid leaking known-public values).
LIVEKIT_URL        = os.environ.get("LIVEKIT_URL",  "ws://39.102.113.104:7880")
LIVEKIT_API_KEY    = os.environ.get("LIVEKIT_API_KEY")
LIVEKIT_API_SECRET = os.environ.get("LIVEKIT_API_SECRET")
LIVEKIT_ROOM       = os.environ.get("LIVEKIT_ROOM", "teleop-room")
LIVEKIT_IDENTITY   = os.environ.get("LK_IDENTITY",  "edward-asus-teleop")
if not (LIVEKIT_API_KEY and LIVEKIT_API_SECRET):
    raise RuntimeError(
        "LIVEKIT_API_KEY and LIVEKIT_API_SECRET env vars required (see ../.env.example)"
    )

# ---------- 速度模式 (press-and-hold) ----------
LIN_HOLD = 0.20      # 按住 W/S/A/D 时的目标线速度 (m/s)
ANG_HOLD = 1.00      # 按住 Q/E 时的目标角速度 (rad/s)
LIN_MAX  = 0.30      # 上限保留 (按钮 / 累加场景用, 当前没用)
ANG_MAX  = 1.20
LIFT_STEP = 0.5      # 按住 G/H 时的归一化 lift 速度幅值 (∈ [-1.0, +1.0]; PR-4 新增)
SEND_HZ  = 20.0      # cmd_vel 发送频率
RELEASE_DEBOUNCE_S = 0.04  # X11 auto-repeat: KeyRelease 后等这么久, 期间无新 Press 才视为真松开

# ---------- 颜色 (贴 user protocol) ----------
C_BG       = '#1e1e2e'
C_FG       = '#cdd6f4'
C_GREEN    = '#a6e3a1'
C_YELLOW   = '#f9e2af'
C_RED      = '#f38ba8'
C_BLUE     = '#89b4fa'
C_PANEL    = '#313244'


# ============================================================
# 共享状态 (GUI 和 LiveKit 线程之间传)
# ============================================================
class SharedState:
    def __init__(self):
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.lift = 0.0                  # PR-4: 归一化 lift 速度 ∈ [-1.0, +1.0] (G/H 键)
        self.seq = 0
        self.sent = 0
        self.recv = 0
        self.connected = False
        self.last_rx_msg = None          # 最新收到的 chassis_state dict
        self.last_rtt_ms = None
        self.last_err = None
        self.lock = threading.Lock()

    def set_held(self, held: set):
        """根据当前按住的键集合, 重新计算 vx/vy/wz/lift (press-and-hold 模型)."""
        with self.lock:
            self.vx = (LIN_HOLD if 'w' in held else 0.0) - (LIN_HOLD if 's' in held else 0.0)
            self.vy = (LIN_HOLD if 'a' in held else 0.0) - (LIN_HOLD if 'd' in held else 0.0)
            self.wz = (ANG_HOLD if 'q' in held else 0.0) - (ANG_HOLD if 'e' in held else 0.0)
            # PR-4: G = 抬升上升, H = 抬升下降; 同按 (g+h) 互相抵消归零
            if 'g' in held and 'h' not in held:
                self.lift = LIFT_STEP
            elif 'h' in held and 'g' not in held:
                self.lift = -LIFT_STEP
            else:
                self.lift = 0.0

    def stop(self):
        """Space 或紧急归零."""
        with self.lock:
            self.vx = self.vy = self.wz = 0.0
            self.lift = 0.0

    def snapshot(self):
        with self.lock:
            return self.vx, self.vy, self.wz, self.lift


# ============================================================
# LiveKit 异步 worker (跑在独立 thread 里)
# ============================================================
class LiveKitWorker:
    """异步 loop 独立线程: 连接 LiveKit + 20Hz 发 cmd_vel + 收 chassis_state"""
    def __init__(self, state: SharedState, inbound: queue.Queue):
        self.state = state
        self.inbound = inbound   # push 事件给 GUI 线程 (connected / disconnected / data)
        self.room = None

    def start(self):
        t = threading.Thread(target=self._run, daemon=True)
        t.start()
        return t

    def _run(self):
        try:
            asyncio.run(self._run_async())
        except Exception as e:
            self.inbound.put(('err', f'{type(e).__name__}: {e}'))

    async def _run_async(self):
        self.room = rtc.Room()

        def on_data(packet: rtc.DataPacket):
            try:
                msg = json.loads(packet.data.decode('utf-8'))
                if msg.get('type') == 'chassis_state':
                    self.state.recv += 1
                    self.state.last_rx_msg = msg
                    t_sent = msg.get('t_sent_ns', 0)
                    if t_sent:
                        self.state.last_rtt_ms = (time.time_ns() - t_sent) / 1e6
                    self.inbound.put(('data', msg))
            except Exception as e:
                self.inbound.put(('err', f'on_data: {e}'))

        self.room.on('data_received', on_data)

        def on_participant_connected(p):
            self.inbound.put(('info', f'+ {p.identity} 加入'))
        def on_participant_disconnected(p):
            self.inbound.put(('info', f'- {p.identity} 离开'))
        self.room.on('participant_connected', on_participant_connected)
        self.room.on('participant_disconnected', on_participant_disconnected)

        # Token
        token = api.AccessToken(LIVEKIT_API_KEY, LIVEKIT_API_SECRET)
        token.with_identity(LIVEKIT_IDENTITY)
        token.with_grants(api.VideoGrants(
            room_join=True, room=LIVEKIT_ROOM,
            can_publish=False, can_publish_data=True, can_subscribe=True,
        ))
        jwt = token.to_jwt()

        self.inbound.put(('info', f'连接 {LIVEKIT_URL}...'))
        await self.room.connect(LIVEKIT_URL, jwt)
        self.state.connected = True
        self.inbound.put(('connected', None))
        self.inbound.put(('info', f'✓ 已加入 {LIVEKIT_ROOM}'))

        # 发送循环 (20Hz)
        dt = 1.0 / SEND_HZ
        while True:
            vx, vy, wz, lift = self.state.snapshot()
            self.state.seq += 1
            payload = {
                'type': 'cmd_vel',
                'vx': vx, 'vy': vy, 'wz': wz,
                't_sent_ns': time.time_ns(),
                'seq': self.state.seq,
                'from': LIVEKIT_IDENTITY,
            }
            data = json.dumps(payload).encode('utf-8')
            try:
                await self.room.local_participant.publish_data(data, reliable=True)
                self.state.sent += 1
            except Exception as e:
                self.state.last_err = str(e)
                self.inbound.put(('err', f'publish: {e}'))

            # PR-4: 同帧紧跟一条 lift_cmd, 与 cmd_vel 一致频率;
            # schema 对齐 rain-orin/cloud_to_ros_bridge dispatcher
            # ({"type":"lift_cmd","speed":<float>}, speed ∈ [-1, +1]).
            lift_payload = {'type': 'lift_cmd', 'speed': lift}
            lift_data = json.dumps(lift_payload).encode('utf-8')
            try:
                await self.room.local_participant.publish_data(lift_data, reliable=True)
            except Exception as e:
                self.state.last_err = str(e)
                self.inbound.put(('err', f'publish lift: {e}'))

            await asyncio.sleep(dt)


# ============================================================
# GUI
# ============================================================
def run_gui(state: SharedState, inbound: queue.Queue):
    import tkinter as tk
    from tkinter import font as tkfont

    root = tk.Tk()
    root.title('仓颉 Cloud Teleop — 182 ASUS → Aliyun → Orin → CangJie')
    root.geometry('980x760')
    root.configure(bg=C_BG)

    big_font    = tkfont.Font(family='DejaVu Sans Mono', size=42, weight='bold')
    title_font  = tkfont.Font(family='DejaVu Sans', size=22, weight='bold')
    data_font   = tkfont.Font(family='DejaVu Sans Mono', size=20)
    small_font  = tkfont.Font(family='DejaVu Sans Mono', size=14)

    # --- 标题 ---
    title = tk.Label(root, text='⚙️ Cloud Teleop Control',
                     font=title_font, bg=C_BG, fg=C_FG)
    title.pack(pady=(14, 8))

    # --- 当前 cmd_vel 显示 ---
    cmd_frame = tk.Frame(root, bg=C_PANEL, relief=tk.RIDGE, bd=2)
    cmd_frame.pack(pady=8, padx=20, fill=tk.X)
    tk.Label(cmd_frame, text='SENDING cmd_vel →',
             font=small_font, bg=C_PANEL, fg=C_BLUE).pack(pady=(10, 2))
    cmd_label = tk.Label(cmd_frame, text='vx=+0.00  vy=+0.00  wz=+0.00  lift=+0.00',
                         font=big_font, bg=C_PANEL, fg=C_GREEN)
    cmd_label.pack(pady=(0, 10))

    # --- 按键说明 ---
    hints = (
        'W / S  前进/后退  |  A / D  左横移/右横移  |  Q / E  左转/右转  '
        '|  G / H  抬升 上/下  |  Space  归零  |  Esc  退出'
    )
    tk.Label(root, text=hints, font=small_font,
             bg=C_BG, fg=C_YELLOW).pack(pady=4)

    # --- press-and-hold 按键状态 ---
    held_keys = set()        # 当前按住 (经过 X11 auto-repeat 去重后)
    last_press_t = {}        # k → 最近 KeyPress 时间戳
    last_release_t = {}      # k → 最近 KeyRelease 时间戳

    def commit_held():
        state.set_held(held_keys)

    # --- 按钮网格 (鼠标点击 = 短脉冲: 按下时加入 held_keys, 200ms 后自动松开) ---
    btn_frame = tk.Frame(root, bg=C_BG)
    btn_frame.pack(pady=10)
    btn_cfg = dict(width=5, height=2, font=data_font,
                   bg=C_PANEL, fg=C_FG, activebackground=C_BLUE)

    def btn_click(k):
        held_keys.add(k)
        commit_held()
        # 200ms 后自动归零 (鼠标点一下相当于按下并立即松开 → 给一小段速度脉冲)
        def release():
            held_keys.discard(k)
            commit_held()
        root.after(200, release)
        root.focus_set()

    def mk_btn(parent, txt, key, r, c):
        b = tk.Button(parent, text=txt, **btn_cfg,
                      command=lambda k=key: btn_click(k))
        b.grid(row=r, column=c, padx=4, pady=4)
    mk_btn(btn_frame, 'Q', 'q', 0, 0)
    mk_btn(btn_frame, 'W', 'w', 0, 1)
    mk_btn(btn_frame, 'E', 'e', 0, 2)
    mk_btn(btn_frame, 'A', 'a', 1, 0)
    mk_btn(btn_frame, 'S', 's', 1, 1)
    mk_btn(btn_frame, 'D', 'd', 1, 2)
    # PR-4: G/H 抬升 (与 WASDQE 同 click-pulse 行为)
    mk_btn(btn_frame, 'G ↑', 'g', 0, 3)
    mk_btn(btn_frame, 'H ↓', 'h', 1, 3)
    tk.Button(btn_frame, text='Space 归零', width=18, height=2,
              font=data_font, bg=C_YELLOW, fg='#11111b',
              command=lambda: (held_keys.clear(), state.stop(), root.focus_set())
              ).grid(row=2, column=0, columnspan=4, pady=(6, 0))

    # --- 状态 bar ---
    status_label = tk.Label(
        root, text='○ 未连  发送=0  回包=0  延时=-- ms',
        font=data_font, bg=C_BG, fg=C_RED)
    status_label.pack(pady=8)

    # --- 接收到的 chassis_state 显示 ---
    ack_frame = tk.Frame(root, bg=C_PANEL, relief=tk.RIDGE, bd=2)
    ack_frame.pack(pady=8, padx=20, fill=tk.BOTH, expand=True)
    tk.Label(ack_frame, text='← CangJie 回包 (4 轮 IK 解算):',
             font=small_font, bg=C_PANEL, fg=C_BLUE).pack(pady=(10, 4), anchor='w', padx=12)
    ack_label = tk.Label(ack_frame, text='(waiting...)',
                         font=data_font, bg=C_PANEL, fg=C_FG, justify='left')
    ack_label.pack(pady=(0, 10), padx=12, anchor='w')

    # --- 事件 log ---
    log_frame = tk.Frame(root, bg=C_BG)
    log_frame.pack(pady=4, padx=20, fill=tk.X)
    log_text = tk.Text(log_frame, height=4, font=small_font,
                       bg='#11111b', fg=C_FG, wrap='none')
    log_text.pack(fill=tk.X)

    def log(msg):
        ts = time.strftime('%H:%M:%S')
        log_text.insert('end', f'[{ts}] {msg}\n')
        log_text.see('end')
        # 限长
        lines = log_text.index('end').split('.')[0]
        if int(lines) > 50:
            log_text.delete('1.0', '20.0')

    # --- 键盘绑定 (press-and-hold + X11 auto-repeat 去重) ---
    # X11 auto-repeat 会在按住时反复发 KeyPress + KeyRelease 对.
    # 策略: KeyRelease 后等 RELEASE_DEBOUNCE_S, 期间又来 KeyPress 就视为 auto-repeat 不松手.

    def on_press(e):
        ch = e.keysym.lower()
        if ch == 'escape':
            root.destroy()
            return
        if ch == 'space' or e.char == ' ':
            held_keys.clear()
            state.stop()
            return
        if ch in ('w','a','s','d','q','e','g','h'):
            now = time.monotonic()
            last_press_t[ch] = now
            if ch not in held_keys:
                held_keys.add(ch)
                commit_held()

    def on_release(e):
        ch = e.keysym.lower()
        if ch not in ('w','a','s','d','q','e','g','h'):
            return
        last_release_t[ch] = time.monotonic()
        # 等一会儿确认是否真松手 (auto-repeat 会在 ~30ms 内补一个 KeyPress)
        def maybe_release(k=ch, t_release=last_release_t[ch]):
            # 如果在我们 schedule 后, 收到了新的 KeyPress, 那是 auto-repeat, 不松手
            if last_press_t.get(k, 0) > t_release:
                return
            held_keys.discard(k)
            commit_held()
        root.after(int(RELEASE_DEBOUNCE_S * 1000), maybe_release)

    root.bind('<KeyPress>', on_press)
    root.bind('<KeyRelease>', on_release)
    root.focus_set()

    # --- 定时刷新 UI ---
    def refresh():
        vx, vy, wz, lift = state.snapshot()
        cmd_label.config(text=f'vx={vx:+.2f}  vy={vy:+.2f}  wz={wz:+.2f}  lift={lift:+.2f}')

        if state.connected:
            status_label.config(
                text=f'● 已连 {LIVEKIT_URL}  '
                     f'发送={state.sent}  回包={state.recv}  '
                     f'延时={state.last_rtt_ms:.0f} ms' if state.last_rtt_ms is not None
                     else f'● 已连 {LIVEKIT_URL}  发送={state.sent}  回包={state.recv}  延时=-- ms',
                fg=C_GREEN)
        else:
            status_label.config(text=f'○ 未连  发送={state.sent}  回包={state.recv}', fg=C_RED)

        # 回包显示
        if state.last_rx_msg:
            wheels = state.last_rx_msg.get('wheels', [])
            ik_us = state.last_rx_msg.get('ik_us', 0)
            mock = '[MOCK]' if state.last_rx_msg.get('mock') else '[REAL]'
            lines = [f'{mock} IK {ik_us:.0f}μs @ CangJie']
            for w in wheels:
                lines.append(
                    f"  {w['name']}: steer={w['steer_deg']:+6.1f}°  "
                    f"drive={w['rad_s']:+.2f} rad/s ({w['rpm']:+5.1f} rpm)"
                )
            ack_label.config(text='\n'.join(lines), fg=C_GREEN)

        # 处理 inbound 事件
        try:
            while True:
                evt = inbound.get_nowait()
                kind, payload = evt
                if kind == 'connected':
                    log('✓ 已连阿里云 LiveKit')
                elif kind == 'info':
                    log(payload)
                elif kind == 'data':
                    pass  # refresh 会自己画
                elif kind == 'err':
                    log(f'[ERR] {payload}')
        except queue.Empty:
            pass

        root.after(50, refresh)  # 20 Hz

    root.after(50, refresh)
    root.mainloop()


# ============================================================
# CLI 自测 (无 GUI, 自动发几档然后打印收到的 chassis_state)
# ============================================================
def run_cli(state: SharedState, inbound: queue.Queue, scenarios=None, per_scene_s=2.0):
    if scenarios is None:
        # PR-4: 4-tuple (vx, vy, wz, lift); lift ∈ [-1.0, +1.0] 归一化
        scenarios = [
            (0.0, 0.0, 0.8, 0.0),   # 原地左转
            (0.25, 0.0, 0.0, 0.0),  # 直行
            (0.15, 0.15, 0.0, 0.0), # 斜 45°
            (0.0, 0.0, 0.0, +0.5),  # 抬升上 (G)
            (0.0, 0.0, 0.0, -0.5),  # 抬升下 (H)
            (0.0, 0.0, 0.0, 0.0),   # 归零
        ]

    # 等连接
    t0 = time.time()
    while not state.connected and time.time() - t0 < 10:
        try:
            evt = inbound.get(timeout=0.3)
            print(f'  [LK] {evt}')
        except queue.Empty:
            pass
    if not state.connected:
        print('❌ LiveKit 连接超时')
        return False

    print('✓ LiveKit 已连, 开始场景测试\n')

    for idx, (vx, vy, wz, lift) in enumerate(scenarios):
        with state.lock:
            state.vx, state.vy, state.wz, state.lift = vx, vy, wz, lift
        before_recv = state.recv
        time.sleep(per_scene_s)
        rx = state.recv - before_recv
        rtt = state.last_rtt_ms
        last = state.last_rx_msg
        print(f'[{idx+1}/{len(scenarios)}] sent vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f} lift={lift:+.2f}')
        print(f'  → 本场景收到 {rx} 个 chassis_state 回包, '
              f'累计发送={state.sent} 收={state.recv} rtt={rtt:.0f} ms' if rtt is not None
              else f'  → 本场景收到 {rx} 个 chassis_state 回包, '
                   f'累计发送={state.sent} 收={state.recv}')
        if last:
            for w in last.get('wheels', []):
                print(f"      {w['name']}: steer={w['steer_deg']:+6.1f}°  "
                      f"drive={w['rad_s']:+.2f} rad/s")
        print()

    return state.recv > 0


# ============================================================
# main
# ============================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cli', action='store_true',
                    help='CLI 自测模式 (无 GUI, 跑几个场景后退出)')
    args = ap.parse_args()

    state = SharedState()
    inbound = queue.Queue()
    worker = LiveKitWorker(state, inbound)
    worker.start()

    if args.cli:
        ok = run_cli(state, inbound)
        print('\n=== CLI 自测结束 ===')
        print('✅ 链路通' if ok else '❌ 链路不通 (0 回包)')
        sys.exit(0 if ok else 1)
    else:
        run_gui(state, inbound)


if __name__ == '__main__':
    main()
