#!/usr/bin/env python3
"""
=====================================================
  Vision Pro → 达妙电机 实时控制 (第四轮交互)
=====================================================

数据链路:
  Vision Pro (LiveKit Data Channel pose)
    → sender (zedmini_livekit_sender.py 内部 _on_data_received)
    → UDP 127.0.0.1:9000  (JSON: {"pitch":deg, "yaw":deg, "t":...})
    → 本脚本 (UDP 监听 + 缓存最新 pose)
    → 200Hz 控制循环 → 达妙电机 (CAN MIT 模式)

关键映射:
  电机1 (CAN 0x01) = Yaw   左右转头  (正方向 = 左)
  电机2 (CAN 0x02) = Pitch 俯仰点头  (正方向 = 抬头)

  vp_yaw_deg / vp_pitch_deg 是相对于 VP 启动时校准姿态的角度,
  电机的零点在 Flash 里(掰过的物理位置), 工作区不对称于零点。
  所以用"工作区中心 + scale × sign × vp_deg"映射:

      target_motor_deg = motor_center + SIGN × SCALE × vp_deg

  SIGN 默认 +1, 如果实际方向反了可改成 -1。
  SCALE 默认 1.0 (1°→1°), 想放大就调大。

工作区(从 dm_motor_calibration.json 读, 内缩 5° 安全余量):
  Yaw   [-60.06°, +5.24°]   中心 -27.41°  半幅 ±32.65°
  Pitch [-44.93°, +26.53°]  中心  -9.20°  半幅 ±35.73°

梯度档位 (跟手感, 详见 dm_motor_vp_control_PRESETS.md):
  --preset L1   安全档    (KP=8  KD=1.2 ff=off rate=100)  软, 慢, 0 风险
  --preset L2   温和档    (KP=15 KD=1.2 ff=0.5/0.3 rate=200)
  --preset L3   默认档 ⭐ (KP=20 KD=1.0 ff=1.0/0.5 rate=300)  推荐起步
  --preset L4   跟手档    (KP=25 KD=1.0 ff=1.2/0.7 rate=500)  你想要的"更跟"
  --preset L5   激进档    (KP=30 KD=0.8 ff=1.5/0.8 rate=600)  有震动风险

  例: /usr/bin/python3 20260407-cc-dm_motor_vp_control.py --preset L4

启动 (手动调参):
  /usr/bin/python3 20260407-cc-dm_motor_vp_control.py

  可选参数 (--preset 之后还能用单独参数覆盖):
  --sign-yaw   +1|-1   (yaw 极性, 默认 +1)
  --sign-pitch +1|-1   (pitch 极性, 默认 +1)
  --scale-yaw   1.0
  --scale-pitch 1.0
  --kp 20  --kd 1.0  --rate-limit 300
  --feedforward on|off  --ff-gain 1.0  --ff-ema 0.5

退出: Ctrl+C 安全归到中心 + 失能 + 关串口

作者: Claude Code
日期: 2026-04-07
"""

import sys, os, time, math, json, signal, socket, threading, argparse
sys.path.insert(0, '/home/rhz/teleop/DM_Control_Python')
import serial
from DM_CAN import Motor, MotorControl, DM_Motor_Type, Control_Type

CALIB_PATH = '/home/rhz/teleop/scripts/dm_motor_calibration.json'

# ============================================================
# 控制参数
# ============================================================
SAFETY_MARGIN_DEG    = 5.0
CTRL_FREQ            = 200
DT                   = 1.0 / CTRL_FREQ
RAMP_TO_CENTER_SECS  = 3.0      # 启动时从当前位置 ramp 到中心的时长
DATA_TIMEOUT_SECS    = 0.5      # >0.5s 没收到 VP pose 就保持中心
TAKEOVER_FADE_SECS   = 1.5      # 进入主循环后 vp 输入缩放从 0→1 淡入,
                                # 防止电机从中心位置"跳"到 vp 当前姿态
PRINT_INTERVAL_SECS  = 2.0      # 状态监测打印周期
# MAX_DELTA_PER_FRAME 改用 --rate-limit 参数动态计算 (默认 300°/s)


# ============================================================
# 梯度档位 (跟手感由低到高, 见 dm_motor_vp_control_PRESETS.md)
#
# 这里只放可调旋钮: kp, kd, rate_limit, feedforward, ff_gain, ff_ema
# sign / scale 不动, 因为那是物理映射, 不是跟手感旋钮
# ============================================================
PRESETS = {
    "L1": dict(kp=8.0,  kd=1.2, rate_limit=100, feedforward="off", ff_gain=0.0, ff_ema=0.3),
    "L2": dict(kp=15.0, kd=1.2, rate_limit=200, feedforward="on",  ff_gain=0.5, ff_ema=0.3),
    "L3": dict(kp=20.0, kd=1.0, rate_limit=300, feedforward="on",  ff_gain=1.0, ff_ema=0.5),
    "L4": dict(kp=25.0, kd=1.0, rate_limit=500, feedforward="on",  ff_gain=1.2, ff_ema=0.7),
    "L5": dict(kp=30.0, kd=0.8, rate_limit=600, feedforward="on",  ff_gain=1.5, ff_ema=0.8),
}


# ============================================================
# 共享状态: UDP 接收线程写, 控制循环读
# ============================================================
class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.vp_pitch_deg     = 0.0   # 来自 sender (sender 内部已 swap quat→roll/pitch/yaw)
        self.vp_yaw_deg       = 0.0
        self.vp_pitch_dot_dps = 0.0   # ★ EMA 平滑后的角速度 (deg/s),作为速度前馈
        self.vp_yaw_dot_dps   = 0.0
        self.last_t           = 0.0   # 上次收到包的本地时间
        self.recv_count       = 0
        # EMA 平滑系数 (由 main 在启动时填充, 避免常量耦合)
        # α=0.3 → 组延迟 ~78ms (温和)
        # α=0.5 → 组延迟 ~33ms (默认平衡)
        # α=0.7 → 组延迟 ~14ms (激进, 容易抖)
        # α=1.0 → 无平滑 (完全跟随,会有抖动)
        self.vel_ema_alpha    = 0.5


def udp_receive_thread(state: SharedState, port: int = 9000):
    """监听 UDP, 解析 JSON 中的 pitch/yaw (度), 更新 SharedState

    每收到一个新 pose, 算与上一帧的差分得到瞬时角速度,
    再用 EMA 平滑后存入 vp_*_dot_dps, 供主循环做速度前馈。
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('127.0.0.1', port))
    sock.settimeout(1.0)
    print(f"  [UDP] 监听 127.0.0.1:{port}")
    while True:
        try:
            data, _ = sock.recvfrom(4096)
            msg = json.loads(data.decode('utf-8'))
            new_p = float(msg.get('pitch', 0.0))
            new_y = float(msg.get('yaw',   0.0))
            now   = time.time()
            with state.lock:
                # 计算瞬时角速度 (deg/s) + EMA 平滑
                # α 从 state 读, 由命令行 --ff-ema 控制
                alpha = state.vel_ema_alpha
                if state.last_t > 0:
                    dt_pose = now - state.last_t
                    if dt_pose > 0.001:    # 防除 0 / 异常 dt
                        inst_p_dot = (new_p - state.vp_pitch_deg) / dt_pose
                        inst_y_dot = (new_y - state.vp_yaw_deg)   / dt_pose
                        state.vp_pitch_dot_dps = (
                            alpha * inst_p_dot +
                            (1 - alpha) * state.vp_pitch_dot_dps
                        )
                        state.vp_yaw_dot_dps = (
                            alpha * inst_y_dot +
                            (1 - alpha) * state.vp_yaw_dot_dps
                        )
                state.vp_pitch_deg = new_p
                state.vp_yaw_deg   = new_y
                state.last_t       = now
                state.recv_count  += 1
        except socket.timeout:
            # 数据流暂停了 —— 速度前馈衰减到 0, 防止用旧速度乱动
            with state.lock:
                state.vp_pitch_dot_dps *= 0.5
                state.vp_yaw_dot_dps   *= 0.5
        except Exception:
            pass


# ============================================================
# 主流程
# ============================================================
def main():
    p = argparse.ArgumentParser(description="Vision Pro → 达妙电机 实时控制",
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog="梯度档位: --preset L1/L2/L3/L4/L5 "
                                       "(见 dm_motor_vp_control_PRESETS.md)")
    p.add_argument("--serial",     default="/dev/ttyACM0", help="串口路径")
    p.add_argument("--udp-port",   type=int, default=9000)
    p.add_argument("--sign-yaw",   type=int, choices=[-1, 1], default=1)
    p.add_argument("--sign-pitch", type=int, choices=[-1, 1], default=1)
    p.add_argument("--scale-yaw",   type=float, default=1.0)
    p.add_argument("--scale-pitch", type=float, default=1.0)

    # ★ 梯度档位 —— 一键选,也可以之后用单独参数覆盖任意一项
    p.add_argument("--preset", choices=list(PRESETS.keys()), default=None,
                   help="梯度档位 L1=安全/L2=温和/L3=默认⭐/L4=跟手/L5=激进; "
                        "不设则用各参数的独立默认值")

    p.add_argument("--kp", type=float, default=None,
                   help="MIT 位置刚度 (8=软, 30=硬, 默认 20)")
    p.add_argument("--kd", type=float, default=None,
                   help="MIT 速度阻尼 (0.5-2.0, 默认 1.0)")
    p.add_argument("--rate-limit", type=float, default=None,
                   help="逐帧位置变化上限 (deg/s, 默认 300)")
    p.add_argument("--feedforward", choices=["on", "off"], default=None,
                   help="速度前馈开关 (默认 on)")
    p.add_argument("--ff-gain", type=float, default=None,
                   help="速度前馈增益 (默认 1.0)")
    p.add_argument("--ff-ema", type=float, default=None,
                   help="速度前馈 EMA α (0.1~1.0, 默认 0.5; 越大越跟手)")
    args = p.parse_args()

    # ----- 解析 preset, 单独参数覆盖 preset -----
    # 1. 先取 L3 默认 (没指定 preset 时也用这一组当基准)
    cfg = dict(PRESETS["L3"])
    # 2. 如果指定了 preset, 用它覆盖
    if args.preset:
        cfg.update(PRESETS[args.preset])
    # 3. 命令行单独参数 (不为 None 的) 再覆盖一次
    for key in ("kp", "kd", "rate_limit", "feedforward", "ff_gain", "ff_ema"):
        v = getattr(args, key)
        if v is not None:
            cfg[key] = v
    # 把最终值写回 args (后面的代码全用 args.kp 等)
    args.kp           = cfg["kp"]
    args.kd           = cfg["kd"]
    args.rate_limit   = cfg["rate_limit"]
    args.feedforward  = cfg["feedforward"]
    args.ff_gain      = cfg["ff_gain"]
    args.ff_ema       = cfg["ff_ema"]
    preset_label      = args.preset if args.preset else "custom"

    # 把 deg/s 上限换算成 rad/帧
    max_delta_per_frame = math.radians(args.rate_limit) * DT

    # ----- 加载校准, 算工作区 -----
    with open(CALIB_PATH) as f:
        calib = json.load(f)

    yaw_min   = calib['summary']['motor1_yaw_can01']['min_deg']   + SAFETY_MARGIN_DEG
    yaw_max   = calib['summary']['motor1_yaw_can01']['max_deg']   - SAFETY_MARGIN_DEG
    pitch_min = calib['summary']['motor2_pitch_can02']['min_deg'] + SAFETY_MARGIN_DEG
    pitch_max = calib['summary']['motor2_pitch_can02']['max_deg'] - SAFETY_MARGIN_DEG

    yaw_center   = (yaw_min + yaw_max) / 2
    pitch_center = (pitch_min + pitch_max) / 2
    yaw_half     = (yaw_max - yaw_min) / 2
    pitch_half   = (pitch_max - pitch_min) / 2

    KP = args.kp
    KD = args.kd

    print("=" * 60)
    print(f"  VP → 达妙 实时控制   [档位: {preset_label}]")
    print("=" * 60)
    print(f"  Yaw   工作区 [{yaw_min:+6.2f}°, {yaw_max:+6.2f}°]  中心 {yaw_center:+6.2f}°")
    print(f"  Pitch 工作区 [{pitch_min:+6.2f}°, {pitch_max:+6.2f}°]  中心 {pitch_center:+6.2f}°")
    print(f"  Yaw   sign={args.sign_yaw:+d}  scale={args.scale_yaw}")
    print(f"  Pitch sign={args.sign_pitch:+d}  scale={args.scale_pitch}")
    print(f"  MIT KP={KP}  KD={KD}  rate-limit={args.rate_limit:.0f}°/s")
    print(f"  速度前馈: {args.feedforward}  增益={args.ff_gain}  EMA α={args.ff_ema}")
    print("=" * 60)

    feedforward_enabled = (args.feedforward == "on")
    # 把命令行的 ff_ema 注入 SharedState (UDP 线程会读它做 EMA 平滑)
    # 在 state 创建之后再赋值,所以这一行放到下面 state 创建后

    # 硬限幅 (工作区 +1° 缓冲)
    yaw_hard_lo = math.radians(yaw_min - 1);   yaw_hard_hi = math.radians(yaw_max + 1)
    pit_hard_lo = math.radians(pitch_min - 1); pit_hard_hi = math.radians(pitch_max + 1)

    def clamp(v, lo, hi): return max(lo, min(hi, v))

    # ----- 启动 UDP 接收线程 -----
    state = SharedState()
    state.vel_ema_alpha = args.ff_ema   # 注入命令行配置
    udp_t = threading.Thread(target=udp_receive_thread,
                             args=(state, args.udp_port), daemon=True)
    udp_t.start()

    # ----- 打开串口 + 创建电机 -----
    ser = serial.Serial(args.serial, 921600, timeout=1)
    mc = MotorControl(ser)
    m_yaw   = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
    m_pitch = Motor(DM_Motor_Type.DM4310, 0x02, 0x12)
    mc.addMotor(m_yaw)
    mc.addMotor(m_pitch)
    time.sleep(0.3)

    mc.switchControlMode(m_yaw,   Control_Type.MIT)
    mc.switchControlMode(m_pitch, Control_Type.MIT)
    mc.enable(m_yaw)
    mc.enable(m_pitch)
    time.sleep(0.3)

    # warmup 读初始位置
    for _ in range(3):
        mc.refresh_motor_status(m_yaw)
        mc.refresh_motor_status(m_pitch)
        time.sleep(0.05)
    cur_y = float(m_yaw.getPosition())
    cur_p = float(m_pitch.getPosition())
    print(f"\n  当前位置: Yaw {math.degrees(cur_y):+.2f}°  Pitch {math.degrees(cur_p):+.2f}°")
    print(f"  → ramp 到中心 [Yaw {yaw_center:+.2f}°, Pitch {pitch_center:+.2f}°]\n")

    # ----- 安全退出 -----
    running = [True]
    def on_sig(sig, frame):
        running[0] = False
        print("\n  [Ctrl+C] 安全归零中, 请稍候...")
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    try:
        # ----- Step 1: 从当前位置 cosine ramp 到中心 -----
        ramp_steps = int(RAMP_TO_CENTER_SECS * CTRL_FREQ)
        yc_rad = math.radians(yaw_center)
        pc_rad = math.radians(pitch_center)
        for i in range(ramp_steps):
            if not running[0]: break
            ratio  = (i + 1) / ramp_steps
            smooth = 0.5 - 0.5 * math.cos(math.pi * ratio)
            ty = cur_y + (yc_rad - cur_y) * smooth
            tp = cur_p + (pc_rad - cur_p) * smooth
            mc.controlMIT(m_yaw,   KP, KD, clamp(ty, yaw_hard_lo, yaw_hard_hi), 0, 0)
            mc.controlMIT(m_pitch, KP, KD, clamp(tp, pit_hard_lo, pit_hard_hi), 0, 0)
            time.sleep(DT)

        # 主循环初始 prev = 当前 (中心)
        prev_y = yc_rad
        prev_p = pc_rad
        last_print = time.time()

        # 主循环开始时间, 用于 vp 输入淡入
        main_loop_t0 = time.time()

        print("  ✓ 已到中心, 开始监听 VP pose ...")
        print(f"  ★ 前 {TAKEOVER_FADE_SECS}s 淡入: vp 输入缩放从 0 → 1, 平滑接管")
        print("  状态监测每 2 秒打印一行: VP 输入 | 目标 | 实际\n")

        # ----- Step 2: 主控制循环 (200Hz) -----
        while running[0]:
            with state.lock:
                vp_p     = state.vp_pitch_deg
                vp_y     = state.vp_yaw_deg
                vp_p_dot = state.vp_pitch_dot_dps   # ★ 速度前馈源
                vp_y_dot = state.vp_yaw_dot_dps
                last_t   = state.last_t
                n_recv   = state.recv_count

            age = time.time() - last_t if last_t > 0 else float('inf')

            # 淡入因子: 主循环开始 1.5s 内, fade 从 0 → 1, 平滑接管
            elapsed_main = time.time() - main_loop_t0
            fade = min(1.0, elapsed_main / TAKEOVER_FADE_SECS)

            # 计算目标位置 + 目标速度前馈
            if age > DATA_TIMEOUT_SECS:
                # 没数据 → 保持中心, 速度前馈也归 0
                target_y_rad = yc_rad
                target_p_rad = pc_rad
                ff_y_rad_per_sec = 0.0
                ff_p_rad_per_sec = 0.0
                state_label = f"NO DATA({age:4.1f}s)" if age != float('inf') else "WAIT     "
            else:
                # ----- 位置目标 -----
                # vp_*_deg 是相对头部初始姿态的角度 (度)
                # fade × scale × sign × vp_deg 是从中心的偏移量 (度)
                offset_y_deg = args.sign_yaw   * args.scale_yaw   * fade * vp_y
                offset_p_deg = args.sign_pitch * args.scale_pitch * fade * vp_p
                target_y_deg = yaw_center   + offset_y_deg
                target_p_deg = pitch_center + offset_p_deg
                target_y_deg = max(yaw_min,   min(yaw_max,   target_y_deg))
                target_p_deg = max(pitch_min, min(pitch_max, target_p_deg))
                target_y_rad = math.radians(target_y_deg)
                target_p_rad = math.radians(target_p_deg)

                # ----- 速度前馈 -----
                # vp_*_dot_dps 已经在 UDP 线程做了 EMA 平滑
                # 同样的 sign × scale × fade × ff_gain 应用到速度上
                if feedforward_enabled:
                    ff_y_dps = (args.sign_yaw   * args.scale_yaw
                                * fade * args.ff_gain * vp_y_dot)
                    ff_p_dps = (args.sign_pitch * args.scale_pitch
                                * fade * args.ff_gain * vp_p_dot)
                    # 速度前馈也限幅 (不超过 rate-limit, 防 EMA 偶发尖峰)
                    ff_y_dps = max(-args.rate_limit, min(args.rate_limit, ff_y_dps))
                    ff_p_dps = max(-args.rate_limit, min(args.rate_limit, ff_p_dps))
                    ff_y_rad_per_sec = math.radians(ff_y_dps)
                    ff_p_rad_per_sec = math.radians(ff_p_dps)
                else:
                    ff_y_rad_per_sec = 0.0
                    ff_p_rad_per_sec = 0.0

                state_label = f"RECV f{fade:.2f}" if fade < 1.0 else "RECV     "

            # 逐帧位置限速 (防 VP 突跳)
            dy = target_y_rad - prev_y
            dp = target_p_rad - prev_p
            dy = max(-max_delta_per_frame, min(max_delta_per_frame, dy))
            dp = max(-max_delta_per_frame, min(max_delta_per_frame, dp))
            ty = prev_y + dy
            tp = prev_p + dp

            # 硬限幅 (任何情况下都不超过)
            ty = clamp(ty, yaw_hard_lo, yaw_hard_hi)
            tp = clamp(tp, pit_hard_lo, pit_hard_hi)

            # MIT 控制: 位置 + 速度前馈 + (无 τ_ff)
            mc.controlMIT(m_yaw,   KP, KD, ty, ff_y_rad_per_sec, 0)
            mc.controlMIT(m_pitch, KP, KD, tp, ff_p_rad_per_sec, 0)
            prev_y, prev_p = ty, tp

            # 状态监测打印 (每 PRINT_INTERVAL_SECS 一次)
            now = time.time()
            if now - last_print >= PRINT_INTERVAL_SECS:
                last_print = now
                mc.refresh_motor_status(m_yaw)
                mc.refresh_motor_status(m_pitch)
                actual_y = math.degrees(float(m_yaw.getPosition()))
                actual_p = math.degrees(float(m_pitch.getPosition()))
                ff_y_dps_disp = math.degrees(ff_y_rad_per_sec)
                ff_p_dps_disp = math.degrees(ff_p_rad_per_sec)
                print(f"[{state_label}] "
                      f"VP: Y{vp_y:+6.1f}° P{vp_p:+6.1f}° "
                      f"vel(Y{vp_y_dot:+5.0f} P{vp_p_dot:+5.0f})°/s | "
                      f"目标: Y{math.degrees(ty):+6.1f}° P{math.degrees(tp):+6.1f}° "
                      f"ff(Y{ff_y_dps_disp:+5.0f} P{ff_p_dps_disp:+5.0f})°/s | "
                      f"实际: Y{actual_y:+6.1f}° P{actual_p:+6.1f}° | "
                      f"#{n_recv}")

            time.sleep(DT)

    except Exception as e:
        print(f"\n  [错误] {e}")
        import traceback; traceback.print_exc()

    finally:
        # ----- 安全收尾: 缓慢归零 + 失能 -----
        print("\n  [收尾] 缓慢回到中心...")
        try:
            for _ in range(3):
                mc.refresh_motor_status(m_yaw)
                mc.refresh_motor_status(m_pitch)
                time.sleep(0.05)
            ey = float(m_yaw.getPosition())
            ep = float(m_pitch.getPosition())
            yc_rad = math.radians(yaw_center)
            pc_rad = math.radians(pitch_center)
            steps = int(2.0 * CTRL_FREQ)
            for i in range(steps):
                ratio = (i + 1) / steps
                smooth = 0.5 - 0.5 * math.cos(math.pi * ratio)
                ty = ey + (yc_rad - ey) * smooth
                tp = ep + (pc_rad - ep) * smooth
                mc.controlMIT(m_yaw,   KP, KD, ty, 0, 0)
                mc.controlMIT(m_pitch, KP, KD, tp, 0, 0)
                time.sleep(DT)
            # 在中心 hold 0.5 秒
            for _ in range(int(0.5 * CTRL_FREQ)):
                mc.controlMIT(m_yaw,   KP, KD, yc_rad, 0, 0)
                mc.controlMIT(m_pitch, KP, KD, pc_rad, 0, 0)
                time.sleep(DT)
        except Exception:
            pass

        print("  [收尾] 失能电机...")
        try:
            mc.disable(m_yaw)
            mc.disable(m_pitch)
        except Exception:
            pass
        if ser.is_open:
            ser.close()
        print("  [收尾] 完成, 退出")


if __name__ == "__main__":
    main()
