# operator-asus/ — ASUS Tkinter GUI 操控面板

> Last updated: 2026-04-26 (PR-4). See [`../ARCHITECTURE.md`](../ARCHITECTURE.md) for system context.

## Role

Operator-end teleop GUI running on the ASUS TUF F15 (`edward-asus`) host. Sends keyboard input to the Aliyun LiveKit data channel; the data lands on Orin's `cloud_to_ros_bridge` → ROS 2 `/cmd_vel` + `/lift_cmd` → CangJie's `chassis_controller_v3` (swerve IK + DM3519 一拖四 lift current loop).

## Hardware / OS

- ASUS TUF F15 FX506HM, Intel i7-11800H, 16 GB RAM
- Hostname `edward-asus`, sudo user `edward`
- Ubuntu 22.04.3, Python 3.10 (Tkinter via stdlib)
- LAN: `192.168.0.172` (eth) + `192.168.0.182` (WiFi); Tailscale `100.103.0.5`

## Keyboard layout

| Key | Action | Magnitude |
|---|---|---|
| `W` / `S` | Chassis 前进 / 后退 (vx) | `LIN_HOLD = 0.20 m/s` |
| `A` / `D` | Chassis 左横移 / 右横移 (vy) | `LIN_HOLD = 0.20 m/s` |
| `Q` / `E` | Chassis 偏航左 / 右 (wz) | `ANG_HOLD = 1.00 rad/s` |
| `G` / `H` | **抬升 上 / 下** (lift, PR-4 新增) | `LIFT_STEP = 0.5` (归一化, ∈ [-1.0, +1.0]) |
| `Space` | 紧急归零 (vx=vy=wz=lift=0) | — |
| `Esc` | 退出 GUI | — |

Hold = continuous publish at 20 Hz. Release → 0.0 (chassis or lift goes to neutral). Mouse-click on a button = 200 ms pulse (same key insert + auto-release path).

X11 auto-repeat dedup is handled in `on_press`/`on_release` with a `RELEASE_DEBOUNCE_S = 0.04 s` window so a held key stays active without flutter.

## Wire format (LiveKit data channel, room `teleop-room`)

Both messages are JSON published on the same data channel as `reliable=True`, 20 Hz each (so 40 packets/sec total per ASUS instance).

```jsonc
// Chassis (existing, unchanged)
{"type": "cmd_vel", "vx": 0.20, "vy": 0.00, "wz": 0.00,
 "t_sent_ns": 1714123456789012345, "seq": 42, "from": "edward-asus-teleop"}

// Lift (PR-4 新增)
{"type": "lift_cmd", "speed": 0.50}
```

`speed ∈ [-1.0, +1.0]` is the normalized lift velocity. The Orin dispatcher clamps to that range and republishes as `std_msgs/msg/Float32` on `/lift_cmd`; CangJie maps it to DM3519 current-loop targets (see [`../rain-cangjie/README.md`](../rain-cangjie/README.md)).

## Bring-up

1. Clone repo:
   ```bash
   cd ~ && git clone https://github.com/thedaosheng/Remote_Control.git
   ```
2. Install Python deps:
   ```bash
   pip install --user -i https://pypi.tuna.tsinghua.edu.cn/simple livekit livekit-api
   ```
3. Configure env (copy `../.env.example` to `.env` at repo root, fill in `LIVEKIT_API_KEY` + `LIVEKIT_API_SECRET`):
   ```bash
   cd ~/Remote_Control
   cp .env.example .env
   # edit .env, fill in the LiveKit secret (see ../SECURITY.md)
   set -a; source .env; set +a
   ```
   The script will fail-fast with a clear error if either env var is missing — by design, no fallback to a leaked default.
4. Run GUI on local display:
   ```bash
   DISPLAY=:0 python3 operator-asus/asus_teleop_gui.py
   ```
5. Or CLI mode (no display, scripted scenarios for self-test of the link):
   ```bash
   python3 operator-asus/asus_teleop_gui.py --cli
   ```
   The CLI now exercises chassis (3 scenarios) plus G/H lift (2 scenarios) plus a final zero, end-to-end through the SFU.
6. Verify the GUI shows `lift=±0.50` next to `vx vy wz` when G or H is held.

## End-to-end demo (mock, assumes Orin + CangJie running per their READMEs)

1. Start `cloud_to_ros_bridge.py` on Orin (or its systemd service when wired up — see [`../rain-orin/README.md`](../rain-orin/README.md)).
2. Start `chassis_controller_v3` on CangJie in mock mode (see [`../rain-cangjie/README.md`](../rain-cangjie/README.md)).
3. Start this GUI on ASUS.
4. Press `G` and hold — observe:
   - Local UI: `lift=+0.50`
   - Orin `cloud_to_ros_bridge` log: `[c2r] ← lift_cmd #N speed=+0.50`
   - CangJie `chassis_controller_v3` mock log: `[lift] norm=+0.50 → I=+2.50 A → int16=+2048 → CAN 0x200 [0x08 0x00 0x00 0x00 0x00 0x00 0x00 0x00]`
5. Release `G`. Observe lift returns to 0; CangJie watchdog will zero within `lift_wdt_s` if publish stops entirely (see chassis_controller_v3 source).
6. Press `H`. Observe negative lift (lift down) symmetrically: `[lift] norm=-0.50 → I=-2.50 A → int16=-2048 → CAN 0x200 [0xF8 0xFF ...]`.

## Files

- `asus_teleop_gui.py` — main Tkinter GUI + LiveKit publisher (~490 lines, single file)
- `README.md` — this file

## Known debt

- `LIFT_STEP = 0.5` is a placeholder; calibrate against real DM3519 + lift frame mass + safety margins later.
- No emergency-stop button on the GUI yet; safety-stop today is only via `chassis_safety_node` ROS 2 services on the Rain side. Future PR could add a GUI button that publishes `{"type":"emergency_stop"}` on the data channel and have `cloud_to_ros_bridge` dispatch to `/arms/emergency_stop_all`.
- No connection state indicator beyond the send/recv counters and RTT line; rebooting the GUI doesn't auto-reconnect on transient SFU drop (manual restart required).
- `Space` only zeros `vx/vy/wz/lift` in the shared state — it doesn't immediately flush a zero packet; the next 20 Hz tick (≤ 50 ms later) sends the zeroed payload.
- Rate doubling (2× 20 Hz instead of 1) on the data channel from this PR-4 split is a small bandwidth cost (~80 B/packet × 20 Hz × 2 = ~3 KB/s up); negligible vs the video uplink, but if it ever matters we can pack `cmd_vel` and `lift_cmd` into a single combined-payload message and split on the receiver.

## See also

- [`../ARCHITECTURE.md`](../ARCHITECTURE.md) — system topology + machine roles + data flows
- [`../SECURITY.md`](../SECURITY.md) — env-var policy and secret rotation procedure
- [`../rain-orin/README.md`](../rain-orin/README.md) — Orin `cloud_to_ros_bridge` (LiveKit data → ROS 2 `/cmd_vel` + `/lift_cmd`)
- [`../rain-cangjie/README.md`](../rain-cangjie/README.md) — CangJie `chassis_controller_v3` (swerve IK + DM3519 一拖四 lift)
- [`../.env.example`](../.env.example) — required env vars
