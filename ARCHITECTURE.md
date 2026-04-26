# ARCHITECTURE — 仓颉 / Faber teleop system

> Last updated: 2026-04-26. This doc is the single source of truth for layout, machine roles, and data flows. README.md keeps a short summary; this file is the detail.

## One-paragraph summary

A US-deployed supermarket shelf-inspection + restock teleop robot, operated remotely from China via an Aliyun LiveKit relay. The system has 5 components: an **Apple Vision Pro** for immersive video + head-pose teleop and an **ASUS Tkinter GUI** for chassis + lift keyboard control on the operator side; the **Aliyun LiveKit Server** as a stateless SFU relay in the cloud; an **AGX Orin** running ROS 2 high-level nodes (video pipeline, LiveKit↔ROS bridge, arm SDK) and a **CangJie Raspberry Pi 4** running the swerve chassis IK and the DM3519 lift controller on the deployment side. End-to-end glass-to-glass video latency runs ~200–260 ms over the public Internet.

## Architecture diagram

```
        ┌──────────────────┐
        │ Vision Pro       │  visionOS 26 + LiveKit Swift SDK (SwiftUIVideoView)
        │ (operator-vp/)   │   ── video sub + head-pose data pub ──┐
        │ MBA 2 (.0.212)   │                                       │
        └──────────────────┘                                       │
                                                                   │
        ┌──────────────────┐                                       │
        │ ASUS Tkinter GUI │  WASD/QE chassis · GH lift            │
        │ (operator-asus/) │   ── data ch: cmd_vel / lift_cmd ─────┤
        │ ASUS .0.182      │                                       │
        └──────────────────┘                                       │
                                                                   v
                                              ┌───────────────────────────────┐
                                              │ Aliyun LiveKit Server v1.10.1 │
                                              │ (cloud/)                      │
                                              │ ws://39.102.113.104:7880      │
                                              │ room: "teleop-room"           │
                                              └───────────────────────────────┘
                                                              ^  |
                                                video pub +   |  |  video sub
                                                ROS↔LK bridge |  v  + head-pose recv
                                              ┌───────────────────────────────┐
                                              │ AGX Orin                      │
                                              │ (rain-orin/)                  │
                                              │ ZED Mini → GStreamer NVENC    │
                                              │ → lk CLI publish              │
                                              │ + ROS 2 Humble nodes          │
                                              │ + AIRBOT arm SDK              │
                                              │ eno1=192.168.8.6 (Rain LAN)   │
                                              └───────────────────────────────┘
                                                              |
                                                ROS 2 over Cyclone DDS
                                                Domain 0, 192.168.8.0/24
                                                              |
                                                              v
                                              ┌───────────────────────────────┐
                                              │ CangJie Raspberry Pi 4        │
                                              │ (rain-cangjie/)               │
                                              │ chassis_controller (swerve IK)│
                                              │ + lift section (DM3519)       │
                                              │ eth0=192.168.8.7 (Rain LAN)   │
                                              └───────────────────────────────┘
                                                              |
                                                          USB-CAN
                                                              |
                                                              v
                                              4 swerve wheels + 1 lift (DM3519 一拖四)
```

## Directory mapping

| Dir | Component | Host | What's inside |
|---|---|---|---|
| `operator-vp/` | Vision Pro Xcode app | 212 (MacBook Air 2) | LiveKit Swift SDK receiver + head-pose publisher (PR-5) |
| `operator-asus/` | Tkinter GUI | ASUS TUF F15 (.182 / 100.103.0.5) | WASD/QE/GH keyboard → LiveKit data (PR-4) |
| `cloud/` | Aliyun LiveKit config | Aliyun ECS 39.102.113.104 | livekit.yaml, deployment notes |
| `rain-orin/` | Orin high-level ROS 2 | AGX Orin (.8.6 / 100.75.76.30) | ros2_ws (livekit_bridge), systemd, Cyclone XML, bridges |
| `rain-cangjie/` | CangJie Pi chassis + lift | Pi 4 (.8.7 / 100.70.149.16) | chassis_controller, chassis_safety, lift IK, debug scripts |
| `scripts/` | Historical scripts (pre-pivot) | — | Kept for reference; not actively maintained |
| `controller/`, `RemoteControl/` | Older VP-side code dirs | 212 | To be reconciled with `operator-vp/` in PR-5 |
| `force-feedback-pen/`, `TouchUsage/`, `HZY/`, `docker/` | Legacy components | — | Retained pending phase-2 review |

## Machine inventory

| Hostname | Role | LAN IP | Tailscale | SSH user | Cyclone iface |
|---|---|---|---|---|---|
| `bit-orin` | Orin (Rain high-level) | `192.168.8.6` (eno1, primary) + `192.168.0.103` (WiFi) | `100.75.76.30` | `bit` | eno1 |
| `CangJie` | Pi (Rain chassis+lift) | `192.168.8.7` (eth0, primary) + `192.168.0.150` (WiFi) | `100.70.149.16` | `cangjie` | eth0 |
| `edward-asus` | ASUS (operator GUI) | `192.168.0.172/.182` | `100.103.0.5` | `edward` | n/a (LiveKit only) |
| `MacBook-Air-2` | 212 (VP dev) | `192.168.0.212` | `100.72.16.46` | `Zhuanz` | n/a |
| (Aliyun ECS) | LiveKit relay | (public 39.102.113.104) | — | `root` | n/a |

## Key data flows

1. **Video** (Orin → VP): ZED Mini → GStreamer NVENC h264 (intra-only 1-slice 30fps 6Mbps Main, locked 2026-04-24 to avoid lk CLI 2.16.2 multi-slice packetize bug) → `tcpserversink :5004` → `lk` CLI → Aliyun LiveKit → VP `SwiftUIVideoView`. Glass-to-glass ~200–260 ms.
2. **Head pose** (VP → Orin): VP LiveKit data channel → `lk_data_bridge` ROS 2 node on Orin → `/vp/head_pose` topic → `dm_motor_controller` (mock until DM4310 hooked).
3. **Chassis cmd** (ASUS → Orin → CangJie): ASUS GUI WASD/QE → LiveKit data `{type:"cmd_vel"}` → Orin `cloud_to_ros_bridge` → ROS 2 `/cmd_vel` (over Cyclone DDS on 192.168.8.0/24) → CangJie `chassis_controller` swerve IK → 4 wheels.
4. **Lift cmd** (ASUS → Orin → CangJie): ASUS GUI G/H → LiveKit data `{type:"lift_cmd"}` → Orin `cloud_to_ros_bridge` → ROS 2 `/lift_cmd` → CangJie `chassis_controller` lift section → DM3519 一拖四 (current loop, no MIT). **PR-3 will land this end-to-end (mock first; real CAN when USB-CAN plugged).**

## ROS 2 cross-machine convention

- RMW: `rmw_cyclonedds_cpp` on both Orin and CangJie (`apt install ros-humble-rmw-cyclonedds-cpp`).
- Each machine has `/home/<user>/cyclonedds.xml` pinning the Rain LAN interface (eno1 on Orin, eth0 on CangJie):

  ```xml
  <CycloneDDS xmlns="https://cdds.io/config">
    <Domain Id="any">
      <General>
        <Interfaces>
          <NetworkInterface name="eno1"/>  <!-- or eth0 -->
        </Interfaces>
        <AllowMulticast>default</AllowMulticast>
      </General>
    </Domain>
  </CycloneDDS>
  ```

- `~/.bashrc` exports `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` + `CYCLONEDDS_URI=file:///home/<user>/cyclonedds.xml`.
- Why pin: on dual-homed hosts (WiFi + Ethernet) without pinning, Cyclone picks the first `UP` iface and silently fails cross-machine multicast.
- ⚠️ **Known debt**: 5 systemd user services on Orin (`zed-hw-gst`, `zed-hw-lk`, `teleop-lk-data-bridge`, `teleop-safety`, `teleop-dm-motor`) do NOT yet have `Environment=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in their unit files — they read defaults, so they fall back to Fast DDS and are isolated from CangJie's Cyclone world. Topics they publish (`/vp/head_pose`, `/dm_motor/*`, `/arm/*`) are NOT visible to CangJie until the unit files are updated. Will be addressed in a future PR.

## Network topology

- **Home LAN** `192.168.0.0/24`: Mac mini, MBA 2 (212), Linux 5090 dev box (181), ASUS, Orin WiFi fallback, CangJie WiFi fallback. Gateway `.0.1`.
- **Rain LAN** (network island, since 2026-04-26) `192.168.8.0/24`: Huawei CPE 5S (gateway `.8.1`, 5G uplink, DHCP source) → gigabit unmanaged switch → Orin eno1 + CangJie eth0. Spare ports for laptop debug etc.
- The two LANs do NOT share L3. Mac mini reaches Orin/CangJie via Tailscale or WiFi-fallback IPs.
- **Tailscale** mesh tailnet `thedaosheng@`: Mac mini `100.82.200.67`, Orin `100.75.76.30`, CangJie `100.70.149.16`, ASUS `100.103.0.5`, 212 `100.72.16.46`.
