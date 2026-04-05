# Remote Control - Teleop WebRTC System

Apple Vision Pro + Linux + Cloud WebRTC teleoperation system.

Linux side-by-side stereo camera (ZED Mini) streams to Apple Vision Pro via WebRTC through a cloud signaling/TURN server, with real-time head pose feedback.

## Architecture

```
Linux (Device A)                    Cloud                     Controller (Vision Pro / Browser)
+-----------------+          +------------------+          +-------------------+
| ZED Mini Camera |          | Signaling Server |          | VP Native App     |
| GStreamer + NVENC| ------→ | ws://8765        | ------→  | LiveKit WebRTC    |
| WebRTC Sender   |          | coturn TURN 3478 |          | Metal Renderer    |
+-----------------+          +------------------+          +-------------------+
                                                           | HTML Receiver     |
                                     ↑                     | WebXR / Canvas    |
                                     |                     +-------------------+
                                     |
                              Head Pose (30Hz)
                              quaternion + position
```

## Directory Structure

```
.
├── device-a-linux/          # A-side: Linux robot (被控制端)
│   ├── zedmini_webrtc_sender.py     # GStreamer webrtcbin sender (production)
│   └── zedmini_aiortc_sender.py     # aiortc sender (alternative)
│
├── cloud/                   # Cloud signaling + TURN
│   ├── signaling_server.py          # WebSocket signaling server v9
│   └── turnserver.conf              # coturn TURN configuration
│
├── controller/              # Controller side (操控端)
│   ├── html-receiver/               # Browser-based WebXR receiver
│   │   └── visionpro_stereo_receiver.html
│   ├── visionos-native/             # Vision Pro native app (Xcode)
│   │   └── RemoteControl/
│   └── clash-config/                # Clash Verge TUN bypass config
│       └── Merge.yaml
```

## Quick Start

### 1. Cloud Server

```bash
# Start signaling server
python3 -u signaling_server.py

# coturn should be running as systemd service
systemctl status coturn
```

Server: `39.102.113.104`, WS port: `8765`, TURN port: `3478`

### 2. Linux Sender (Device A)

```bash
# Must use system Python (not conda)
/usr/bin/python3 -u zedmini_webrtc_sender.py
```

Requires: ZED Mini camera, NVIDIA GPU (nvh264enc), GStreamer 1.20+

### 3. Controller

**Option A: Browser (Mac/Windows)**
- Serve `visionpro_stereo_receiver.html` via HTTPS
- Open in Vision Pro Safari or desktop browser

**Option B: Vision Pro Native App**
- Open `RemoteControl.xcodeproj` in Xcode
- Build & run on Vision Pro device

### Clash Verge TUN Bypass

If using Clash Verge with TUN mode, the `Merge.yaml` config ensures WebRTC UDP traffic bypasses the virtual NIC. Without this, TURN allocation and DTLS handshake will fail.

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Video | 1344x376 SBS, 60fps, H.264 8Mbps |
| Signaling | `ws://39.102.113.104:8765` |
| TURN | `39.102.113.104:3478` (user: `remote`) |
| Latency | ~50-90ms end-to-end |
