# Device A - Linux Sender (被控制端)

ZED Mini stereo camera capture + NVENC H.264 encoding + WebRTC streaming.

## Files

| File | Description |
|------|-------------|
| `zedmini_webrtc_sender.py` | GStreamer webrtcbin sender (production, v7.3) |
| `zedmini_aiortc_sender.py` | aiortc pure-Python sender (alternative) |

## Requirements

- ZED Mini camera (`/dev/video0`)
- NVIDIA GPU with nvh264enc support
- System Python 3.10+ (`/usr/bin/python3`, NOT conda)
- GStreamer 1.20+ with plugins: `gst-plugins-bad`, `gst-plugins-ugly`, `gstreamer1.0-nice`
- Python packages: `websockets`, `PyGObject` (gi)

## Usage

```bash
/usr/bin/python3 -u zedmini_webrtc_sender.py
```

## Pipeline

```
v4l2src (ZED Mini 1344x376 YUY2 60fps)
  → videoconvert → I420
  → videoflip rotate-180
  → nvh264enc (8Mbps CBR, zerolatency)
  → rtph264pay
  → webrtcbin → DTLS-SRTP → ICE → TURN relay
```

## Pose Reception

Receives Vision Pro head pose via signaling server, converts quaternion to euler angles, and forwards to motor controller via UDP (localhost:9000).
