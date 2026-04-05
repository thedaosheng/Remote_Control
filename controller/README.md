# Controller (操控端)

Receives stereo video from Linux sender and renders on Apple Vision Pro or browser.

## Options

### 1. HTML WebXR Receiver (`html-receiver/`)

Browser-based receiver with WebXR stereo rendering support.

- Works on Vision Pro Safari (WebXR immersive mode)
- Falls back to side-by-side canvas on desktop browsers
- Serve via HTTPS (self-signed cert OK):

```bash
cd html-receiver
python3 -c "
import http.server, ssl
server = http.server.HTTPServer(('0.0.0.0', 8443), http.server.SimpleHTTPRequestHandler)
ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
ctx.load_cert_chain('cert.pem', 'key.pem')
server.socket = ctx.wrap_socket(server.socket, server_side=True)
print('https://0.0.0.0:8443')
server.serve_forever()
"
```

### 2. Vision Pro Native App (`visionos-native/`)

Native visionOS app using CompositorServices + Metal for stereo rendering.

- Open `RemoteControl.xcodeproj` in Xcode
- Target: Apple Vision Pro device
- Uses LiveKitWebRTC framework for WebRTC
- Metal shaders for NV12 → RGB conversion
- ARKit WorldTrackingProvider for head pose

### 3. Clash Verge Config (`clash-config/`)

**Critical for Linux sender.** If Clash Verge TUN mode is enabled on the sender machine, WebRTC UDP traffic (TURN, DTLS, SRTP) gets intercepted by the virtual NIC, breaking connectivity.

Copy `Merge.yaml` to `~/.config/clash-verge-rev/Merge.yaml` and restart Clash Verge.

This excludes TURN server and local network from TUN routing, allowing WebRTC to work properly.
