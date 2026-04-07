# 交接 Prompt — 修复 visionOS 26 立体视频沉浸式渲染黑屏

## 任务总述

我们已经实现了 Linux → 云端 LiveKit Server → Apple Vision Pro 的完整视频传输链路。**视频帧已经成功送到 VP 的 VideoFrameHandler**，但 **CompositorServices + Metal 自定义渲染管线在 visionOS 26 上一直黑屏**。

你的任务是：**修好 CompositorServices Metal 渲染**，让用户进入 ImmersiveSpace 后能看到立体视频，左眼看左半 SBS，右眼看右半 SBS，画面跟着头动。

---

## 三端环境信息（必读）

### A 端 — Linux 工控机（推流端）
- **当前 shell 用户**：`rhz` (执行 agent 的本机)
- **IP**：`192.168.0.181`（局域网）/ `36.110.x.x`（公网 srflx）
- **Sender 脚本**：`/home/rhz/teleop/scripts/20260406-cc-zedmini_livekit_sender.py`
- **当前运行状态**：用 systemd 后台跑着
  ```bash
  # 查看状态
  systemctl --user status lk-s2
  # 看日志
  journalctl --user -u lk-s2 --no-pager -n 20
  # 重启
  systemctl --user stop lk-s2
  systemd-run --user --unit=lk-s3 /usr/bin/python3 -u /home/rhz/teleop/scripts/20260406-cc-zedmini_livekit_sender.py
  ```
- **正常日志**：
  ```
  [GStreamer] 摄像头已启动
  [LiveKit] VideoSource 已创建: 1344x376
  [LiveKit] VideoTrack 已创建: zed-mini-stereo
  [GStreamer] 帧 #301: 2021376 bytes (期望 2021376), 1344x376 RGBA
  [LiveKit] 推帧 #301: 1344x376 RGBA, 平均 60.0 fps, 运行 5.0s
  ```
- **依赖**：
  - 系统 Python（`/usr/bin/python3`，**不能用 conda**）
  - GStreamer 1.20+ + python3-gi
  - `livekit` `livekit-api` `numpy` (pip)
  - ZED Mini 在 `/dev/video0`
  - NVIDIA GPU 用于 NVENC 硬编码

### B 端 — 阿里云 LiveKit Server（云端中继）
- **IP**：`39.102.113.104`
- **SSH**：`ssh root@39.102.113.104`（已配置 SSH key 免密）
- **LiveKit 版本**：1.10.1
- **配置文件**：`/etc/livekit.yaml`
- **启动命令**：
  ```bash
  ssh root@39.102.113.104 "ss -tlnp | grep 7880"  # 检查在不在跑
  ssh root@39.102.113.104 "tail -30 /tmp/livekit.log"  # 查日志
  
  # 重启
  ssh root@39.102.113.104 "pkill livekit-server; sleep 1; \
    nohup livekit-server --config /etc/livekit.yaml --node-ip 39.102.113.104 \
    > /tmp/livekit.log 2>&1 &"
  ```
- **端口**：
  - TCP 7880 — WebSocket 信令
  - TCP 7881 — RTC TCP fallback
  - UDP 50000-60000 — RTC UDP（视频数据走这里）
  - 阿里云安全组上述端口已放行
- **API Key/Secret**（在 `/etc/livekit.yaml` 里）：
  - Key: `teleop_key`
  - Secret: `teleop_secret_key_2026`
- **房间名**：`teleop-room`

### C 端 — Mac + Apple Vision Pro（接收/渲染端）
- **Mac IP**：`192.168.0.212`
- **SSH**：`ssh Zhuanz@192.168.0.212`，密码 `Wawjxyz3!`
  ```python
  # 推荐用 pexpect SSH（密码登录）
  import pexpect, time
  c = pexpect.spawn('ssh Zhuanz@192.168.0.212', timeout=30)
  c.expect('assword')
  c.sendline('Wawjxyz3!')
  ```
  注意：scp 已经配好 SSH key 免密，可以直接 `scp file Zhuanz@192.168.0.212:/path` 不用密码
- **Xcode**：26.4
- **Swift**：6.3
- **visionOS**：26.4 (真机)
- **项目路径**：`/Users/Zhuanz/teleop/RemoteControl/`
- **关键 Swift 文件**：
  - `RemoteControl/Rendering/StereoVideoRenderer.swift` — **要修的文件**
  - `RemoteControl/Rendering/VideoShaders.metal` — **可能要修的 shader**
  - `RemoteControl/RemoteControlApp.swift` — App 入口
  - `RemoteControl/LiveKit/LiveKitManager.swift` — LiveKit 连接管理（不要碰）
  - `RemoteControl/LiveKit/LiveKitContentView.swift` — 主 UI（不要碰）
  - `RemoteControl/WebRTC/VideoFrameHandler.swift` — LiveKit VideoRenderer 适配（不要碰）

### LiveKit 连接参数（VP 端代码硬编码的）
```swift
url:      "ws://39.102.113.104:7880"
roomName: "teleop-room"
identity: "vision-pro-receiver"
token:    "eyJhbGci...NouDG7Xh-cu_F050stjbIYc0QKWoOh8MSfQvlbdJ18Q"  // 30天有效，硬编码在 LiveKitManager.swift
```

---

## 三端验证流程

### 1. 验证 Linux sender 在推流
```bash
journalctl --user -u lk-s2 --no-pager -n 5
# 应看到 "推帧 #xxx" 60fps
```

### 2. 验证 LiveKit Server 收到了 sender
```bash
ssh root@39.102.113.104 "grep 'participant active' /tmp/livekit.log | tail -3"
# 应看到 zed-mini-sender participant active
```

### 3. 验证 Python receiver 能解码（不碰 VP 也能测）
```bash
/usr/bin/python3 -u -c "
import asyncio
from livekit import rtc, api

async def main():
    token = api.AccessToken('teleop_key', 'teleop_secret_key_2026')
    token.with_identity('test-receiver')
    token.with_grants(api.VideoGrants(room_join=True, room='teleop-room'))
    room = rtc.Room()
    cnt = [0]

    @room.on('track_subscribed')
    def on_track(track, pub, participant):
        print(f'Subscribed: {participant.identity}')
        asyncio.ensure_future(read(track))

    async def read(track):
        stream = rtc.VideoStream(track)
        async for event in stream:
            cnt[0] += 1
            if cnt[0] >= 60:
                print(f'OK: 收到 {cnt[0]} 帧 ({event.frame.width}x{event.frame.height})')
                return

    await room.connect('ws://39.102.113.104:7880', token.to_jwt())
    await asyncio.sleep(10)
    await room.disconnect()

asyncio.run(main())
" 2>&1 | grep -v "Decode Error"
```
预期输出：`OK: 收到 60 帧 (1344x376)`

### 4. 验证 VP 能收到视频（用窗口模式）
临时把 `RemoteControlApp.swift` 中的 `LiveKitContentView()` 改成 `LiveKitSimpleTestView()`，编译运行。普通窗口应该显示双眼可见的视频。这一步是确认 VP 端 LiveKit 数据通路 OK。

确认后改回 `LiveKitContentView()` 继续修 immersive 渲染。

### 5. 修 immersive 渲染（你的核心任务）
改完后：
```bash
# 编译验证
ssh Zhuanz@192.168.0.212 "cd /Users/Zhuanz/teleop/RemoteControl && \
  xcodebuild -scheme RemoteControl -destination generic/platform=visionOS build 2>&1 | \
  grep 'error:' | head -10"
# 必须 0 个 Swift error（CodeSign 失败可以忽略）

# 然后告诉用户在 Xcode 里 Run
```

---

## 当前状态（什么是已经验证可工作的）

| 测试 | 结果 |
|------|------|
| Linux sender 推流到 LiveKit Server | ✅ 1344x376 @60fps 稳定 |
| Python receiver 订阅并解码 | ✅ 100% 帧解码成功 |
| VP 用 LiveKit `SwiftUIVideoView` 在普通窗口显示 | ✅ 双眼可见视频 |
| VP 用 RealityKit 在 ImmersiveSpace 显示视频 | ✅ 视频在平面上，但不是真正 per-eye |
| **VP 用 CompositorServices Metal 渲染** | ❌ **黑屏** |

**视频数据通路 100% 通了**。问题只在最后一步：把 CVPixelBuffer NV12 帧用 Metal 渲染到 visionOS 26 的 layered 纹理上。

---

## 不要做的事情

1. **不要改 LiveKit 部分** —— 网络、解码、信令、Pose 回传都没问题
2. **不要回到自建 WebRTC** —— GStreamer webrtcbin 和 aiortc 都试过了，DTLS/SRTP 不兼容
3. **不要用 RealityKit 平面方案做 per-eye** —— RealityKit 没有 per-eye visibility API
4. **不要让用户 Build** —— 必须自己 SSH 到 Mac 用 xcodebuild 验证

---

## 已经尝试过的修复（都没解决黑屏）

1. ✅ 设置 `rasterizationRateMap`（foveation 必需）
2. ✅ 设置 `renderTargetArrayLength` 匹配 layered 布局
3. ✅ 设置 `maxVertexAmplificationCount = 2`
4. ✅ 设置 `inputPrimitiveTopology = .triangle`
5. ✅ `setVertexAmplificationCount(2, viewMappings: ...)` 配置左右眼 viewMapping
6. ✅ `frame.startUpdate()` / `endUpdate()` / `startSubmission()` / `endSubmission()` 正确调用
7. ✅ `drawable.encodePresent(commandBuffer:)` 在 commit 之前调用
8. ✅ Pipeline `colorFormat`/`depthFormat` 从 `layerRenderer.configuration` 读取
9. ✅ 用 `layerRenderer.device` 而不是 `MTLCreateSystemDefaultDevice()`
10. ✅ 用 `timing.trackableAnchorTime.timeInterval` 查 device anchor
11. ✅ Depth state: `compareFunction = .always`, `isDepthWriteEnabled = false`
12. ✅ Cull mode: `.none`
13. ✅ Quad 顶点是 NDC `[-1,1]`，覆盖整个 viewport

**即使把 fragment shader 改成 `return float4(1, 0, 0, 1)`（纯红）和 clear color 设为红色，也是黑屏。**

---

## 关键日志（当前正常运行时打印）

```
[Pipeline] viewCount=2 maxAmp=2 color=81 depth=252 layout=layered foveation=true
[Pipeline] Created OK
[Texture] #1 Y:OK CbCr:OK planes=1344x376+672x188 ioSurface=true full=true
[Render] #1 drawable0/1 views=2 anchor=true video=true rrm=1 colorArr=2
[Render] DRAW #1:
  colorTex 1888x1792 type=3 arr=2
  vp0 x=0.0 y=0.0 w=4338.0 h=3478.0
  vp1 x=0.0 y=0.0 w=4338.0 h=3478.0
  rrm=SET
  Y    1344x376 fmt=10
  CbCr 672x188 fmt=30
  L oX=0.037499994 sX=0.425 oY=0.0 sY=1.0
  R oX=0.5375 sX=0.425 oY=0.0 sY=1.0
  full=1
```

注意：
- `viewport (4338x3478)` >> `colorTex (1888x1792)` —— foveation 注视点渲染特性
- `colorTex type=3` = `MTLTextureType.type2DArray`
- `colorTex arr=2` = 2 个 slice（左右眼）
- `anchor=true video=true` 都正常

---

## 可能的根因（按可能性排序）

### 1. 渲染目标 slice 没写对 ⭐⭐⭐
现在 vertex shader 输出 `[[render_target_array_index]] = amplification_id`，但可能 visionOS 26 需要不同方式指定 slice。

**调试方向**：尝试不写 `[[render_target_array_index]]`，改用 `setVertexAmplificationCount` 的 `renderTargetArrayIndexOffset` 自动路由。或者显式设置 `rpd.colorAttachments[0].slice = 0` + `renderTargetArrayLength = drawable.views.count`。

### 2. depth attachment 配置错误 ⭐⭐
使用了 `drawable.depthTextures[0]`，但可能也需要设置 `slice` 和 `renderTargetArrayLength`。

### 3. visionOS 26 ImmersiveSpace 配置问题 ⭐⭐
当前用 `.immersionStyle(.full, in: .full)`。可能需要 `.mixed` 或不同配置。

### 4. encodePresent 在 visionOS 26 上的语义变了 ⭐
官方 Compositor Services template 可能对 visionOS 26 有不同的提交方式。

---

## 调试 / 修复步骤建议

### Step 1: 找 Apple 官方 Compositor Services Metal sample 仔细对比

Apple 在 Xcode 26.4 内置了模板：
```bash
ssh Zhuanz@192.168.0.212 "find /Applications/Xcode.app -path '*Compositor*' -name '*.swift' 2>/dev/null"
```
应该能找到：
```
/Applications/Xcode.app/Contents/Developer/Library/Xcode/Templates/Project Templates/MultiPlatform/Application/Compositor Services.xctemplate/
```

仔细对比模板和我们的代码的差异。**模板能跑通，我们跑不通，差异就是 bug 所在**。

### Step 2: 简化到极致

写一个最小的 demo：去掉所有视频纹理，去掉 vertex amplification，就一个三角形 + 红色 fragment shader。如果连这个都黑屏，问题在 CompositorLayer 配置或 ImmersiveSpace 配置。如果三角形显示了，再逐步加回 vertex amplification。

### Step 3: 对比 RealityKit 沉浸方案

`controller/visionos-native/RemoteControl/LiveKit/RealityKitVideoRenderer.swift` 是已知能在 ImmersiveSpace 显示视频的 RealityKit 实现。临时把 App 入口指向它（在 `RemoteControlApp.swift` 中），看 ImmersiveSpace 配置是不是问题。

---

## 成功标准

进入 Immersive Space 后：
1. **看到视频画面**（不是黑屏）
2. **左眼看到 SBS 视频的左半部分**
3. **右眼看到 SBS 视频的右半部分**
4. **画面跟着头部转动**（CompositorServices + ARKit deviceAnchor 自动处理）

加分项：
- 进入 immersive 流畅，没有几秒的黑屏延迟
- 左右眼对齐良好，有立体感
- 60 fps 稳定不卡顿
