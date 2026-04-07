# Controller — Apple Vision Pro 控制端

Vision Pro 上的 LiveKit WebRTC receiver，订阅来自 Linux 端的立体视频，渲染到沉浸式空间，同时通过 Data Channel 30Hz 回传头部姿态。

## 当前状态

| 模式 | 视频显示 | 状态 |
|------|---------|------|
| **普通窗口（`LiveKitSimpleTestView`）** | ✅ 双眼可见 SBS 视频 | 完全可用 |
| **RealityKit 沉浸（`RealityKitVideoView`）** | ✅ 视频在 RealityKit 平面上 | 可用，但不是真正的 per-eye |
| **CompositorServices Metal 沉浸（`StereoVideoRenderer`）** | ❌ 黑屏（vertex amplification 已配置但渲染未到屏幕） | **未跑通**，待修复 |

**简而言之**：视频数据从 Linux 一路通到 VP 没问题，**最后一步把 SBS 视频用 vertex amplification 真正渲染到左/右眼还没修好**。

## 目录结构

```
controller/
├── README.md
└── visionos-native/
    ├── RemoteControl.xcodeproj/        ← Xcode 项目
    └── RemoteControl/
        ├── RemoteControlApp.swift      ← @main App 入口
        ├── Info.plist
        ├── Models/
        │   └── AppState.swift          ← @Observable 全局状态
        ├── LiveKit/                    ← LiveKit 相关代码
        │   ├── LiveKitManager.swift           ← Room 连接管理 + 视频订阅 + Pose 发送
        │   ├── LiveKitContentView.swift       ← 主 UI（带 Enter Immersive Space 按钮）
        │   ├── LiveKitSimpleTest.swift        ← 简单窗口测试视图（已验证可工作）
        │   ├── RealityKitVideoRenderer.swift  ← RealityKit 平面渲染（备选）
        │   └── LiveKitRemoteControlApp.swift  ← 旧入口（@main 已注释）
        ├── WebRTC/
        │   └── VideoFrameHandler.swift ← LiveKit VideoRenderer 适配器（缓存 CVPixelBuffer）
        ├── Rendering/
        │   ├── StereoVideoRenderer.swift  ← CompositorServices 立体渲染（黑屏待修）
        │   └── VideoShaders.metal         ← Metal NV12→RGB shader + vertex amplification
        ├── Tracking/
        │   └── HeadPoseTracker.swift   ← ARKit WorldTrackingProvider 头部追踪
        └── Assets.xcassets/
```

## 技术栈

- **Xcode 26.4**, Swift 6.3
- **LiveKit Swift SDK 2.12.1** （通过 Swift Package Manager 添加）
- **visionOS 26.0+** target
- **CompositorServices** + **Metal** 用于立体渲染
- **ARKit** WorldTrackingProvider 用于头部追踪
- **RealityKit** 备选渲染方案

## Setup（首次配置）

### 1. 添加 LiveKit Swift SDK 包依赖

在 Xcode 中：
1. File → Add Package Dependencies
2. URL: `https://github.com/livekit/client-sdk-swift`
3. Version: `2.12.1` 或 `Up to Next Major from: 2.0.0`
4. Target: `RemoteControl`

### 2. 删除旧的 LiveKitWebRTC framework

如果项目里有 `Frameworks/LiveKitWebRTC.xcframework`，删掉。它跟新版 LiveKit SDK 提供的 LiveKitWebRTC 冲突会导致 "duplicate output file" 错误。

### 3. Token 配置

`LiveKit/LiveKitManager.swift` 顶部的 `LiveKitConfig` enum 中已经硬编码了一个 30 天有效的 receiver token。
如果过期了，用 `device-a-linux/generate_livekit_tokens.py` 重新生成并替换。

### 4. Build

选 target = 你的 Vision Pro 真机（或 Apple Vision Pro Simulator），点 Run。

## 工作流

```
[App 启动]
  ↓
LiveKitContentView 显示 "Connect" 按钮
  ↓ 用户点击
LiveKitManager.connect()
  → Room.connect(url:token:) 连到 LiveKit Server
  → 监听 didSubscribeTrack 事件
  → 收到视频轨道后调用 track.add(videoRenderer: appState.videoFrameHandler)
  → VideoFrameHandler 实现 LiveKit 的 VideoRenderer 协议
  → render(frame:) 把 CVPixelBuffer 缓存起来
  ↓
连接成功后显示 "Enter Immersive Space" 按钮
  ↓ 用户点击
ImmersiveSpace 打开
  ↓
StereoVideoRenderer.startRenderLoop()
  → 60Hz 渲染循环
  → 从 VideoFrameHandler.consumeLatestFrame() 取最新帧
  → CVMetalTextureCache 创建 Y/CbCr 纹理
  → vertex amplification 单 pass 渲染左右眼
  ↓
30Hz HeadPoseTracker → LiveKitManager.publishPose()
  → room.localParticipant.publish(data:options:.init(reliable:false))
  → 通过 Data Channel UDP 模式发到 sender
```

## 已验证可工作的部分

**用 `LiveKitSimpleTestView` 切换 App 入口**（`RemoteControlApp.swift` 中把 `LiveKitContentView()` 改为 `LiveKitSimpleTestView()`）：
- 普通窗口显示 LiveKit 的 `SwiftUIVideoView`
- 视频画面在窗口中正常显示
- 双眼能看到画面（visionOS 默认窗口是双眼可见的 2D 投影）
- **这证明从 Linux → 云端 → VP 的视频数据通路完全通了**

## 已知问题：CompositorServices 黑屏

详见根目录的 `HANDOFF_PROMPT.md`。简单说就是：
- LiveKit 已经把视频帧 1344x376 NV12 送到 VideoFrameHandler ✓
- StereoVideoRenderer 正确创建了 Metal 纹理 ✓
- Pipeline 创建成功，vertex amplification 配置正确 ✓
- encodePresent 调用了，无报错 ✓
- **但屏幕全黑** —— 即使 shader 返回纯红色或 clear color 设为红色都看不到

## 关键日志（正常运行时应该看到）

```
[LiveKit] ✓ Room 连接成功
[LiveKit] ★ 订阅了 zed-mini-sender 的轨道
[LiveKit] ★ 收到视频轨道！绑定到渲染器
[LiveKit] ✓ Pose 定时器已启动 (30Hz)
[VideoFrame] ★ FIRST FRAME! 1344x376 format=0x34323066 planes=2
[Pipeline] viewCount=2 maxAmp=2 color=81 depth=252 layout=layered foveation=true
[Pipeline] Created OK
[Render] startRenderLoop
[Render] ARKit ready
[Texture] #1 Y:OK CbCr:OK planes=1344x376+672x188 ioSurface=true full=true
[Render] #1 drawable0/1 views=2 anchor=true video=true rrm=1 colorArr=2
```

format `0x34323066` = `'420f'` = `kCVPixelFormatType_420YpCbCr8BiPlanarFullRange`（NV12 全范围）。
