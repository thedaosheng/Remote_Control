# 交接 Prompt — 修复 visionOS 26 立体视频沉浸式渲染黑屏

## 任务总述

我们已经实现了 Linux → 云端 LiveKit Server → Apple Vision Pro 的完整视频传输链路。**视频帧已经成功送到 VP 的 VideoFrameHandler**，但 **CompositorServices + Metal 自定义渲染管线在 visionOS 26 上一直黑屏**。

你的任务是：**修好 CompositorServices Metal 渲染**，让用户进入 ImmersiveSpace 后能看到立体视频，左眼看左半 SBS，右眼看右半 SBS，画面跟着头动。

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

1. **不要改 LiveKit 部分** —— 网络、解码、信令、Pose 回传都没问题，不要碰
2. **不要回到自建 WebRTC 方案** —— GStreamer webrtcbin 和 aiortc 都试过了，DTLS/SRTP 不兼容
3. **不要用 RealityKit 平面方案做 per-eye** —— RealityKit 没有 per-eye visibility API，两个平面会互相争抢
4. **不要让用户去 Build** —— 必须自己 SSH 到 Mac 用 xcodebuild 验证编译通过

---

## 已经尝试过的修复（都没解决黑屏）

1. ✅ 设置 `rasterizationRateMap`（foveation 注视点渲染必需）
2. ✅ 设置 `renderTargetArrayLength` 匹配 layered 布局
3. ✅ 设置 `maxVertexAmplificationCount = 2`
4. ✅ 设置 `inputPrimitiveTopology = .triangle`（vertex shader 写 `[[render_target_array_index]]` 必需）
5. ✅ `setVertexAmplificationCount(2, viewMappings: ...)` 配置左右眼 viewMapping
6. ✅ `frame.startUpdate()` / `endUpdate()` / `startSubmission()` / `endSubmission()` 正确调用
7. ✅ `drawable.encodePresent(commandBuffer:)` 在 commit 之前调用
8. ✅ Pipeline 配置 `colorFormat`/`depthFormat` 从 `layerRenderer.configuration` 读取（不硬编码）
9. ✅ 用 `layerRenderer.device` 而不是 `MTLCreateSystemDefaultDevice()`
10. ✅ 用 `timing.trackableAnchorTime.timeInterval` 查 device anchor（不是 `CACurrentMediaTime`）
11. ✅ Depth state: `compareFunction = .always`, `isDepthWriteEnabled = false`
12. ✅ Cull mode: `.none`，`frontFacing: .counterClockwise`
13. ✅ Quad 顶点是 NDC `[-1,1]`，覆盖整个 viewport

**即使把 fragment shader 改成 `return float4(1, 0, 0, 1)`（纯红）和 clear color 设为红色，也是黑屏。**

---

## 关键日志（正常运行时打印）

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
- `viewport (4338x3478)` >> `colorTex (1888x1792)` —— foveation 注视点渲染特性，rasterizationRateMap 负责映射
- `colorTex type=3` = MTLTextureType.type2DArray
- `colorTex arr=2` = 2 个 slice（左右眼）
- `anchor=true video=true` 都正常

---

## 可能的根因（按可能性排序）

### 1. 渲染目标 slice 没写对 ⭐⭐⭐
现在用 vertex shader 输出 `[[render_target_array_index]] = amplification_id`，但可能 visionOS 26 需要不同的方式指定 slice。

**调试方向**：在 setRenderPassDescriptor 时显式设置 `rpd.colorAttachments[0].slice = 0` 和 `renderTargetArrayLength = drawable.views.count`，让 GPU 知道完整的 slice 范围。或者尝试不写 `[[render_target_array_index]]`，改用 `setVertexAmplificationCount` 的 `renderTargetArrayIndexOffset` 自动路由。

### 2. depth attachment 配置错误 ⭐⭐
使用了 `drawable.depthTextures[0]`，但可能也需要设置 `slice` 和 `renderTargetArrayLength`。

### 3. visionOS 26 ImmersiveSpace 配置问题 ⭐⭐
当前用 `.immersionStyle(.full, in: .full)`。可能需要 `.mixed` 或不同的配置才能让 CompositorServices 内容显示。

### 4. encodePresent 在 visionOS 26 上的语义变了 ⭐
官方 Compositor Services template 可能对 visionOS 26 有不同的提交方式。

### 5. SwiftUI 上层有遮挡 ⭐
某种 SwiftUI 视图把 immersive content 遮住了。

---

## 调试 / 修复步骤建议

### Step 1: 复现并确认黑屏

参考 `controller/visionos-native/RemoteControl/Rendering/StereoVideoRenderer.swift` 当前实现（有非常详细的诊断日志）。

### Step 2: 找一个已知能工作的 visionOS 26 Compositor Services Metal sample

Apple 在 Xcode 26.4 内置了模板：
```
/Applications/Xcode.app/Contents/Developer/Library/Xcode/Templates/Project Templates/MultiPlatform/Application/Compositor Services.xctemplate/
```

仔细对比这个模板和我们的代码的差异。**模板能跑通，我们跑不通，差异就是 bug 所在**。

### Step 3: 简化到极致

写一个最小的 demo：去掉所有视频纹理，去掉 vertex amplification，就一个三角形 + 红色 fragment shader。如果连这个都黑屏，问题在 CompositorLayer 配置或 ImmersiveSpace 配置。如果三角形显示了，再逐步加回 vertex amplification。

### Step 4: 用 Xcode GPU Frame Capture

让用户在 Xcode 里点 "Capture GPU Frame"，能看到每个 draw call 的输入输出。这是最直接的诊断工具。但需要用户操作。

---

## 环境信息

- **Mac**: `Zhuanz@192.168.0.212`，密码 `Wawjxyz3!`（pexpect SSH）
- **VP**: 真机连在 Mac 上，Xcode 26.4，visionOS 26.4
- **项目路径**: `/Users/Zhuanz/teleop/RemoteControl/`
- **关键文件**:
  - `/Users/Zhuanz/teleop/RemoteControl/RemoteControl/Rendering/StereoVideoRenderer.swift`
  - `/Users/Zhuanz/teleop/RemoteControl/RemoteControl/Rendering/VideoShaders.metal`
  - `/Users/Zhuanz/teleop/RemoteControl/RemoteControl/RemoteControlApp.swift`

- **Linux 工控机**: `192.168.0.181`（已经在跑 sender，无需操作）
- **LiveKit Server**: `39.102.113.104:7880`（已部署运行中）

---

## 编译验证（每次改完代码必做）

```bash
ssh Zhuanz@192.168.0.212 "cd /Users/Zhuanz/teleop/RemoteControl && \
  xcodebuild -scheme RemoteControl -destination generic/platform=visionOS build 2>&1 | \
  grep 'error:' | head -10"
```

只要 Swift `error:` 数量为 0 就算编译通过（CodeSign 失败可以忽略，那是命令行没有签名权限）。

**绝对不要让用户去 Build**。每次改完代码都自己 xcodebuild 验证，验证通过才告诉用户 Run。

---

## 测试流程

1. 你修完 StereoVideoRenderer.swift / VideoShaders.metal
2. SCP 上传到 Mac
3. xcodebuild 验证编译
4. 告诉用户在 Xcode 里 Run 到真机
5. 用户进入 Immersive Space
6. 用户拍照/描述看到了什么
7. 根据反馈继续迭代

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
