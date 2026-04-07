import Metal
import MetalKit
import CompositorServices
import SwiftUI
import CoreVideo
import ARKit
import simd

// =============================================================================
// CompositorLayer 配置
//   - layered 布局（必须，texture array,每个 slice 对应一只眼睛）
//   - foveation 开启（vision OS 26 vertex amplification 流程依赖 rasterizationRateMap）
//   - depth/color 选用 visionOS 标准格式
// =============================================================================

struct VideoLayerConfiguration: CompositorLayerConfiguration {
    func makeConfiguration(
        capabilities: LayerRenderer.Capabilities,
        configuration: inout LayerRenderer.Configuration
    ) {
        // ★ 关键: 不要显式覆盖 colorFormat 和 depthFormat!
        //    Apple 官方 Compositor Services 模板完全不设置这两个,
        //    让 visionOS 根据设备的显示管线选最佳格式(visionOS 26 上通常是 HDR 浮点)。
        //    之前强制设成 bgra8Unorm_srgb 可能导致 compositor 无法正确呈现 → 黑屏。

        // 必须开启 foveation —— Apple 模板里 layered 渲染依赖 rasterizationRateMap
        let foveationEnabled = capabilities.supportsFoveation
        configuration.isFoveationEnabled = foveationEnabled

        // 查询设备支持的 layout,前提:开启 foveation
        // visionOS 26 + Vision Pro 上 layered 是最高效的布局
        let opts: LayerRenderer.Capabilities.SupportedLayoutsOptions =
            foveationEnabled ? [.foveationEnabled] : []
        let supported = capabilities.supportedLayouts(options: opts)

        // 优先使用 layered(单 pass + vertex amplification),退回 dedicated(两 pass)
        // 注意: Apple 模板用 .dedicated 作为 fallback,而不是 .shared。
        configuration.layout = supported.contains(.layered) ? .layered : .dedicated
    }
}

// =============================================================================
// 全屏四边形顶点定义 (NDC [-1,1]^2 + UV [0,1]^2)
// 单 pass + vertex amplification:
//   一个 4 顶点 quad 同时绘制到左右眼两个 slice
//   shader 内部根据 [[amplification_id]] 选择不同 UV 子区
// =============================================================================

private struct QuadVertex {
    var position: SIMD2<Float>   // 裁剪空间坐标 (NDC)
    var uv:       SIMD2<Float>   // 单位纹理坐标，未做 SBS 偏移
}

// 4 顶点 + 6 索引画一个全屏矩形
// nonisolated 避免渲染线程 (非 MainActor) 引用时的 isolation warning
nonisolated private let quadVertices: [QuadVertex] = [
    QuadVertex(position: SIMD2(-1,  1), uv: SIMD2(0, 0)),   // 左上
    QuadVertex(position: SIMD2( 1,  1), uv: SIMD2(1, 0)),   // 右上
    QuadVertex(position: SIMD2(-1, -1), uv: SIMD2(0, 1)),   // 左下
    QuadVertex(position: SIMD2( 1, -1), uv: SIMD2(1, 1)),   // 右下
]

nonisolated private let quadIndices: [UInt16] = [0, 2, 1,  1, 2, 3]

// =============================================================================
// EyeUniforms —— 一次传左右眼两组 UV 变换 + YCbCr range 标志
// 必须与 VideoShaders.metal 中的 EyeUniforms / EyePairUniforms 完全匹配。
// =============================================================================

private struct EyeParams {
    var uvOffsetX: Float       // SBS 纹理水平偏移
    var uvScaleX:  Float       // 水平缩放（= 0.5 * cal.scale）
    var uvOffsetY: Float       // 垂直偏移
    var uvScaleY:  Float       // 垂直缩放
    var pad0: Float = 0
    var pad1: Float = 0
    var pad2: Float = 0
    var pad3: Float = 0        // 16 字节对齐
}

private struct EyePairUniforms {
    var eyes:        (EyeParams, EyeParams)   // 0 = 左眼, 1 = 右眼 —— 对应 [[amplification_id]]
    var isFullRange: UInt32                   // 1 = full range, 0 = video range BT.601
    var pad0: UInt32 = 0
    var pad1: UInt32 = 0
    var pad2: UInt32 = 0                       // 对齐到 16 字节
}

// =============================================================================
// ViewProjectionUniforms —— 每帧更新,给 vertex shader 做 world → clip 变换。
//
// 这是修好"黑屏"的关键 ——
//   visionOS Compositor 对每个 drawable 必须有 deviceAnchor,它用 deviceAnchor
//   做 reprojection(motion-to-photon warp)。reprojection 的前提是:顶点输出
//   的 clip 坐标必须代表真实的 world-space 位置(通过 view * projection 变换得到)。
//   如果我们只输出 raw NDC, compositor 会把这些像素当成"某个未知 world 位置",
//   用 deviceAnchor 做 warp 后直接把内容 warp 到相机视野外 → 黑屏。
//
// 做法与 Apple 官方 Compositor Services 模板完全一致:
//   viewMatrix   = (deviceAnchor.originFromAnchorTransform * view.transform).inverse
//   projMatrix   = drawable.computeProjection(viewIndex:)
//   vp           = projMatrix * viewMatrix
//   顶点输出     = vp * modelMatrix * localPos
// =============================================================================

private struct ViewProjectionUniforms {
    var vpLeft:  simd_float4x4   // 左眼 world → clip
    var vpRight: simd_float4x4   // 右眼 world → clip
    var model:   simd_float4x4   // quad local → world (含头位姿 + 前向平移 + 缩放)
}

// 单位矩阵平移/缩放的便捷构造(Apple 模板里也有这个模式)
nonisolated private func matrix4x4_translation(_ x: Float, _ y: Float, _ z: Float) -> simd_float4x4 {
    return simd_float4x4(
        SIMD4<Float>(1, 0, 0, 0),
        SIMD4<Float>(0, 1, 0, 0),
        SIMD4<Float>(0, 0, 1, 0),
        SIMD4<Float>(x, y, z, 1)
    )
}

nonisolated private func matrix4x4_scale(_ sx: Float, _ sy: Float, _ sz: Float) -> simd_float4x4 {
    return simd_float4x4(
        SIMD4<Float>(sx,  0,  0, 0),
        SIMD4<Float>( 0, sy,  0, 0),
        SIMD4<Float>( 0,  0, sz, 0),
        SIMD4<Float>( 0,  0,  0, 1)
    )
}

// =============================================================================
// StereoVideoRenderer —— CompositorServices + Metal + Vertex Amplification
//
// 渲染流程（每帧）：
//   1. queryNextFrame -> startUpdate -> startSubmission
//   2. queryDrawables 拿到 1 个 LayerRenderer.Drawable（builtIn）
//   3. 设置 deviceAnchor（让 compositor 做 reprojection,使画面跟头动）
//   4. 单个 render pass:
//        - colorAttachment[0] = drawable.colorTextures[0]   (type2DArray, 2 slice)
//        - depthAttachment    = drawable.depthTextures[0]
//        - rasterizationRateMap = drawable.rasterizationRateMaps.first  (foveation)
//        - renderTargetArrayLength = 2                       (layered)
//   5. setViewports([leftViewport, rightViewport])           (复数)
//   6. setVertexAmplificationCount(2, viewMappings: [...])
//        其中两个 viewMapping 的 viewportArrayIndexOffset/renderTargetArrayIndexOffset 分别 = 0 / 1
//   7. drawIndexedPrimitives 一次提交 6 个索引 -> GPU 自动放大成 2 份分别写到两个 slice
//   8. encodePresent / commit / endSubmission
//
// shader 内部:
//   vertex   shader 接收 [[amplification_id]] uint amp_id, 用 amp_id 索引 EyePairUniforms.eyes
//                  并输出 [[render_target_array_index]] = amp_id 让 GPU 写到对应 slice
//   fragment shader 用插值后的 uv（已被 SBS 偏移过）采 NV12 双平面 -> RGB
// =============================================================================

final class StereoVideoRenderer: @unchecked Sendable {

    // ---- 注入依赖 ----
    // LayerRenderer 是 ObjC 桥接类型,默认推断为 MainActor 隔离
    // 但渲染线程必须能访问它,所以标 nonisolated(unsafe)
    private nonisolated(unsafe) let layerRenderer: LayerRenderer
    private let frameHandler:  VideoFrameHandler

    // ---- Metal ----
    // 所有 nonisolated 的属性都用 nonisolated(unsafe),因为本类是 @unchecked Sendable
    // 渲染线程在专用 Thread 里跑,所有访问都是单线程的,不会有竞态
    private let device:                              MTLDevice
    private let commandQueue:                        MTLCommandQueue
    private nonisolated(unsafe) var pipelineState:   MTLRenderPipelineState?
    private nonisolated(unsafe) var depthState:      MTLDepthStencilState?
    private nonisolated(unsafe) var vertexBuffer:    MTLBuffer?
    private nonisolated(unsafe) var indexBuffer:     MTLBuffer?
    private nonisolated(unsafe) var textureCache:    CVMetalTextureCache?

    // ---- ARKit (头部位姿) ----
    private let arSession                       = ARKitSession()
    private let worldTracking                   = WorldTrackingProvider()
    private nonisolated(unsafe) var arReady     = false

    // ---- 视频纹理状态 ----
    private nonisolated(unsafe) var yTexture:           MTLTexture?
    private nonisolated(unsafe) var cbcrTexture:        MTLTexture?
    private nonisolated(unsafe) var lastBuffer:         CVPixelBuffer?
    private nonisolated(unsafe) var currentIsFullRange: Bool = true

    // ---- 调试统计 ----
    private nonisolated(unsafe) var renderCount:     Int = 0
    private nonisolated(unsafe) var textureLogCount: Int = 0

    // -------------------------------------------------------------------------
    // init —— 构建 Metal 资源、初始化 ARKit
    // -------------------------------------------------------------------------
    init(layerRenderer: LayerRenderer, frameHandler: VideoFrameHandler) {
        self.layerRenderer = layerRenderer
        self.frameHandler  = frameHandler

        // visionOS 26: LayerRenderer 自带 device,直接用避免多 device 不一致
        self.device = layerRenderer.device
        guard let q = device.makeCommandQueue() else {
            fatalError("Cannot create Metal command queue")
        }
        self.commandQueue = q

        // 构建 pipeline / 顶点 buffer / 纹理 cache
        buildPipeline()
        buildDepthStencilState()
        buildQuadBuffers()
        CVMetalTextureCacheCreate(nil, nil, device, nil, &textureCache)
    }

    // -------------------------------------------------------------------------
    // Pipeline 构建 —— 关键：maxVertexAmplificationCount = viewCount(=2)
    // 没有这一行，下面的 setVertexAmplificationCount(2, ...) 会运行时报错
    // -------------------------------------------------------------------------
    private func buildPipeline() {
        guard let lib = device.makeDefaultLibrary() else {
            print("[Pipeline] FATAL: no default Metal library")
            return
        }
        let vs = lib.makeFunction(name: "vs_video")
        let fs = lib.makeFunction(name: "fs_video_nv12")

        // 顶点描述符:
        //   attribute(0) = position (float2) at offset 0
        //   attribute(1) = uv       (float2) at offset 8
        //   两者共享 buffer(1) (与 setVertexBuffer(buffer, offset: 0, index: 1) 对应)
        let vd = MTLVertexDescriptor()
        vd.attributes[0].format      = .float2
        vd.attributes[0].offset      = 0
        vd.attributes[0].bufferIndex = 1
        vd.attributes[1].format      = .float2
        vd.attributes[1].offset      = MemoryLayout<SIMD2<Float>>.stride
        vd.attributes[1].bufferIndex = 1
        vd.layouts[1].stride         = MemoryLayout<QuadVertex>.stride

        let pd = MTLRenderPipelineDescriptor()
        pd.vertexFunction   = vs
        pd.fragmentFunction = fs
        pd.vertexDescriptor = vd

        // 必须从 LayerRenderer 配置取实际格式（避免 dual-pass mismatch）
        let cfg = layerRenderer.configuration
        pd.colorAttachments[0].pixelFormat = cfg.colorFormat
        pd.depthAttachmentPixelFormat      = cfg.depthFormat

        // ★ 关键 1: 最大 vertex amplification 数 = viewCount (=2)
        let viewCount = layerRenderer.properties.viewCount
        pd.maxVertexAmplificationCount = max(viewCount, 1)

        // ★ 关键 2: 必须设置 inputPrimitiveTopology
        // 当 vertex shader 写 [[render_target_array_index]] 时,
        // Metal 需要知道图元类型才能正确路由到 array slice
        pd.inputPrimitiveTopology = .triangle

        print("[Pipeline] viewCount=\(viewCount) maxAmp=\(pd.maxVertexAmplificationCount) " +
              "color=\(cfg.colorFormat.rawValue) depth=\(cfg.depthFormat.rawValue) " +
              "layout=\(cfg.layout == .layered ? "layered" : (cfg.layout == .shared ? "shared" : "dedicated")) " +
              "foveation=\(cfg.isFoveationEnabled)")

        do {
            pipelineState = try device.makeRenderPipelineState(descriptor: pd)
            print("[Pipeline] Created OK")
        } catch {
            print("[Pipeline] FAILED: \(error)")
        }
    }

    // -------------------------------------------------------------------------
    // 深度状态 —— 与 Apple 官方 Compositor Services 模板一致
    //
    // visionOS 采用反转 Z(near=1, far=0),深度比较函数用 .greater。
    // ★ 必须开启 isDepthWriteEnabled,否则 compositor 读到的是 clearDepth(0.0),
    //   以为所有内容都在"无穷远",motion-to-photon reprojection 就会出错
    //   → 头部稍微动一下画面就被 warp 走 → 表现为"一闪而过然后黑屏"。
    // -------------------------------------------------------------------------
    private func buildDepthStencilState() {
        let dd = MTLDepthStencilDescriptor()
        dd.depthCompareFunction = .greater
        dd.isDepthWriteEnabled  = true
        depthState = device.makeDepthStencilState(descriptor: dd)
    }

    // -------------------------------------------------------------------------
    // 顶点 / 索引 buffer
    // -------------------------------------------------------------------------
    private func buildQuadBuffers() {
        vertexBuffer = device.makeBuffer(
            bytes: quadVertices,
            length: MemoryLayout<QuadVertex>.stride * quadVertices.count,
            options: .storageModeShared
        )
        indexBuffer = device.makeBuffer(
            bytes: quadIndices,
            length: MemoryLayout<UInt16>.stride * quadIndices.count,
            options: .storageModeShared
        )
    }

    // -------------------------------------------------------------------------
    // 启动渲染循环 + ARKit 头追踪
    // -------------------------------------------------------------------------
    nonisolated func startRenderLoop() {
        print("[Render] startRenderLoop")

        Task { [self] in
            do {
                print("[Render] Starting ARKit session")
                try await arSession.run([worldTracking])
                arReady = true
                print("[Render] ARKit ready")
            } catch {
                print("[Render] ARKit failed: \(error)")
            }
        }

        let t = Thread { [self] in renderLoop() }
        t.name             = "StereoVideo.RenderThread"
        t.qualityOfService = .userInteractive
        t.start()
        print("[Render] Render thread started")
    }

    nonisolated private func renderLoop() {
        while true {
            switch layerRenderer.state {
            case .paused:
                layerRenderer.waitUntilRunning()
            case .running:
                autoreleasepool { renderFrame() }
            case .invalidated:
                print("[Render] LayerRenderer invalidated, exit loop")
                return
            @unknown default:
                break
            }
        }
    }

    // -------------------------------------------------------------------------
    // 单帧渲染入口
    // -------------------------------------------------------------------------
    nonisolated private func renderFrame() {
        // 1. 取下一帧
        guard let frame = layerRenderer.queryNextFrame() else { return }

        // 2. update 阶段（与 submission 阶段分离,Apple 模板要求）
        frame.startUpdate()
        // 我们没有 CPU 侧 game state,直接进入 endUpdate
        frame.endUpdate()

        // 3. 等到 optimalInputTime（让 compositor 给最准确的预测时间）
        guard let timing = frame.predictTiming() else { return }
        LayerRenderer.Clock().wait(until: timing.optimalInputTime)

        // 4. 查询 drawables（visionOS 26: 数组,builtIn / capture）
        let drawables = frame.queryDrawables()
        guard !drawables.isEmpty else { return }

        // 5. 创建 command buffer
        guard let cmdBuf = commandQueue.makeCommandBuffer() else { return }

        // 6. submission 阶段开始
        frame.startSubmission()

        // 7. 上传最新视频帧到 Metal 纹理
        updateTextures()

        // 8. 取头部位姿（要用 trackableAnchorTime,跟 frame 预测对齐）
        let trackingTime = timing.trackableAnchorTime.timeInterval
        let anchor = arReady
            ? worldTracking.queryDeviceAnchor(atTimestamp: trackingTime)
            : nil

        renderCount += 1

        // 9. 遍历 drawables —— 通常只有一个 builtIn,可能多一个 capture
        for (dIdx, drawable) in drawables.enumerated() {
            drawable.deviceAnchor = anchor

            if renderCount <= 5 || renderCount % 300 == 0 {
                let hasVideo = (yTexture != nil && cbcrTexture != nil)
                print("[Render] #\(renderCount) drawable\(dIdx)/\(drawables.count) " +
                      "views=\(drawable.views.count) anchor=\(anchor != nil) " +
                      "video=\(hasVideo) rrm=\(drawable.rasterizationRateMaps.count) " +
                      "colorArr=\(drawable.colorTextures[0].arrayLength)")
            }

            renderDrawable(drawable: drawable, cmdBuf: cmdBuf)
            // encodePresent 必须在 makeRenderCommandEncoder.endEncoding() 之后,在 commit 之前
            drawable.encodePresent(commandBuffer: cmdBuf)
        }

        // 10. 提交并通知 compositor
        cmdBuf.commit()
        frame.endSubmission()
    }

    // -------------------------------------------------------------------------
    // 单 drawable 单 pass 渲染 —— 这里做 vertex amplification
    // -------------------------------------------------------------------------
    nonisolated private func renderDrawable(
        drawable: LayerRenderer.Drawable,
        cmdBuf: MTLCommandBuffer
    ) {
        guard let pipeline = pipelineState,
              let depth    = depthState,
              let vBuf     = vertexBuffer,
              let iBuf     = indexBuffer
        else { return }

        // -- Render pass descriptor: 单 attachment + array length = views.count --
        let rpd = MTLRenderPassDescriptor()

        rpd.colorAttachments[0].texture     = drawable.colorTextures[0]
        rpd.colorAttachments[0].loadAction  = .clear
        rpd.colorAttachments[0].storeAction = .store
        // ★ DEBUG: 红色背景测试 — 如果连这个都看不到说明渲染根本没到屏幕上
        rpd.colorAttachments[0].clearColor  = MTLClearColor(red: 1, green: 0, blue: 0, alpha: 1)

        // visionOS Compositor Services 采用反转 Z 约定(近=1, 远=0),
        // Apple 官方模板 clearDepth = 0.0,表示"全部内容都在无穷远处"。
        // 用 clearDepth = 1.0 会让 Compositor 把画面当作"贴在眼前",
        // 头稍微一动就触发极大的 reprojection 视差,画面被 warp 出屏幕 → 黑屏。
        // storeAction 也改为 .store,让 Compositor 能稳定读取深度做 timewarp。
        rpd.depthAttachment.texture     = drawable.depthTextures[0]
        rpd.depthAttachment.loadAction  = .clear
        rpd.depthAttachment.storeAction = .store
        rpd.depthAttachment.clearDepth  = 0.0

        // ★ 关键 2: foveation 必须挂 rasterizationRateMap
        // 不挂的话,viewport 是逻辑坐标,纹理是物理坐标,GPU 不知道怎么映射 → 写出去全黑
        // Apple 模板用 .first 是因为 layered 布局下两眼共用一张 array texture
        rpd.rasterizationRateMap = drawable.rasterizationRateMaps.first

        // ★ 关键 3: layered 布局必须设 renderTargetArrayLength
        // 这个值 = views.count = 2 —— 告诉 GPU 这次 pass 要写到 array texture 的 2 个 slice
        if layerRenderer.configuration.layout == .layered {
            rpd.renderTargetArrayLength = drawable.views.count
        }

        guard let enc = cmdBuf.makeRenderCommandEncoder(descriptor: rpd) else { return }
        enc.label = "StereoVideo.RenderPass"

        // ★ 顺序与 Apple 官方 Compositor Services 模板一致:
        //    1. cullMode / frontFacing
        //    2. pipelineState  ← 必须在 setVertexAmplificationCount 之前!
        //    3. depthStencilState
        //    4. setViewports
        //    5. setVertexAmplificationCount (需要当前 pipeline 的 maxVertexAmplificationCount)
        enc.setCullMode(.none)        // quad 是双面的
        enc.setFrontFacing(.counterClockwise)
        enc.setRenderPipelineState(pipeline)
        enc.setDepthStencilState(depth)

        // -- Viewports: 每只眼睛一个 (左/右),由 textureMap 提供 --
        let viewports = drawable.views.map { $0.textureMap.viewport }
        enc.setViewports(viewports)

        // -- Vertex amplification: 一次 draw 调用绘制 N 份 --
        if drawable.views.count > 1 {
            // 两个 viewMapping:
            //   index 0 (左眼): viewport[0], slice 0
            //   index 1 (右眼): viewport[1], slice 1
            var viewMappings = (0..<drawable.views.count).map { i in
                MTLVertexAmplificationViewMapping(
                    viewportArrayIndexOffset:    UInt32(i),
                    renderTargetArrayIndexOffset: UInt32(i)
                )
            }
            enc.setVertexAmplificationCount(drawable.views.count, viewMappings: &viewMappings)
        }

        // -- EyePair Uniforms (index 0): SBS UV 偏移 + YCbCr range --
        var uniforms = makeEyePairUniforms()
        enc.setVertexBytes(&uniforms,   length: MemoryLayout<EyePairUniforms>.size, index: 0)
        enc.setFragmentBytes(&uniforms, length: MemoryLayout<EyePairUniforms>.size, index: 0)

        // -- 顶点 buffer (slot 1, 与 vertex descriptor bufferIndex = 1 对应) --
        enc.setVertexBuffer(vBuf, offset: 0, index: 1)

        // -- ViewProjection Uniforms (index 2): 真正的 world → clip 变换 --
        // 这是修黑屏的核心 —— 必须和 deviceAnchor 的 reprojection 数学保持一致。
        var vpu = makeViewProjectionUniforms(drawable: drawable)
        enc.setVertexBytes(&vpu, length: MemoryLayout<ViewProjectionUniforms>.stride, index: 2)

        // -- 视频纹理 --
        if let yTex = yTexture, let uvTex = cbcrTexture {
            if renderCount <= 3 {
                let v0 = drawable.views[0].textureMap.viewport
                let v1 = drawable.views.count > 1 ? drawable.views[1].textureMap.viewport
                                                   : drawable.views[0].textureMap.viewport
                let colorTex = drawable.colorTextures[0]
                print("[Render] DRAW #\(renderCount):")
                print("  colorTex \(colorTex.width)x\(colorTex.height) type=\(colorTex.textureType.rawValue) arr=\(colorTex.arrayLength)")
                print("  vp0 x=\(v0.originX) y=\(v0.originY) w=\(v0.width) h=\(v0.height)")
                print("  vp1 x=\(v1.originX) y=\(v1.originY) w=\(v1.width) h=\(v1.height)")
                print("  rrm=\(drawable.rasterizationRateMaps.count > 0 ? "SET" : "NONE")")
                print("  Y    \(yTex.width)x\(yTex.height) fmt=\(yTex.pixelFormat.rawValue)")
                print("  CbCr \(uvTex.width)x\(uvTex.height) fmt=\(uvTex.pixelFormat.rawValue)")
                print("  L oX=\(uniforms.eyes.0.uvOffsetX) sX=\(uniforms.eyes.0.uvScaleX) oY=\(uniforms.eyes.0.uvOffsetY) sY=\(uniforms.eyes.0.uvScaleY)")
                print("  R oX=\(uniforms.eyes.1.uvOffsetX) sX=\(uniforms.eyes.1.uvScaleX) oY=\(uniforms.eyes.1.uvOffsetY) sY=\(uniforms.eyes.1.uvScaleY)")
                print("  full=\(uniforms.isFullRange)")
            }

            enc.setFragmentTexture(yTex,  index: 0)
            enc.setFragmentTexture(uvTex, index: 1)

            enc.drawIndexedPrimitives(
                type:              .triangle,
                indexCount:        quadIndices.count,
                indexType:         .uint16,
                indexBuffer:       iBuf,
                indexBufferOffset: 0
            )
        } else if renderCount <= 3 {
            print("[Render] #\(renderCount) NO video texture yet")
        }

        enc.endEncoding()
    }

    // -------------------------------------------------------------------------
    // EyePairUniforms 计算 —— 把 SBS UV 区段算好,左右眼一次传完
    // -------------------------------------------------------------------------
    nonisolated private func makeEyePairUniforms() -> EyePairUniforms {
        let cal = frameHandler.calibration

        // 每只眼睛占 SBS 纹理宽度的一半,再做 cal.scale 比例缩放居中
        let halfScale = 0.5 * cal.scale
        let hMargin   = (0.5 - halfScale) * 0.5

        // 左眼 UV 段: [hMargin + leftHOffset, hMargin + leftHOffset + halfScale]
        let left = EyeParams(
            uvOffsetX: hMargin + cal.leftHOffset,
            uvScaleX:  halfScale,
            uvOffsetY: (1.0 - cal.vscale) * 0.5 + cal.vOffset,
            uvScaleY:  cal.vscale
        )

        // 右眼 UV 段: [0.5 + hMargin + rightHOffset, ...]
        let right = EyeParams(
            uvOffsetX: 0.5 + hMargin + cal.rightHOffset,
            uvScaleX:  halfScale,
            uvOffsetY: (1.0 - cal.vscale) * 0.5 + cal.vOffset,
            uvScaleY:  cal.vscale
        )

        return EyePairUniforms(
            eyes:        (left, right),
            isFullRange: currentIsFullRange ? 1 : 0
        )
    }

    // -------------------------------------------------------------------------
    // makeViewProjectionUniforms —— 每帧根据当前头位姿计算左右眼 view-projection
    //                               和 quad 的 model matrix
    //
    // 设计:
    //   quad 固定在头部正前方 videoDistance 米处(head-locked 视觉),
    //   尺寸按 16:9 左右眼单眼分辨率估算 —— 给用户一个合适的沉浸感。
    //   注意 modelMatrix 已经包含 headFromWorld(将 quad 从 head-local 坐标
    //   放到 world 坐标),这样 compositor 的 reprojection 看到的是真实的
    //   world 位置,warp 数学才能自洽 → 不黑屏。
    // -------------------------------------------------------------------------
    nonisolated private func makeViewProjectionUniforms(
        drawable: LayerRenderer.Drawable
    ) -> ViewProjectionUniforms {

        // deviceAnchor 可能为 nil(ARKit 还没 ready),用单位矩阵兜底。
        // 即使此时渲染了,compositor 也会丢弃这帧(已经有警告日志),
        // 下一帧就正常了。
        let headFromWorld = drawable.deviceAnchor?.originFromAnchorTransform
            ?? matrix_identity_float4x4

        // Per-eye view matrix: world → eye
        //   view.transform 是 eye → device 的变换(左右眼偏移)
        //   headFromWorld  是 device → world
        //   相乘得到 eye → world,取逆就是 world → eye
        let viewLeft  = (headFromWorld * drawable.views[0].transform).inverse
        let viewRight: simd_float4x4
        if drawable.views.count > 1 {
            viewRight = (headFromWorld * drawable.views[1].transform).inverse
        } else {
            viewRight = viewLeft
        }

        // Per-eye projection matrix (非对称视锥,含畸变/FOV)
        let projLeft  = drawable.computeProjection(viewIndex: 0)
        let projRight = drawable.views.count > 1
            ? drawable.computeProjection(viewIndex: 1)
            : projLeft

        // Model matrix —— 把本地 quad [-1,1] 放到头前方 2m,缩放到 3m x 1.68m
        //   视频原始 1344x376 是 SBS(左右并排),单眼 672x376 ≈ 16:9 长宽比
        //   halfW / halfH 决定 quad 在 world 里的物理尺寸
        let videoDistance: Float = 2.0          // 距头 2m
        let halfW:         Float = 1.5          // 3m 宽(约 73° 水平 FOV)
        let halfH:         Float = 0.84         // 1.68m 高 (≈ 3 / 1.787)

        let model = headFromWorld
            * matrix4x4_translation(0, 0, -videoDistance)   // head 前方
            * matrix4x4_scale(halfW, halfH, 1)              // 本地 quad [-1,1] 缩到物理尺寸

        return ViewProjectionUniforms(
            vpLeft:  projLeft  * viewLeft,
            vpRight: projRight * viewRight,
            model:   model
        )
    }

    // -------------------------------------------------------------------------
    // 从 CVPixelBuffer (NV12) 更新 Metal 纹理 (零拷贝,IOSurface 共享)
    // -------------------------------------------------------------------------
    nonisolated private func updateTextures() {
        guard let cache = textureCache,
              let data  = frameHandler.consumeLatestFrame()
        else { return }

        let pb = data.buffer
        lastBuffer         = pb
        currentIsFullRange = data.isFullRange

        // 周期性 flush 防止旧纹理累积占内存
        CVMetalTextureCacheFlush(cache, 0)

        let w0 = CVPixelBufferGetWidthOfPlane(pb, 0)
        let h0 = CVPixelBufferGetHeightOfPlane(pb, 0)
        let w1 = CVPixelBufferGetWidthOfPlane(pb, 1)
        let h1 = CVPixelBufferGetHeightOfPlane(pb, 1)

        var cvY:    CVMetalTexture?
        var cvCbCr: CVMetalTexture?

        // plane 0 = Y (r8Unorm)
        let statusY = CVMetalTextureCacheCreateTextureFromImage(
            nil, cache, pb, nil, .r8Unorm, w0, h0, 0, &cvY
        )
        // plane 1 = CbCr (rg8Unorm)
        let statusCbCr = CVMetalTextureCacheCreateTextureFromImage(
            nil, cache, pb, nil, .rg8Unorm, w1, h1, 1, &cvCbCr
        )

        textureLogCount += 1
        if textureLogCount <= 3 {
            let hasIO = CVPixelBufferGetIOSurface(pb) != nil
            print("[Texture] #\(textureLogCount) Y:\(statusY == 0 ? "OK" : "FAIL(\(statusY))") " +
                  "CbCr:\(statusCbCr == 0 ? "OK" : "FAIL(\(statusCbCr))") " +
                  "planes=\(w0)x\(h0)+\(w1)x\(h1) ioSurface=\(hasIO) full=\(currentIsFullRange)")
        }

        if let cvY    { yTexture    = CVMetalTextureGetTexture(cvY) }
        if let cvCbCr { cbcrTexture = CVMetalTextureGetTexture(cvCbCr) }
    }
}

// =============================================================================
// LayerRenderer.Clock.Instant -> TimeInterval —— Apple 模板里的辅助
// 供 trackableAnchorTime 转 ARKit 期望的 TimeInterval (秒)
// =============================================================================

extension LayerRenderer.Clock.Instant {
    // nonisolated 让渲染线程可以从 Instant 算 TimeInterval 而不触发 MainActor 检查
    fileprivate nonisolated var timeInterval: TimeInterval {
        let comps = LayerRenderer.Clock.Instant.epoch.duration(to: self).components
        let nanos = TimeInterval(comps.attoseconds / 1_000_000_000)
        return TimeInterval(comps.seconds) + (nanos / TimeInterval(NSEC_PER_SEC))
    }
}
