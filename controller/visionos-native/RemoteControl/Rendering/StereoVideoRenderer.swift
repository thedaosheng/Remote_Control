import Metal
import MetalKit
import CompositorServices
import SwiftUI
import CoreVideo
import ARKit
import simd

// ---------------------------------------------------------------------------
// CompositorLayer configuration
// ---------------------------------------------------------------------------

struct VideoLayerConfiguration: CompositorLayerConfiguration {
    func makeConfiguration(
        capabilities: LayerRenderer.Capabilities,
        configuration: inout LayerRenderer.Configuration
    ) {
        configuration.depthFormat  = .depth32Float
        configuration.colorFormat  = .bgra8Unorm_srgb

        let opts: LayerRenderer.Capabilities.SupportedLayoutsOptions = []
        let supported = capabilities.supportedLayouts(options: opts)
        configuration.layout = supported.contains(.dedicated) ? .dedicated : .layered
    }
}

// ---------------------------------------------------------------------------
// Quad geometry — fullscreen NDC [-1,1]²
// ---------------------------------------------------------------------------

private struct QuadVertex {
    var position: SIMD2<Float>
    var uv:       SIMD2<Float>
}

private let quadVertices: [QuadVertex] = [
    QuadVertex(position: SIMD2(-1,  1), uv: SIMD2(0, 0)),
    QuadVertex(position: SIMD2( 1,  1), uv: SIMD2(1, 0)),
    QuadVertex(position: SIMD2(-1, -1), uv: SIMD2(0, 1)),
    QuadVertex(position: SIMD2( 1, -1), uv: SIMD2(1, 1)),
]

private let quadIndices: [UInt16] = [0, 2, 1,  1, 2, 3]

// Must match EyeUniforms in VideoShaders.metal
private struct EyeUniforms {
    var uvOffsetX:   Float
    var uvScaleX:    Float
    var uvOffsetY:   Float
    var uvScaleY:    Float
    var isFullRange: UInt32
    var pad0: Float = 0; var pad1: Float = 0; var pad2: Float = 0
}

// ---------------------------------------------------------------------------
// StereoVideoRenderer
// ---------------------------------------------------------------------------

final class StereoVideoRenderer: @unchecked Sendable {

    private let layerRenderer:    LayerRenderer
    private let frameHandler:     VideoFrameHandler
    private let device:           MTLDevice
    private let commandQueue:     MTLCommandQueue
    private var pipelineState:    MTLRenderPipelineState?
    private var vertexBuffer:     MTLBuffer?
    private var indexBuffer:      MTLBuffer?
    private var textureCache:     CVMetalTextureCache?

    // ARKit — provides device anchor for each frame
    private let arSession = ARKitSession()
    private let worldTracking = WorldTrackingProvider()

    // Cached video textures
    private var yTexture:    MTLTexture?
    private var cbcrTexture: MTLTexture?
    private var lastBuffer:  CVPixelBuffer?

    init(layerRenderer: LayerRenderer, frameHandler: VideoFrameHandler) {
        self.layerRenderer = layerRenderer
        self.frameHandler  = frameHandler

        guard let dev = MTLCreateSystemDefaultDevice() else {
            fatalError("Metal not available")
        }
        self.device       = dev
        self.commandQueue = dev.makeCommandQueue()!

        buildPipeline()
        buildQuadBuffers()
        CVMetalTextureCacheCreate(nil, nil, dev, nil, &textureCache)
    }

    // MARK: - Build resources

    private func buildPipeline() {
        guard let lib = device.makeDefaultLibrary() else {
            print("[Render] no default Metal library")
            return
        }
        let vs = lib.makeFunction(name: "vs_video")
        let fs = lib.makeFunction(name: "fs_video_nv12")

        let vd = MTLVertexDescriptor()
        vd.attributes[0].format      = .float2
        vd.attributes[0].offset      = 0
        vd.attributes[0].bufferIndex = 1
        vd.attributes[1].format      = .float2
        vd.attributes[1].offset      = MemoryLayout<SIMD2<Float>>.stride
        vd.attributes[1].bufferIndex = 1
        vd.layouts[1].stride         = MemoryLayout<QuadVertex>.stride

        let pd                            = MTLRenderPipelineDescriptor()
        pd.vertexFunction                 = vs
        pd.fragmentFunction               = fs
        pd.vertexDescriptor               = vd
        pd.colorAttachments[0].pixelFormat = .bgra8Unorm_srgb
        pd.depthAttachmentPixelFormat      = .depth32Float

        pipelineState = try? device.makeRenderPipelineState(descriptor: pd)
        if pipelineState == nil {
            print("[Render] ✗ Pipeline creation failed!")
        }
    }

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

    // MARK: - Render loop

    nonisolated func startRenderLoop() {
        print("[Render] startRenderLoop called")

        // Start ARKit world tracking for device anchors
        Task {
            do {
                print("[Render] Starting ARKit session...")
                try await arSession.run([worldTracking])
                arReady = true
                print("[Render] ✓ ARKit WorldTrackingProvider started")
            } catch {
                print("[Render] ✗ ARKit failed: \(error)")
            }
        }

        let t = Thread { [self] in renderLoop() }
        t.name = "StereoVideo.RenderThread"
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
                renderFrame()
            case .invalidated:
                return
            @unknown default:
                break
            }
        }
    }

    private var renderCount = 0
    private var arReady = false

    nonisolated private func renderFrame() {
        guard let frame = layerRenderer.queryNextFrame() else { return }
        guard let drawable = frame.queryDrawables().first else { return }
        guard let pipeline = pipelineState else {
            if renderCount == 0 { print("[Render] ✗ pipelineState is nil") }
            return
        }
        guard let vBuf = vertexBuffer, let iBuf = indexBuffer else { return }
        guard let cmdBuf = commandQueue.makeCommandBuffer() else { return }

        // ★ visionOS 26: Must set device anchor or drawable won't be presented.
        // Use CACurrentMediaTime() which shares the same timebase as ARKit.
        let anchor = worldTracking.queryDeviceAnchor(atTimestamp: CACurrentMediaTime())
        drawable.deviceAnchor = anchor

        renderCount += 1

        // Log first few frames and periodically
        if renderCount <= 5 || renderCount % 300 == 0 {
            let hasVideo = (yTexture != nil && cbcrTexture != nil)
            print("[Render] #\(renderCount) views=\(drawable.views.count) anchor=\(anchor != nil) video=\(hasVideo) arReady=\(arReady)")
        }

        // If no anchor yet, still submit frame (clear to black) but it won't display.
        // Once ARKit provides anchors, frames will start displaying.
        frame.startSubmission()

        updateTextures()

        for i in 0..<drawable.views.count {
            renderEye(
                index: i,
                view: drawable.views[i],
                drawable: drawable,
                pipeline: pipeline,
                vertexBuf: vBuf,
                indexBuf: iBuf,
                cmdBuf: cmdBuf
            )
        }

        drawable.encodePresent(commandBuffer: cmdBuf)
        cmdBuf.commit()

        frame.endSubmission()
    }

    nonisolated private func renderEye(
        index: Int,
        view: LayerRenderer.Drawable.View,
        drawable: LayerRenderer.Drawable,
        pipeline: MTLRenderPipelineState,
        vertexBuf: MTLBuffer,
        indexBuf: MTLBuffer,
        cmdBuf: MTLCommandBuffer
    ) {
        let texMap = view.textureMap

        let rpd = MTLRenderPassDescriptor()
        rpd.colorAttachments[0].texture     = drawable.colorTextures[texMap.textureIndex]
        rpd.colorAttachments[0].loadAction  = .clear
        rpd.colorAttachments[0].storeAction = .store
        rpd.colorAttachments[0].clearColor  = MTLClearColor(red: 0, green: 0, blue: 0, alpha: 1)
        rpd.colorAttachments[0].slice       = texMap.sliceIndex

        rpd.depthAttachment.texture         = drawable.depthTextures[texMap.textureIndex]
        rpd.depthAttachment.loadAction      = .clear
        rpd.depthAttachment.storeAction     = .dontCare
        rpd.depthAttachment.clearDepth      = 1.0
        rpd.depthAttachment.slice           = texMap.sliceIndex

        guard let enc = cmdBuf.makeRenderCommandEncoder(descriptor: rpd) else { return }
        enc.setViewport(view.textureMap.viewport)
        enc.setRenderPipelineState(pipeline)

        var uniforms = makeUniforms(eyeIndex: index)
        enc.setVertexBytes(&uniforms,   length: MemoryLayout<EyeUniforms>.size, index: 0)
        enc.setFragmentBytes(&uniforms, length: MemoryLayout<EyeUniforms>.size, index: 0)
        enc.setVertexBuffer(vertexBuf, offset: 0, index: 1)

        if let yTex = yTexture, let uTex = cbcrTexture {
            enc.setFragmentTexture(yTex, index: 0)
            enc.setFragmentTexture(uTex, index: 1)
            enc.drawIndexedPrimitives(
                type:              .triangle,
                indexCount:        quadIndices.count,
                indexType:         .uint16,
                indexBuffer:       indexBuf,
                indexBufferOffset: 0
            )
        }
        enc.endEncoding()
    }

    // MARK: - UV helpers

    nonisolated private func makeUniforms(eyeIndex: Int) -> EyeUniforms {
        let cal = frameHandler.calibration
        let halfScale = 0.5 * cal.scale
        let hMargin   = (0.5 - halfScale) * 0.5

        let offsetX: Float
        if eyeIndex == 0 {
            offsetX = hMargin + cal.leftHOffset
        } else {
            offsetX = 0.5 + hMargin + cal.rightHOffset
        }

        let vMargin = (1.0 - cal.vscale) * 0.5
        let offsetY = vMargin + cal.vOffset

        return EyeUniforms(
            uvOffsetX:   offsetX,
            uvScaleX:    halfScale,
            uvOffsetY:   offsetY,
            uvScaleY:    cal.vscale,
            isFullRange: 0
        )
    }

    // MARK: - Texture upload from CVPixelBuffer (NV12)

    nonisolated private func updateTextures() {
        guard
            let cache = textureCache,
            let data  = frameHandler.consumeLatestFrame()
        else { return }

        let pb = data.buffer
        if pb === lastBuffer { return }
        lastBuffer = pb

        var cvY:    CVMetalTexture?
        var cvCbCr: CVMetalTexture?

        CVMetalTextureCacheCreateTextureFromImage(
            nil, cache, pb, nil, .r8Unorm,
            CVPixelBufferGetWidthOfPlane(pb, 0),
            CVPixelBufferGetHeightOfPlane(pb, 0),
            0, &cvY
        )
        CVMetalTextureCacheCreateTextureFromImage(
            nil, cache, pb, nil, .rg8Unorm,
            CVPixelBufferGetWidthOfPlane(pb, 1),
            CVPixelBufferGetHeightOfPlane(pb, 1),
            1, &cvCbCr
        )

        if let cvY    { yTexture    = CVMetalTextureGetTexture(cvY)    }
        if let cvCbCr { cbcrTexture = CVMetalTextureGetTexture(cvCbCr) }
    }
}
