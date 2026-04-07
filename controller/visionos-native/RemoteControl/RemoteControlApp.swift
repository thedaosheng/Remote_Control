import SwiftUI
import CompositorServices

// =============================================================================
// RemoteControlApp —— App 入口
//
// - WindowGroup: LiveKitContentView (LiveKit 连接 + Calibration UI)
// - ImmersiveSpace "stereoVideo":
//     CompositorLayer + StereoVideoRenderer (vertex amplification 单 pass)
//
// 注意:
//   只能有一个 @main —— LiveKitRemoteControlApp.swift 已经把 @main 注释掉了
// =============================================================================

@main
struct RemoteControlApp: App {

    @State private var appState = AppState()

    var body: some Scene {
        // 主窗口: LiveKit 连接 + 校准 UI
        WindowGroup {
            LiveKitContentView()
                .environment(appState)
        }

        // 沉浸空间: CompositorServices + 自定义 Metal 渲染
        // 用 vertex amplification 在单 pass 内同时画左右眼
        ImmersiveSpace(id: "stereoVideo") {
            CompositorLayer(configuration: VideoLayerConfiguration()) { layerRenderer in
                let renderer = StereoVideoRenderer(
                    layerRenderer: layerRenderer,
                    frameHandler: appState.videoFrameHandler
                )
                renderer.startRenderLoop()
            }
        }
        .immersionStyle(selection: .constant(.full), in: .full)
    }
}
