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
                // ★ 把 appState.headPoseTracker 传进去,StereoVideoRenderer
                //   每帧渲染时把 deviceAnchor 推到这个共享 tracker,
                //   LiveKitManager 的 pose 定时器从同一个 tracker 读 pose
                //   发到 Data Channel。
                let renderer = StereoVideoRenderer(
                    layerRenderer: layerRenderer,
                    frameHandler: appState.videoFrameHandler,
                    headPoseTracker: appState.headPoseTracker
                )
                renderer.startRenderLoop()
            }
        }
        .immersionStyle(selection: .constant(.full), in: .full)
    }
}
