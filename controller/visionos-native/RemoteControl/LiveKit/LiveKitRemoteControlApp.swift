//
//  LiveKitRemoteControlApp.swift
//  RemoteControl
//
//  Created on 2026-04-06.
//  App 入口文件的 LiveKit 版本。
//  将原来 ContentView 替换为 LiveKitContentView。
//  ImmersiveSpace + StereoVideoRenderer 部分完全不变。
//
//  使用方法：
//  在 RemoteControlApp.swift 中把 ContentView() 改为 LiveKitContentView()
//  或者直接把这个文件的内容覆盖到 RemoteControlApp.swift
//

import SwiftUI
import CompositorServices

// 注意：实际使用时不要同时存在两个 。
// 请参考本文件内容修改 RemoteControlApp.swift，
// 或删除 RemoteControlApp.swift 后取消下面的注释。

// 
struct LiveKitRemoteControlApp: App {

    @State private var appState = AppState()

    var body: some Scene {
        WindowGroup {
            // ← 这里是唯一的改动：用 LiveKitContentView 替代 ContentView
            LiveKitContentView()
                .environment(appState)
        }

        // ImmersiveSpace 完全不变 —
        // StereoVideoRenderer 从 VideoFrameHandler 获取帧，
        // 不关心帧是从 WebRTC 还是 LiveKit 来的
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
