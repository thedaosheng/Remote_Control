import SwiftUI
import CompositorServices

@main
struct RemoteControlApp: App {

    @State private var appState = AppState()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environment(appState)
        }

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
