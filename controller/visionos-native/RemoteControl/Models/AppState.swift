import Foundation
import Observation

// MARK: - Calibration (Sendable for cross-isolation access)
struct Calibration: Sendable {
    var scale: Float     = 0.85
    var vscale: Float    = 1.00
    var leftHOffset: Float  = 0.000
    var rightHOffset: Float = 0.000
    var vOffset: Float   = 0.000
}

// MARK: - AppState
@Observable
class AppState {
    // Connection
    var connectionStatus: ConnectionStatus = .disconnected
    var errorMessage: String?

    // Immersive space
    var isImmersiveSpaceOpen = false

    // Video stats
    var isReceivingVideo = false
    var fps: Double = 0

    // Calibration (mirrored to VideoFrameHandler on change)
    var calibration = Calibration() {
        didSet { videoFrameHandler.calibration = calibration }
    }

    // Shared objects — accessed across isolation boundaries with @unchecked Sendable
    nonisolated(unsafe) let videoFrameHandler = VideoFrameHandler()

    enum ConnectionStatus: String {
        case disconnected = "Disconnected"
        case connecting   = "Connecting…"
        case signaling    = "Signaling…"
        case streaming    = "Streaming"
    }
}
