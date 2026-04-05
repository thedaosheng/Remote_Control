import Foundation

// Orchestrates SignalingClient + WebRTCClient + HeadPoseTracker.
// All callbacks update AppState on the MainActor.
@MainActor
final class ConnectionManager {

    private let appState:    AppState
    private let signaling:   SignalingClient
    private let webRTC:      WebRTCClient
    private let headTracker: HeadPoseTracker

    // Frame-rate tracking
    private var lastFPSCheck = Date()
    private var fpsFrameCount = 0
    private var hasActiveOffer = false

    init(appState: AppState) {
        self.appState    = appState
        self.signaling   = SignalingClient()
        self.webRTC      = WebRTCClient(videoFrameHandler: appState.videoFrameHandler)
        self.headTracker = HeadPoseTracker()
        wireCallbacks()
    }

    func connect() {
        appState.connectionStatus = .connecting
        headTracker.start()
        signaling.connect()
    }

    func disconnect() {
        hasActiveOffer = false
        stopOfferRetry()
        signaling.disconnect()
        webRTC.close()
        headTracker.stop()
        appState.connectionStatus = .disconnected
        appState.isReceivingVideo = false
    }

    // MARK: - Wiring

    private func wireCallbacks() {
        // Signaling → self
        signaling.delegate = self

        // Pose provider for signaling timer
        signaling.onNeedPose = { [weak self] in
            guard let self else { return ([0,0,0], [0,0,0,1]) }
            return self.headTracker.poseArrays() ?? ([0,0,0], [0,0,0,1])
        }

        // WebRTC connected/disconnected
        webRTC.onConnected = { [weak self] in
            Task { @MainActor [weak self] in
                self?.appState.connectionStatus = .streaming
                self?.appState.isReceivingVideo = true
                self?.startFPSTimer()
            }
        }
        webRTC.onDisconnected = { [weak self] in
            Task { @MainActor [weak self] in
                self?.appState.connectionStatus = .connecting
                self?.appState.isReceivingVideo = false
                self?.hasActiveOffer = false
                self?.startOfferRetry()
            }
        }

        // WebRTC → signaling (ICE candidates)
        webRTC.onCandidate = { [weak self] mid, mLineIndex, sdp in
            self?.signaling.sendCandidate(sdpMid: mid, sdpMLineIndex: mLineIndex, sdp: sdp)
        }
    }

    // MARK: - Request offer retry

    private var offerRetryTask: Task<Void, Never>?

    /// 连上信令后开始重试 request_offer，直到收到 offer
    private func startOfferRetry() {
        offerRetryTask?.cancel()
        offerRetryTask = Task { [weak self] in
            // 等 8 秒给 sender 重建管道的时间
            try? await Task.sleep(for: .seconds(8))
            while !Task.isCancelled {
                guard let self else { break }
                // 如果已经在 streaming 就停止重试
                if self.appState.connectionStatus == .streaming { break }
                print("[ConnectionManager] 重发 request_offer（未收到 offer）")
                self.signaling.sendRequestOffer()
                try? await Task.sleep(for: .seconds(8))
            }
        }
    }

    private func stopOfferRetry() {
        offerRetryTask?.cancel()
        offerRetryTask = nil
    }

    // MARK: - FPS counter

    private var fpsTimer: Task<Void, Never>?

    private func startFPSTimer() {
        fpsTimer?.cancel()
        fpsTimer = Task { [weak self] in
            var previousCount = 0
            while !Task.isCancelled {
                try? await Task.sleep(for: .seconds(1))
                guard let self else { break }
                let count = self.appState.videoFrameHandler.frameCount
                let fps   = Double(count - previousCount)
                previousCount = count
                self.appState.fps = fps
            }
        }
    }
}

// MARK: - SignalingDelegate

extension ConnectionManager: SignalingDelegate {

    func signalingDidConnect() {
        appState.connectionStatus = .signaling
        signaling.sendRequestOffer()
        startOfferRetry()
    }

    func signalingDidDisconnect() {
        appState.connectionStatus = .connecting
        appState.isReceivingVideo = false
    }

    func signalingDidReceiveOffer(sdp: String) {
        // 只处理第一个 offer，忽略后续重复的
        if hasActiveOffer {
            print("[ConnectionManager] 忽略重复 offer（已有活跃会话）")
            return
        }
        hasActiveOffer = true
        stopOfferRetry()
        webRTC.handleOffer(sdp: sdp) { [weak self] result in
            switch result {
            case .success(let answerSDP):
                self?.signaling.sendAnswer(sdp: answerSDP)
            case .failure(let error):
                Task { @MainActor [weak self] in
                    self?.appState.errorMessage = "WebRTC error: \(error.localizedDescription)"
                }
            }
        }
    }

    func signalingDidReceiveCandidate(sdpMid: String?, sdpMLineIndex: Int32, sdp: String) {
        webRTC.addCandidate(sdpMid: sdpMid, sdpMLineIndex: sdpMLineIndex, sdp: sdp)
    }
}
