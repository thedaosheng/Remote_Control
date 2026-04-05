import Foundation

// Delegate is always called on the MainActor.
@MainActor
protocol SignalingDelegate: AnyObject {
    func signalingDidConnect()
    func signalingDidDisconnect()
    func signalingDidReceiveOffer(sdp: String)
    func signalingDidReceiveCandidate(sdpMid: String?, sdpMLineIndex: Int32, sdp: String)
}

// MARK: - SignalingClient
//
// Uses plain ws:// (port 8765) to avoid visionOS self-signed cert issues.
// The server listens on both ws://8765 (Linux) and wss://8766 (old VP).
// Both endpoints route to the same signaling handler.

final class SignalingClient: NSObject, @unchecked Sendable {

    // Plain ws:// — same port as Linux sender. No TLS = no cert issues.
    private let url = URL(string: "ws://39.102.113.104:8765")!
    private let reconnectInterval: TimeInterval = 3

    private var session: URLSession!
    private var wsTask: URLSessionWebSocketTask?
    private let lock = NSLock()

    weak var delegate: SignalingDelegate?

    // Pose sender callback: called on a 30 Hz timer (background queue)
    var onNeedPose: (() -> (position: [Double], quaternion: [Double]))?

    private var poseTimer: DispatchSourceTimer?
    private let poseQueue = DispatchQueue(label: "signaling.pose", qos: .userInteractive)
    private var isConnected = false
    private var shouldReconnect = true
    private var connectAttempt = 0

    override init() {
        super.init()
        session = URLSession(configuration: .default, delegate: self, delegateQueue: nil)
    }

    // MARK: - Public API

    func connect() {
        shouldReconnect = true
        openConnection()
    }

    func disconnect() {
        shouldReconnect = false
        stopPoseTimer()
        lock.withLock {
            wsTask?.cancel(with: .goingAway, reason: nil)
            wsTask = nil
        }
    }

    func sendAnswer(sdp: String) {
        sendJSON(["type": "answer", "sdp": sdp])
    }

    // 告诉 sender 发送新的 offer
    func sendRequestOffer() {
        sendJSON(["type": "request_offer"])
    }

    func sendCandidate(sdpMid: String, sdpMLineIndex: Int32, sdp: String) {
        sendJSON([
            "type": "ice",
            "candidate": sdp,
            "sdpMid": sdpMid,
            "sdpMLineIndex": sdpMLineIndex
        ])
    }

    // MARK: - Connection

    private func openConnection() {
        connectAttempt += 1
        let attempt = connectAttempt
        print("┌──────────────────────────────────────────────")
        print("│ [WS] Connect #\(attempt) → \(url)")
        print("└──────────────────────────────────────────────")

        // Cancel old task
        lock.withLock {
            wsTask?.cancel(with: .goingAway, reason: nil)
            wsTask = nil
        }

        let task = session.webSocketTask(with: url)
        lock.withLock { wsTask = task }
        task.resume()
        // receive() starts in didOpenWithProtocol delegate
    }

    // MARK: - Receive

    private var receiveCount = 0

    private func startReceiveLoop(task: URLSessionWebSocketTask) {
        receiveCount = 0
        receive(from: task)
    }

    private func receive(from task: URLSessionWebSocketTask) {
        task.receive { [weak self] result in
            guard let self else { return }
            switch result {
            case .failure(let error):
                print("[WS] Receive error: \(error)")
                self.handleDisconnect()
            case .success(let msg):
                self.receiveCount += 1
                switch msg {
                case .string(let text):
                    if self.receiveCount <= 10 {
                        print("[WS] RECV #\(self.receiveCount): \(text.prefix(150))")
                    }
                    self.handleMessage(text)
                case .data(let data):
                    if let text = String(data: data, encoding: .utf8) {
                        if self.receiveCount <= 10 {
                            print("[WS] RECV #\(self.receiveCount): \(text.prefix(150))")
                        }
                        self.handleMessage(text)
                    }
                @unknown default:
                    break
                }
                self.receive(from: task)
            }
        }
    }

    // MARK: - Message handling

    private func handleMessage(_ text: String) {
        guard let data = text.data(using: .utf8),
              let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
              let type = json["type"] as? String else { return }

        switch type {
        case "ping":
            let senderTime = json["sender_time"] as? Double ?? 0
            let seq        = json["seq"] as? Int ?? 0
            sendJSON([
                "type": "pong",
                "sender_time": senderTime,
                "receiver_time": Date().timeIntervalSince1970,
                "seq": seq
            ])

        case "offer":
            if let sdp = json["sdp"] as? String {
                print("[WS] ★ RECEIVED OFFER ★ (\(sdp.count) chars)")
                Task { @MainActor [weak self] in self?.delegate?.signalingDidReceiveOffer(sdp: sdp) }
            }

        case "candidate", "ice":
            let sdp    = json["candidate"] as? String ?? ""
            let mid    = json["sdpMid"] as? String   // nil if sender doesn't provide it
            let mIndex = json["sdpMLineIndex"] as? Int32 ?? 0
            print("[WS] Received ICE candidate: mid=\(mid ?? "nil") mLineIndex=\(mIndex)")
            Task { @MainActor [weak self] in
                self?.delegate?.signalingDidReceiveCandidate(sdpMid: mid, sdpMLineIndex: mIndex, sdp: sdp)
            }

        default:
            break
        }
    }

    // MARK: - Send

    private var sendCount = 0

    private func sendJSON(_ dict: [String: Any]) {
        guard let data = try? JSONSerialization.data(withJSONObject: dict),
              let text = String(data: data, encoding: .utf8) else { return }

        let msgType = dict["type"] as? String ?? "?"
        sendCount += 1
        let seq = sendCount

        lock.withLock { wsTask }?.send(.string(text)) { error in
            if let error {
                print("[WS] SEND #\(seq) (\(msgType)) FAILED: \(error)")
            } else if msgType != "pose" && msgType != "pong" {
                print("[WS] SEND #\(seq) (\(msgType)) OK [\(text.count) bytes]")
            }
        }
    }

    // MARK: - Disconnect / Reconnect

    private func handleDisconnect() {
        let wasConnected = lock.withLock {
            let was = isConnected
            isConnected = false
            wsTask?.cancel(with: .goingAway, reason: nil)
            wsTask = nil
            return was
        }
        stopPoseTimer()

        guard shouldReconnect else { return }

        if wasConnected {
            Task { @MainActor [weak self] in self?.delegate?.signalingDidDisconnect() }
        }

        print("[WS] Will reconnect in \(reconnectInterval)s…")
        DispatchQueue.global().asyncAfter(deadline: .now() + reconnectInterval) { [weak self] in
            self?.openConnection()
        }
    }

    // MARK: - Pose timer

    private func startPoseTimer() {
        stopPoseTimer()
        let timer = DispatchSource.makeTimerSource(queue: poseQueue)
        timer.schedule(deadline: .now(), repeating: 1.0 / 30.0)
        timer.setEventHandler { [weak self] in self?.sendPose() }
        timer.resume()
        poseTimer = timer
    }

    private func stopPoseTimer() {
        poseTimer?.cancel()
        poseTimer = nil
    }

    private func sendPose() {
        guard let onNeedPose else { return }
        let pose = onNeedPose()
        sendJSON([
            "type": "pose",
            "t": Date().timeIntervalSince1970,
            "p": pose.position,
            "q": pose.quaternion
        ])
    }
}

// MARK: - URLSessionWebSocketDelegate

extension SignalingClient: URLSessionWebSocketDelegate {
    func urlSession(
        _ session: URLSession,
        webSocketTask: URLSessionWebSocketTask,
        didOpenWithProtocol protocol: String?
    ) {
        print("[WS] ✓ WebSocket OPEN (plain ws://, no TLS)")
        lock.withLock { isConnected = true }
        sendCount = 0

        sendJSON([
            "type": "register",
            "role": "receiver",
            "ip": "visionpro",
            "name": "VP-native"
        ])
        print("[WS] Sent register")

        startPoseTimer()
        Task { @MainActor [weak self] in self?.delegate?.signalingDidConnect() }

        // Start receive loop from the delegate callback
        if let task = lock.withLock({ wsTask }) {
            startReceiveLoop(task: task)
        }
    }

    func urlSession(
        _ session: URLSession,
        webSocketTask: URLSessionWebSocketTask,
        didCloseWith closeCode: URLSessionWebSocketTask.CloseCode,
        reason: Data?
    ) {
        print("[WS] Closed: code=\(closeCode.rawValue)")
        handleDisconnect()
    }
}
