import Foundation
import LiveKitWebRTC

// MARK: - WebRTCClient
// Manages LKRTCPeerConnection lifecycle: SDP negotiation and ICE exchange.
// LiveKit's WebRTC XCFramework prefixes all Obj-C types with "LK" to avoid
// symbol conflicts with other WebRTC builds.

final class WebRTCClient: NSObject, @unchecked Sendable {

    private static let factory: LKRTCPeerConnectionFactory = {
        LKRTCInitializeSSL()
        LKRTCSetMinDebugLogLevel(.error)
        let enc = LKRTCDefaultVideoEncoderFactory()
        let dec = LKRTCDefaultVideoDecoderFactory()
        return LKRTCPeerConnectionFactory(encoderFactory: enc, decoderFactory: dec)
    }()

    private var peerConnection: LKRTCPeerConnection?
    private let videoFrameHandler: VideoFrameHandler
    private let lock = NSLock()

    // Buffer ICE candidates that arrive before setRemoteDescription completes
    private var pendingCandidates: [LKRTCIceCandidate] = []
    private var remoteDescriptionSet = false

    var onCandidate:    ((String, Int32, String) -> Void)?  // mid, mLineIndex, sdp
    var onConnected:    (() -> Void)?
    var onDisconnected: (() -> Void)?

    init(videoFrameHandler: VideoFrameHandler) {
        self.videoFrameHandler = videoFrameHandler
        super.init()
    }

    // MARK: - Public API

    func handleOffer(sdp: String, completion: @escaping (Result<String, Error>) -> Void) {
        let config = makeConfig()
        let constraints = LKRTCMediaConstraints(
            mandatoryConstraints: ["OfferToReceiveVideo": "true",
                                   "OfferToReceiveAudio": "false"],
            optionalConstraints: ["DtlsSrtpKeyAgreement": "true"]
        )

        guard let pc = WebRTCClient.factory.peerConnection(
            with: config,
            constraints: constraints,
            delegate: self
        ) else {
            completion(.failure(WebRTCError.failedToCreate))
            return
        }

        lock.withLock {
            peerConnection = pc
            remoteDescriptionSet = false
            // Don't clear pendingCandidates — they may have arrived before the offer
        }

        print("[WebRTC] ===== OFFER SDP =====")
        print(sdp)
        print("[WebRTC] ===== END OFFER =====")
        let remoteDesc = LKRTCSessionDescription(type: .offer, sdp: sdp)
        pc.setRemoteDescription(remoteDesc) { [weak self] error in
            guard let self else { return }
            if let error {
                print("[WebRTC] setRemoteDescription failed: \(error)")
                completion(.failure(error))
                return
            }
            print("[WebRTC] setRemoteDescription OK, creating answer…")

            // ★ Drain buffered candidates now that remote description is set
            self.lock.withLock { self.remoteDescriptionSet = true }
            self.drainPendingCandidates()

            pc.answer(for: constraints) { [weak self] answer, error in
                guard let self else { return }
                if let error {
                    print("[WebRTC] createAnswer failed: \(error)")
                    completion(.failure(error))
                    return
                }
                guard let answer else {
                    completion(.failure(WebRTCError.noAnswer))
                    return
                }
                pc.setLocalDescription(answer) { error in
                    if let error {
                        print("[WebRTC] setLocalDescription failed: \(error)")
                        completion(.failure(error))
                    } else {
                        print("[WebRTC] ===== ANSWER SDP =====")
                        print(answer.sdp)
                        print("[WebRTC] ===== END ANSWER =====")
                        completion(.success(answer.sdp))
                    }
                }
            }
        }
    }

    func addCandidate(sdpMid: String?, sdpMLineIndex: Int32, sdp: String) {
        let candidate = LKRTCIceCandidate(sdp: sdp, sdpMLineIndex: sdpMLineIndex, sdpMid: sdpMid)

        let (pc, isReady) = lock.withLock { (peerConnection, remoteDescriptionSet) }

        if isReady, let pc {
            // Remote description set AND peerConnection exists — add immediately
            pc.add(candidate) { error in
                if let error {
                    print("[WebRTC] addIceCandidate failed: \(error)")
                }
            }
        } else {
            // Buffer: either no peerConnection yet, or remoteDescription not set
            lock.withLock { pendingCandidates.append(candidate) }
            let count = lock.withLock { pendingCandidates.count }
            if count <= 3 || count % 10 == 0 {
                print("[WebRTC] Buffered ICE candidate (pending=\(count), pc=\(pc != nil), ready=\(isReady))")
            }
        }
    }

    private func drainPendingCandidates() {
        let candidates = lock.withLock {
            let c = pendingCandidates
            pendingCandidates.removeAll()
            return c
        }
        guard let pc = lock.withLock({ peerConnection }) else { return }
        print("[WebRTC] ★ Draining \(candidates.count) buffered ICE candidates")
        for candidate in candidates {
            pc.add(candidate) { error in
                if let error {
                    print("[WebRTC] addIceCandidate (drain) failed: \(error)")
                }
            }
        }
    }

    func close() {
        lock.withLock {
            peerConnection?.close()
            peerConnection = nil
            remoteDescriptionSet = false
            pendingCandidates.removeAll()
        }
        print("[WebRTC] Closed and reset state")
    }

    // MARK: - ICE server config

    private func makeConfig() -> LKRTCConfiguration {
        let stun = LKRTCIceServer(urlStrings: ["stun:stun.l.google.com:19302"])
        // TURN: 同时尝试 UDP 和 TCP
        let turn = LKRTCIceServer(
            urlStrings: [
                "turn:39.102.113.104:3478",
                "turn:39.102.113.104:3478?transport=tcp"
            ],
            username: "remote",
            credential: "Wawjxyz3!"
        )
        let config = LKRTCConfiguration()
        config.iceServers = [stun, turn]
        config.sdpSemantics = .unifiedPlan
        config.bundlePolicy = .balanced
        config.rtcpMuxPolicy = .require
        config.iceTransportPolicy = .all  // ★ 允许直连和 TURN relay
        return config
    }

    enum WebRTCError: Error {
        case failedToCreate
        case noAnswer
    }
}

// MARK: - LKRTCPeerConnectionDelegate

extension WebRTCClient: LKRTCPeerConnectionDelegate {

    func peerConnection(_ peerConnection: LKRTCPeerConnection,
                        didChange stateChanged: LKRTCSignalingState) {
        let names: [Int: String] = [0:"stable",1:"haveLocalOffer",2:"haveLocalPranswer",3:"haveRemoteOffer",4:"haveRemotePranswer",5:"closed"]
        print("[WebRTC] Signaling state → \(names[stateChanged.rawValue] ?? "\(stateChanged.rawValue)")")
    }

    func peerConnection(_ peerConnection: LKRTCPeerConnection,
                        didRemove stream: LKRTCMediaStream) { }

    func peerConnectionShouldNegotiate(_ peerConnection: LKRTCPeerConnection) { }

    func peerConnection(_ peerConnection: LKRTCPeerConnection,
                        didChange newState: LKRTCIceConnectionState) {
        let names = [0:"new",1:"checking",2:"connected",3:"completed",4:"failed",5:"disconnected",6:"closed",7:"count"]
        print("[WebRTC] ICE state → \(names[newState.rawValue] ?? "\(newState.rawValue)")")
        switch newState {
        case .connected, .completed:
            // DTLS diagnostic
            if let senders = peerConnection.senders as? [LKRTCRtpSender] {
                for s in senders {
                    if let dtls = s.dtlsTransport {
                        print("[WebRTC] DTLS state: \(dtls.state.rawValue) transport=\(type(of: dtls))")
                    } else {
                        print("[WebRTC] DTLS: no transport on sender")
                    }
                }
            }
            for receiver in peerConnection.receivers {
                print("[WebRTC] receiver track: \(receiver.track?.kind ?? "nil") enabled=\(receiver.track?.isEnabled ?? false)")
            }
            onConnected?()
        case .disconnected, .failed, .closed:
            onDisconnected?()
        default:
            break
        }
    }

    func peerConnection(_ peerConnection: LKRTCPeerConnection,
                        didChange newState: LKRTCIceGatheringState) { }

    func peerConnection(_ peerConnection: LKRTCPeerConnection,
                        didGenerate candidate: LKRTCIceCandidate) {
        print("[WebRTC] ★ Generated ICE: \(candidate.sdp)")
        onCandidate?(candidate.sdpMid ?? "0", candidate.sdpMLineIndex, candidate.sdp)
    }

    func peerConnection(_ peerConnection: LKRTCPeerConnection,
                        didRemove candidates: [LKRTCIceCandidate]) { }

    func peerConnection(_ peerConnection: LKRTCPeerConnection,
                        didOpen dataChannel: LKRTCDataChannel) { }

    func peerConnection(_ peerConnection: LKRTCPeerConnection,
                        didAdd rtpReceiver: LKRTCRtpReceiver,
                        streams mediaStreams: [LKRTCMediaStream]) {
        let track = rtpReceiver.track
        print("[WebRTC] ★ didAdd rtpReceiver: kind=\(track?.kind ?? "nil") enabled=\(track?.isEnabled ?? false) streams=\(mediaStreams.count)")
        if let videoTrack = track as? LKRTCVideoTrack {
            print("[WebRTC] ★ Adding VideoFrameHandler to video track")
            videoTrack.isEnabled = true
            videoTrack.add(videoFrameHandler)
        } else {
            print("[WebRTC] ⚠ Track is not LKRTCVideoTrack: \(type(of: track))")
        }
    }

    func peerConnection(_ peerConnection: LKRTCPeerConnection,
                        didAdd stream: LKRTCMediaStream) {
        print("[WebRTC] ★ didAdd stream: id=\(stream.streamId) videoTracks=\(stream.videoTracks.count) audioTracks=\(stream.audioTracks.count)")
        // Also try adding handler via stream (fallback for Plan B style)
        for videoTrack in stream.videoTracks {
            print("[WebRTC] ★ Adding VideoFrameHandler via stream.videoTrack")
            videoTrack.isEnabled = true
            videoTrack.add(videoFrameHandler)
        }
    }
}
