//
//  LiveKitManager.swift
//  RemoteControl
//
//  Created on 2026-04-06.
//  替换原有 ConnectionManager + SignalingClient + WebRTCClient，
//  使用 LiveKit Swift SDK 通过 LiveKit Server 中继完成：
//    - 视频轨道订阅（接收 sender 推送的双目拼接流）
//    - Data Channel 发送 head pose（30Hz）
//

import Foundation
import LiveKit

// MARK: - LiveKit 服务器配置
// 所有连接参数集中定义，方便后续修改
enum LiveKitConfig {
    /// LiveKit 服务器 WebSocket 地址（公网阿里云）
    /// 如果配了 TLS 证书可改为 wss://
    static let serverURL = "ws://39.102.113.104:7880"

    /// 房间名称 — 需要与 sender 端一致
    static let roomName = "teleop-room"

    /// 预生成的 JWT Token（30 天有效，到期后用 Python 重新生成）
    /// Identity: vision-pro-receiver
    /// Grants: room_join=true, can_subscribe=true, can_publish=false, can_publish_data=true
    static let receiverToken = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJuYW1lIjoiVmlzaW9uIFBybyBSZWNlaXZlciIsInZpZGVvIjp7InJvb21Kb2luIjp0cnVlLCJyb29tIjoidGVsZW9wLXJvb20iLCJjYW5QdWJsaXNoIjpmYWxzZSwiY2FuU3Vic2NyaWJlIjp0cnVlLCJjYW5QdWJsaXNoRGF0YSI6dHJ1ZX0sInN1YiI6InZpc2lvbi1wcm8tcmVjZWl2ZXIiLCJpc3MiOiJ0ZWxlb3Bfa2V5IiwibmJmIjoxNzc1NDA2NzQwLCJleHAiOjE3Nzc5OTg3NDB9.NouDG7Xh-cu_F050stjbIYc0QKWoOh8MSfQvlbdJ18Q"

    /// Pose 数据发送频率（Hz）
    static let poseFrequency: Double = 30.0

    /// Pose 数据的 Data Channel topic 名称
    static let poseTopic = "pose"
}

// MARK: - PoseData
// Data Channel 传输的 head pose JSON 结构体
// 与原 SignalingClient 的 pose 消息格式保持一致

private struct PoseData: Codable {
    let type: String        // 固定为 "pose"
    let t: Double           // Unix 时间戳（秒）
    let p: [Double]         // 位置 [x, y, z]，单位：米
    let q: [Double]         // 四元数 [x, y, z, w]
}

// MARK: - LiveKitManager
// 管理与 LiveKit Server 的所有交互：
//   1. 连接/断开 Room
//   2. 监听远端参与者（sender）的视频轨道
//   3. 将 VideoTrack 的帧送入 VideoFrameHandler（复用现有 Metal 渲染管线）
//   4. 通过 Data Channel 以 30Hz 发送 head pose

@MainActor
final class LiveKitManager {

    // MARK: - 依赖
    private let appState: AppState              // 全局状态（UI 绑定）
    // ★ headTracker 从 appState 拿,不再自己 new 一个。
    //   原因:visionOS 一个进程只能有一个 WorldTrackingProvider。
    //   现在 StereoVideoRenderer 是唯一运行 ARKit 的地方,
    //   LiveKitManager 只从共享 tracker 读缓存的 pose。
    private let headTracker: HeadPoseTracker
    private let room: Room                       // LiveKit Room 实例

    // MARK: - 内部状态
    private var poseTimer: Task<Void, Never>?   // 30Hz pose 发送定时器
    private var fpsTimer: Task<Void, Never>?    // 1Hz FPS 统计定时器
    private var isConnected = false              // 防止重复连接

    // 诊断计数器:帮我们定位 pose 回传是否通
    private var poseNilCount: Int = 0     // 连续多少帧 headTracker 没返回 pose
    private var poseSentCount: Int = 0    // 成功 publish 的累计次数

    // MARK: - 初始化
    init(appState: AppState) {
        self.appState = appState
        // ★ 从 AppState 拿共享 tracker(StereoVideoRenderer 每帧往里写)
        //   之前是 `HeadPoseTracker()` 自己 new 一个,会导致两个 ARKitSession 冲突
        self.headTracker = appState.headPoseTracker

        // 创建 Room，配置选项
        // Room 会自动管理 PeerConnection、ICE、DTLS/SRTP 协商
        self.room = Room()

        // 注册自己为 Room 的事件代理
        room.delegates.add(delegate: self)
    }

    // MARK: - 连接
    // 调用后依次执行：
    //   1. 启动 ARKit head tracking
    //   2. 连接到 LiveKit Server
    //   3. 自动订阅 room 中已有的和新加入的 sender 视频轨道

    func connect() {
        guard !isConnected else {
            print("[LiveKit] 已经连接，忽略重复调用")
            return
        }

        appState.connectionStatus = .connecting
        appState.errorMessage = nil

        // ★ 不再启动 HeadPoseTracker 的 ARKit session ——
        //   tracker 只是共享缓存,ARKit 由 StereoVideoRenderer 独占运行。
        //   所以 pose 只有在用户"进入 Immersive Space"之后才会开始有数据。
        //   在此之前 poseArrays() 返回 nil,sendPose() 会静默跳过(正确行为)。
        headTracker.reset()   // 清掉可能的 stale pose 缓存

        // 异步连接到 LiveKit room
        Task {
            do {
                print("┌──────────────────────────────────────────────")
                print("│ [LiveKit] 正在连接: \(LiveKitConfig.serverURL)")
                print("│ [LiveKit] 房间: \(LiveKitConfig.roomName)")
                print("│ [LiveKit] 身份: vision-pro-receiver")
                print("└──────────────────────────────────────────────")

                // Room.connect() 内部处理：
                //   - WebSocket 信令连接
                //   - JWT token 验证
                //   - PeerConnection 创建
                //   - ICE candidate 交换
                //   - DTLS/SRTP 握手
                // 全部由 LiveKit SDK 自动完成，无需手动管理
                try await room.connect(
                    url: LiveKitConfig.serverURL,
                    token: LiveKitConfig.receiverToken,
                    connectOptions: ConnectOptions(
                        autoSubscribe: true  // 自动订阅所有远端轨道
                    ),
                    roomOptions: RoomOptions()  // 使用默认选项
                )

                isConnected = true
                appState.connectionStatus = .signaling
                print("[LiveKit] ✓ Room 连接成功，等待 sender 推流...")

                // 检查是否已有参与者在推流
                // （如果 sender 先于 receiver 加入，轨道已经在 room 中了）
                checkExistingTracks()

            } catch {
                print("[LiveKit] ✗ 连接失败: \(error)")
                appState.connectionStatus = .disconnected
                appState.errorMessage = "LiveKit 连接失败: \(error.localizedDescription)"
                isConnected = false
            }
        }
    }

    // MARK: - 断开连接
    //
    // ★ 改成 async —— 之前是"fire and forget"的 Task { room.disconnect() },
    //   外层立刻继续执行。ContentView 在 disconnect 调用后立马把 liveKitManager
    //   设为 nil,Room 对象被释放,但服务器端 vision-pro-receiver session 还没
    //   清理,下次 Connect 时用同一 identity 会冲突/卡住。
    //   现在把整个流程 await 干净,再返回,确保服务器端 session 彻底清理。
    //
    // 同时 Room 对象不再释放 —— LiveKit Room 支持 disconnect 后再次 connect,
    // 复用同一个 Room 比每次新建更稳。

    func disconnect() async {
        print("[LiveKit] 断开连接 (awaiting)…")

        // 停止所有定时器(取消 Task,不需要 await)
        stopPoseTimer()
        stopFPSTimer()

        // ★ tracker 不再自己 run ARKit,只清缓存
        //   真正的 ARKit 生命周期跟着 ImmersiveSpace 走,
        //   由 StereoVideoRenderer 负责。
        headTracker.reset()

        // ★ 等 Room 真正断开(WebSocket 关闭、PeerConnection 清理)
        await room.disconnect()

        // 重置状态
        isConnected = false
        appState.connectionStatus = .disconnected
        appState.isReceivingVideo = false
        appState.errorMessage = nil

        // 重置诊断计数器,下次连接时重新统计
        poseNilCount  = 0
        poseSentCount = 0

        print("[LiveKit] ✓ 断开完成")
    }

    // MARK: - 检查已有轨道
    // 处理 receiver 后于 sender 加入房间的情况

    private func checkExistingTracks() {
        // 遍历所有远端参与者
        for (_, participant) in room.remoteParticipants {
            print("[LiveKit] 发现已有参与者: \(participant.identity?.stringValue ?? "unknown")")
            // 遍历该参与者的所有轨道发布
            for (_, publication) in participant.trackPublications {
                if let remotePublication = publication as? RemoteTrackPublication,
                   let track = remotePublication.track {
                    // 如果已经有订阅的视频轨道，直接绑定
                    if let videoTrack = track as? VideoTrack {
                        print("[LiveKit] ★ 发现已订阅的视频轨道，直接绑定")
                        attachVideoTrack(videoTrack)
                    }
                }
            }
        }
    }

    // MARK: - 视频轨道绑定
    // 将 LiveKit 的 VideoTrack 绑定到现有的 VideoFrameHandler
    // VideoFrameHandler 实现了 VideoRenderer 协议，
    // LiveKit SDK 的 VideoTrack.add(videoRenderer:) 会将解码后的帧回调给它

    private func attachVideoTrack(_ track: VideoTrack) {
        print("[LiveKit] ★ 绑定视频轨道到 VideoFrameHandler")
        // LiveKit 的 VideoTrack.add(videoRenderer:) 会将解码后的帧
        // 回调给 VideoFrameHandler.render(frame:)，
        // 后续走原有的 CVPixelBuffer → Metal texture → CompositorLayer 渲染管线
        track.add(videoRenderer: appState.videoFrameHandler)

        // 更新 UI 状态
        appState.connectionStatus = .streaming
        appState.isReceivingVideo = true

        // 启动 pose 发送定时器和 FPS 统计
        startPoseTimer()
        startFPSTimer()
    }

    // MARK: - 视频轨道解绑

    private func detachVideoTrack(_ track: VideoTrack) {
        print("[LiveKit] 解绑视频轨道")
        track.remove(videoRenderer: appState.videoFrameHandler)

        // 如果没有其他视频轨道了，更新状态
        appState.isReceivingVideo = false
        appState.connectionStatus = .signaling

        stopPoseTimer()
        stopFPSTimer()
    }

    // MARK: - Pose 发送
    // 通过 LiveKit Data Channel 以 30Hz 发送 head pose
    // Data Channel 比 WebSocket 延迟更低（走 SCTP over DTLS，与媒体同路径）

    private func startPoseTimer() {
        stopPoseTimer()
        poseTimer = Task { [weak self] in
            // 使用 AsyncStream + Task.sleep 实现精确 30Hz 定时
            let interval = 1.0 / LiveKitConfig.poseFrequency
            while !Task.isCancelled {
                guard let self else { break }
                self.sendPose()
                try? await Task.sleep(for: .seconds(interval))
            }
        }
        print("[LiveKit] ✓ Pose 定时器已启动 (\(Int(LiveKitConfig.poseFrequency))Hz)")
    }

    private func stopPoseTimer() {
        poseTimer?.cancel()
        poseTimer = nil
    }

    /// 编码 head pose 为 JSON 并通过 Data Channel 发送
    /// 使用 unreliable 模式（UDP 语义）降低延迟
    ///
    /// ★ 本版本加了诊断日志,用于定位"sender 收不到 pose"的根因:
    ///   1. poseArrays() 返回 nil 的频率 → 是否 headTracker 没拿到数据
    ///      (典型原因:两个 ARKitSession 冲突,或 session 还没 ready)
    ///   2. publish 是否抛异常 → 是否 Data Channel 有问题
    ///   3. 成功次数 → 是否 SDK 静默丢包
    private func sendPose() {
        // 从共享 HeadPoseTracker 获取当前 pose
        // StereoVideoRenderer 每帧会往里写,所以只有用户进入 Immersive Space
        // 后才会有数据。之前 nil 是正常的(还没进沉浸空间 → 不需要发 pose)。
        guard let (position, quaternion) = headTracker.poseArrays() else {
            // 每 90 次(约 3 秒)打印一次诊断
            poseNilCount += 1
            if poseNilCount == 1 || poseNilCount % 90 == 0 {
                print("[Pose] tracker 还没数据 (#\(poseNilCount)) " +
                      "—— 等待用户进入 Immersive Space")
            }
            return
        }

        // 有数据了 → 重置 nil 计数
        if poseNilCount > 0 {
            print("[Pose] ✓ tracker 首次返回数据(之前 nil \(poseNilCount) 次),开始发送")
            poseNilCount = 0
        }

        // 构造 JSON 消息
        let poseData = PoseData(
            type: "pose",
            t: Date().timeIntervalSince1970,
            p: position,
            q: quaternion
        )

        // 编码为 JSON data
        guard let jsonData = try? JSONEncoder().encode(poseData) else {
            print("[Pose] ⚠ JSONEncoder encode failed")
            return
        }

        // 通过 LiveKit Data Channel 发送
        // reliable: false → 使用不可靠传输（类似 UDP），降低延迟
        // topic: "pose" → sender 端可按 topic 过滤
        Task { [weak self] in
            guard let self else { return }
            do {
                try await self.room.localParticipant.publish(
                    data: jsonData,
                    options: DataPublishOptions(
                        topic: LiveKitConfig.poseTopic,
                        reliable: false
                    )
                )
                // ★ 诊断:publish 成功
                await MainActor.run { [weak self] in
                    guard let self else { return }
                    self.poseSentCount += 1
                    // 第 1 次 + 每 300 次(10秒)打一行
                    if self.poseSentCount == 1 || self.poseSentCount % 300 == 0 {
                        print("[Pose] ✓ sent #\(self.poseSentCount) " +
                              "pos=[\(String(format: "%.3f,%.3f,%.3f", position[0], position[1], position[2]))]")
                    }
                }
            } catch {
                // ★ 诊断:publish 抛异常,打出来(之前是静默吞掉的)
                print("[Pose] ✗ publish error: \(error)")
            }
        }
    }

    // MARK: - FPS 统计
    // 每秒读取 VideoFrameHandler 的帧计数，更新 UI 显示

    private func startFPSTimer() {
        stopFPSTimer()
        fpsTimer = Task { [weak self] in
            var previousCount = 0
            while !Task.isCancelled {
                try? await Task.sleep(for: .seconds(1))
                guard let self else { break }
                let count = self.appState.videoFrameHandler.frameCount
                let fps = Double(count - previousCount)
                previousCount = count
                self.appState.fps = fps
            }
        }
    }

    private func stopFPSTimer() {
        fpsTimer?.cancel()
        fpsTimer = nil
    }
}

// MARK: - RoomDelegate
// 监听 LiveKit Room 事件
// 关键事件：
//   - didSubscribeTrack: 成功订阅远端轨道（收到视频）
//   - didUnsubscribeTrack: 取消订阅（sender 断开或停止推流）
//   - participantDidConnect/Disconnect: 参与者进出房间
//   - didUpdateConnectionState: 连接状态变化

extension LiveKitManager: RoomDelegate {

    /// Room 连接状态变化
    nonisolated func room(_ room: Room, didUpdateConnectionState connectionState: ConnectionState, from oldConnectionState: ConnectionState) {
        Task { @MainActor [weak self] in
            guard let self else { return }
            print("[LiveKit] 连接状态: \(oldConnectionState) → \(connectionState)")

            switch connectionState {
            case .disconnected:
                // 如果是意外断开，尝试提示用户
                if self.isConnected {
                    self.appState.connectionStatus = .disconnected
                    self.appState.isReceivingVideo = false
                    self.isConnected = false
                    self.stopPoseTimer()
                    self.stopFPSTimer()
                    self.appState.errorMessage = "LiveKit 连接断开"
                    print("[LiveKit] ✗ 意外断连")
                }
            case .connected:
                print("[LiveKit] ✓ 连接已恢复/建立")
            case .reconnecting:
                print("[LiveKit] ⟳ 正在重连...")
                self.appState.errorMessage = "LiveKit 正在重连..."
            case .connecting:
                // 正在建立初始连接，无需额外处理
                print("[LiveKit] ⟳ 正在连接...")
            case .disconnecting:
                // 正在断开连接，无需额外处理
                print("[LiveKit] ⟳ 正在断开...")
            @unknown default:
                // 未来 SDK 新增的状态，打印日志但不影响运行
                print("[LiveKit] ⚠ 未知连接状态: \(connectionState)")
            }
        }
    }

    /// 成功订阅了远端参与者的一个轨道
    /// 这是收到视频的关键回调
    /// 注意：LiveKit SDK 2.x 新签名只有 publication 参数，track 通过 publication.track 获取
    nonisolated func room(_ room: Room, participant: RemoteParticipant, didSubscribeTrack publication: RemoteTrackPublication) {
        Task { @MainActor [weak self] in
            guard let self else { return }
            let identity = participant.identity?.stringValue ?? "unknown"
            let sidStr = publication.sid.stringValue
            print("[LiveKit] ★ 订阅了 \(identity) 的轨道: sid=\(sidStr)")

            // 从 publication 中获取 track，只处理视频轨道
            if let videoTrack = publication.track as? VideoTrack {
                print("[LiveKit] ★ 收到视频轨道！绑定到渲染器")
                self.attachVideoTrack(videoTrack)
            }
        }
    }

    /// 取消订阅远端轨道（sender 停推或离开）
    /// LiveKit SDK 2.x 新签名：只有 publication 参数
    nonisolated func room(_ room: Room, participant: RemoteParticipant, didUnsubscribeTrack publication: RemoteTrackPublication) {
        Task { @MainActor [weak self] in
            guard let self else { return }
            let identity = participant.identity?.stringValue ?? "unknown"
            print("[LiveKit] 取消订阅 \(identity) 的轨道")

            if let videoTrack = publication.track as? VideoTrack {
                self.detachVideoTrack(videoTrack)
            }
        }
    }

    /// 新参与者加入房间
    /// LiveKit SDK 2.x: participantDidJoin 已重命名为 participantDidConnect
    nonisolated func room(_ room: Room, participantDidConnect participant: RemoteParticipant) {
        Task { @MainActor in
            let identity = participant.identity?.stringValue ?? "unknown"
            print("[LiveKit] 参与者加入: \(identity)")
        }
    }

    /// 参与者离开房间
    /// LiveKit SDK 2.x: participantDidLeave 已重命名为 participantDidDisconnect
    nonisolated func room(_ room: Room, participantDidDisconnect participant: RemoteParticipant) {
        Task { @MainActor [weak self] in
            guard let self else { return }
            let identity = participant.identity?.stringValue ?? "unknown"
            print("[LiveKit] 参与者离开: \(identity)")

            // 如果正在 streaming，sender 离开意味着视频断了
            if self.appState.isReceivingVideo {
                self.appState.isReceivingVideo = false
                self.appState.connectionStatus = .signaling
                self.stopPoseTimer()
                self.stopFPSTimer()
                print("[LiveKit] Sender 离开，等待重新推流...")
            }
        }
    }

    /// 收到远端 Data Channel 数据
    /// LiveKit SDK 2.x: 新增 encryptionType 参数
    /// 目前 receiver 端不需要处理 sender 发来的数据，
    /// 但预留此接口以备后续扩展（如 sender 发送机器人状态反馈）
    nonisolated func room(_ room: Room, participant: RemoteParticipant?, didReceiveData data: Data, forTopic topic: String, encryptionType: EncryptionType) {
        // 未来可在此处理 sender 发来的机器人状态数据
        // 例如：关节角度反馈、力矩传感器数据等
    }
}
