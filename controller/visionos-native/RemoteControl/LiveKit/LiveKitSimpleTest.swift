import SwiftUI
import LiveKit

// 最简单的测试：用 LiveKit 的 SwiftUI VideoView 直接显示视频
// 不使用 CompositorServices / Metal / 自定义渲染
// 如果这个能显示视频，说明问题在 Metal 渲染管线

struct LiveKitSimpleTestView: View {
    @State private var room = Room()
    @State private var remoteVideoTrack: VideoTrack?
    @State private var status = "未连接"

    var body: some View {
        VStack(spacing: 20) {
            Text("LiveKit 视频测试").font(.title)
            Text(status).foregroundColor(.gray)

            if let track = remoteVideoTrack {
                // LiveKit 官方 VideoView — 自动处理解码和渲染
                SwiftUIVideoView(track, layoutMode: .fit)
                    .frame(width: 600, height: 300)
                    .border(Color.green, width: 2)
                Text("视频轨道已绑定").foregroundColor(.green)
            } else {
                Rectangle()
                    .fill(Color.gray.opacity(0.3))
                    .frame(width: 600, height: 300)
                    .overlay(Text("等待视频...").foregroundColor(.white))
            }

            Button("连接") {
                Task { await connect() }
            }
            .padding()
            .background(Color.blue)
            .foregroundColor(.white)
            .cornerRadius(8)
        }
        .padding(40)
    }

    func connect() async {
        status = "正在连接..."

        let token = LiveKitConfig.receiverToken

        room.delegates.add(delegate: RoomDelegateHandler { track in
            DispatchQueue.main.async {
                self.remoteVideoTrack = track
                self.status = "已收到视频轨道!"
            }
        })

        do {
            try await room.connect(url: "ws://39.102.113.104:7880", token: token)
            status = "已连接，等待视频..."

            // 检查已有参与者
            for (_, participant) in room.remoteParticipants {
                for (_, pub) in participant.trackPublications {
                    if let track = pub.track as? VideoTrack {
                        remoteVideoTrack = track
                        status = "已收到视频轨道!"
                    }
                }
            }
        } catch {
            status = "连接失败: \(error.localizedDescription)"
        }
    }
}

class RoomDelegateHandler: RoomDelegate {
    let onTrack: (VideoTrack) -> Void
    init(onTrack: @escaping (VideoTrack) -> Void) { self.onTrack = onTrack }

    func room(_ room: Room, participant: RemoteParticipant, didSubscribeTrack publication: RemoteTrackPublication) {
        if let track = publication.track as? VideoTrack {
            onTrack(track)
        }
    }
}
