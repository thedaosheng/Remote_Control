//
//  LiveKitContentView.swift
//  RemoteControl
//
//  Created on 2026-04-06.
//  基于原 ContentView.swift 修改，将 ConnectionManager 替换为 LiveKitManager。
//  UI 布局、Calibration 部分完全复用，只改连接管理逻辑。
//

import SwiftUI

struct LiveKitContentView: View {

    @Environment(AppState.self) private var appState
    @Environment(\.openImmersiveSpace)    private var openImmersiveSpace
    @Environment(\.dismissImmersiveSpace) private var dismissImmersiveSpace

    /// LiveKitManager 替代原来的 ConnectionManager
    /// 内部管理 Room 连接、视频订阅、pose 发送
    @State private var liveKitManager: LiveKitManager?

    var body: some View {
        @Bindable var state = appState

        NavigationStack {
            Form {
                // MARK: 连接状态区域
                Section("Connection (LiveKit)") {
                    HStack {
                        Circle()
                            .fill(statusColor)
                            .frame(width: 10, height: 10)
                        Text(appState.connectionStatus.rawValue)
                            .font(.headline)
                        Spacer()
                        if appState.isReceivingVideo {
                            Label(String(format: "%.0f fps", appState.fps), systemImage: "video.fill")
                                .foregroundStyle(.green)
                                .font(.caption)
                        }
                    }

                    // 显示 LiveKit 服务器信息
                    Text("Server: 39.102.113.104:7880")
                        .font(.caption2)
                        .foregroundStyle(.secondary)

                    // 错误信息显示
                    if let err = appState.errorMessage {
                        Text(err).foregroundStyle(.red).font(.caption)
                    }
                }

                // MARK: 连接控制按钮
                Section {
                    if appState.connectionStatus == .disconnected {
                        Button("Connect via LiveKit") {
                            // 创建 LiveKitManager 并连接
                            let mgr = LiveKitManager(appState: appState)
                            liveKitManager = mgr
                            mgr.connect()
                        }
                        .buttonStyle(.borderedProminent)
                    } else {
                        Button("Disconnect", role: .destructive) {
                            liveKitManager?.disconnect()
                            liveKitManager = nil
                        }
                    }

                    // 沉浸空间控制（仅在收到视频流时可用）
                    if appState.connectionStatus == .streaming {
                        Button(appState.isImmersiveSpaceOpen
                               ? "Exit Immersive Space"
                               : "Enter Immersive Space") {
                            Task {
                                if appState.isImmersiveSpaceOpen {
                                    await dismissImmersiveSpace()
                                    appState.isImmersiveSpaceOpen = false
                                } else {
                                    let result = await openImmersiveSpace(id: "stereoVideo")
                                    if case .opened = result {
                                        appState.isImmersiveSpaceOpen = true
                                    }
                                }
                            }
                        }
                        .buttonStyle(.bordered)
                    }
                }

                // MARK: 标定参数调节（完全复用原来的）
                Section("Calibration") {
                    SliderRow(label: "Scale",        value: $state.calibration.scale,
                              range: 0.5...1.0, format: "%.2f")
                    SliderRow(label: "V-Scale",      value: $state.calibration.vscale,
                              range: 0.5...1.0, format: "%.2f")
                    SliderRow(label: "Left H-Offset", value: $state.calibration.leftHOffset,
                              range: -0.1...0.1, format: "%.3f")
                    SliderRow(label: "Right H-Offset", value: $state.calibration.rightHOffset,
                              range: -0.1...0.1, format: "%.3f")
                    SliderRow(label: "V-Offset",     value: $state.calibration.vOffset,
                              range: -0.1...0.1, format: "%.3f")

                    Button("Reset Calibration") {
                        appState.calibration = Calibration()
                    }
                    .foregroundStyle(.secondary)
                }
            }
            .navigationTitle("RemoteControl")
        }
    }

    /// 根据连接状态返回对应的指示灯颜色
    private var statusColor: Color {
        switch appState.connectionStatus {
        case .disconnected: return .red
        case .connecting:   return .orange
        case .signaling:    return .yellow
        case .streaming:    return .green
        }
    }
}

// MARK: - SliderRow 辅助视图
// 标定参数滑条（与原 ContentView 完全一致）

private struct SliderRow: View {
    let label: String
    @Binding var value: Float
    let range: ClosedRange<Float>
    let format: String

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            HStack {
                Text(label)
                Spacer()
                Text(String(format: format, value))
                    .monospacedDigit()
                    .foregroundStyle(.secondary)
            }
            Slider(value: $value, in: range)
        }
    }
}

#Preview(windowStyle: .automatic) {
    LiveKitContentView()
        .environment(AppState())
}
