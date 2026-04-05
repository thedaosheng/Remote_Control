import SwiftUI

struct ContentView: View {

    @Environment(AppState.self) private var appState
    @Environment(\.openImmersiveSpace)    private var openImmersiveSpace
    @Environment(\.dismissImmersiveSpace) private var dismissImmersiveSpace

    @State private var connectionManager: ConnectionManager?

    var body: some View {
        @Bindable var state = appState

        NavigationStack {
            Form {
                // MARK: Status
                Section("Connection") {
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

                    if let err = appState.errorMessage {
                        Text(err).foregroundStyle(.red).font(.caption)
                    }
                }

                // MARK: Controls
                Section {
                    if appState.connectionStatus == .disconnected {
                        Button("Connect") {
                            let mgr = ConnectionManager(appState: appState)
                            connectionManager = mgr
                            mgr.connect()
                        }
                        .buttonStyle(.borderedProminent)
                    } else {
                        Button("Disconnect", role: .destructive) {
                            connectionManager?.disconnect()
                            connectionManager = nil
                        }
                    }

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

                // MARK: Calibration
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

    private var statusColor: Color {
        switch appState.connectionStatus {
        case .disconnected: return .red
        case .connecting:   return .orange
        case .signaling:    return .yellow
        case .streaming:    return .green
        }
    }
}

// MARK: - Helper

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
    ContentView()
        .environment(AppState())
}
