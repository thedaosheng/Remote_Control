# operator-vp/ — Apple Vision Pro receiver

This directory will hold the Xcode project (`RemoteControl/RemoteControl/RemoteControl.xcodeproj`) currently on `Zhuanz@192.168.0.212`, including LiveKit Swift SDK integration, head-pose data channel publisher, and emergency-stop command publisher. To be imported in PR-5 once 212 is reliably online.

The working render path is visionOS 26 + LiveKit `SwiftUIVideoView` (NOT the custom Metal CompositorServices path which historically had black-screen issues — kept only as historical reference in `controller/` until reconciled in PR-5).
