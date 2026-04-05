import ARKit
import QuartzCore
import simd

// Provides device head pose relative to the calibration pose (first captured frame).
// Call `start()` once, then read `currentPose` at any time from any thread.
final class HeadPoseTracker: @unchecked Sendable {

    struct Pose: Sendable {
        let position:   SIMD3<Double>   // meters in world space
        let quaternion: simd_quatd      // relative to initial orientation
    }

    private let lock = NSLock()
    private var _currentPose: Pose?
    private var initialInverseQuat: simd_quatd?
    private var session: ARKitSession?
    private var worldProvider: WorldTrackingProvider?

    var currentPose: Pose? { lock.withLock { _currentPose } }

    // MARK: - Lifecycle

    func start() {
        Task.detached { [weak self] in
            await self?.runSession()
        }
    }

    func stop() {
        session?.stop()
    }

    // MARK: - ARKit session

    private func runSession() async {
        let provider = WorldTrackingProvider()
        let session  = ARKitSession()

        lock.withLock {
            self.worldProvider  = provider
            self.session        = session
        }

        do {
            try await session.run([provider])
        } catch {
            return
        }

        for await update in provider.anchorUpdates {
            guard update.event == .updated || update.event == .added else { continue }
            processDeviceAnchor(provider)
        }
    }

    private func processDeviceAnchor(_ provider: WorldTrackingProvider) {
        guard let anchor = provider.queryDeviceAnchor(atTimestamp: CACurrentMediaTime()) else { return }

        let t4x4 = anchor.originFromAnchorTransform
        let pos   = SIMD3<Double>(Double(t4x4.columns.3.x),
                                  Double(t4x4.columns.3.y),
                                  Double(t4x4.columns.3.z))

        // Extract rotation as quaternion from the 3x3 upper-left
        let rotMatrix = simd_double3x3(
            SIMD3<Double>(Double(t4x4.columns.0.x), Double(t4x4.columns.0.y), Double(t4x4.columns.0.z)),
            SIMD3<Double>(Double(t4x4.columns.1.x), Double(t4x4.columns.1.y), Double(t4x4.columns.1.z)),
            SIMD3<Double>(Double(t4x4.columns.2.x), Double(t4x4.columns.2.y), Double(t4x4.columns.2.z))
        )
        let quat = simd_quatd(rotMatrix)

        lock.withLock {
            if initialInverseQuat == nil {
                // Calibrate: record inverse of first orientation
                initialInverseQuat = quat.inverse
            }
            let relativeQuat = initialInverseQuat! * quat
            _currentPose = Pose(position: pos, quaternion: relativeQuat)
        }
    }

    // MARK: - Serialisable accessors for signaling

    func poseArrays() -> (position: [Double], quaternion: [Double])? {
        guard let p = currentPose else { return nil }
        return (
            [p.position.x, p.position.y, p.position.z],
            [p.quaternion.vector.x, p.quaternion.vector.y,
             p.quaternion.vector.z, p.quaternion.vector.w]
        )
    }
}
