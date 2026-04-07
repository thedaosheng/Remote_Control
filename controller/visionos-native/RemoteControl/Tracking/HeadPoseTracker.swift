import Foundation
import simd

// =============================================================================
// HeadPoseTracker —— 线程安全的 head pose 缓存 (不再独立 run ARKit session)
//
// 为什么改成纯缓存?
//   visionOS 同一个进程只能有一个活跃的 WorldTrackingProvider。
//   之前 HeadPoseTracker 自己 run 了一个 ARKitSession + WorldTrackingProvider,
//   StereoVideoRenderer 进入 immersive space 时又 run 了一个,
//   后启动的把先启动的 "饿死",导致 HeadPoseTracker 的 anchorUpdates 永远
//   拿不到数据,_currentPose 一直是 nil,pose 回传完全不通。
//
// 新设计:
//   StereoVideoRenderer 是唯一运行 ARKit 的地方(它本来就必须跑),
//   每帧渲染时把当前的 originFromAnchorTransform (4x4 float) 推给
//   HeadPoseTracker。LiveKitManager 的 pose timer 读 poseArrays() 拿到
//   最新缓存的 pose 发到 LiveKit Data Channel。
//
// 线程安全:
//   - StereoVideoRenderer 在专用渲染线程调 updateFromTransform()
//   - LiveKitManager 在 @MainActor 上以 30Hz 调 poseArrays()
//   - 用 NSLock 保护 _currentPose / initialInverseQuat
// =============================================================================

final class HeadPoseTracker: @unchecked Sendable {

    struct Pose: Sendable {
        let position:   SIMD3<Double>   // 世界坐标系里的位置(米)
        let quaternion: simd_quatd      // 相对于首帧的旋转(做了校准)
    }

    // 互斥锁保护下列共享状态
    private let lock = NSLock()
    private var _currentPose: Pose?
    private var initialInverseQuat: simd_quatd?

    var currentPose: Pose? { lock.withLock { _currentPose } }

    // -------------------------------------------------------------------------
    // updateFromTransform —— 渲染线程每帧调用
    //
    // 输入: ARKit 的 deviceAnchor.originFromAnchorTransform (4x4 float)
    //
    // 处理:
    //   1. 提取 translation (columns.3 前三个分量) 作为 position
    //   2. 3x3 上三角作为旋转矩阵,转成四元数
    //   3. 首次调用时记录 quat 的 inverse 作为"零点校准",
    //      后续 pose 的 quaternion 都是相对于这个零点的
    //      (这样头第一次进沉浸空间时朝向就是 (0,0,0,1),用户感觉不到偏航偏移)
    // -------------------------------------------------------------------------
    func updateFromTransform(_ transform: simd_float4x4) {
        // 位置:从第 4 列的前 3 个分量读 (Metal 用 column-major)
        let pos = SIMD3<Double>(
            Double(transform.columns.3.x),
            Double(transform.columns.3.y),
            Double(transform.columns.3.z)
        )

        // 旋转:取左上 3x3 子矩阵,转成 double 精度
        // 注意 simd_float4x4 是 column-major,columns.0/1/2 是三个基向量
        let rotMatrix = simd_double3x3(
            SIMD3<Double>(
                Double(transform.columns.0.x),
                Double(transform.columns.0.y),
                Double(transform.columns.0.z)
            ),
            SIMD3<Double>(
                Double(transform.columns.1.x),
                Double(transform.columns.1.y),
                Double(transform.columns.1.z)
            ),
            SIMD3<Double>(
                Double(transform.columns.2.x),
                Double(transform.columns.2.y),
                Double(transform.columns.2.z)
            )
        )
        let quat = simd_quatd(rotMatrix)

        lock.withLock {
            if initialInverseQuat == nil {
                // 第一次:记录零点校准(初始朝向的 inverse)
                initialInverseQuat = quat.inverse
            }
            let relativeQuat = initialInverseQuat! * quat
            _currentPose = Pose(position: pos, quaternion: relativeQuat)
        }
    }

    // -------------------------------------------------------------------------
    // reset —— 断开连接时调用,清掉校准,下次进入 immersive 重新建立零点
    // -------------------------------------------------------------------------
    func reset() {
        lock.withLock {
            _currentPose = nil
            initialInverseQuat = nil
        }
    }

    // -------------------------------------------------------------------------
    // poseArrays —— LiveKitManager 序列化发送时用
    //   返回 (position [x,y,z], quaternion [x,y,z,w])
    //   如果还没收到任何 pose,返回 nil(LiveKitManager 会跳过那次发送)
    // -------------------------------------------------------------------------
    func poseArrays() -> (position: [Double], quaternion: [Double])? {
        guard let p = currentPose else { return nil }
        return (
            [p.position.x, p.position.y, p.position.z],
            [
                p.quaternion.vector.x,
                p.quaternion.vector.y,
                p.quaternion.vector.z,
                p.quaternion.vector.w
            ]
        )
    }
}
