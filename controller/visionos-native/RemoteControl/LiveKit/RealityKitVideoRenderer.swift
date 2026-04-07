import SwiftUI
import RealityKit
import LiveKit
import CoreVideo
import CoreImage

// =============================================================================
// 用 RealityKit 在 ImmersiveSpace 中显示立体视频
// SBS (Side-by-Side) 视频：左半=左眼，右半=右眼
// 两个平面分别对应左右眼，用 x 偏移模拟 IPD
// =============================================================================

struct RealityKitVideoView: View {
    let frameHandler: VideoFrameHandler

    @State private var leftEntity: ModelEntity?
    @State private var rightEntity: ModelEntity?

    // SBS 视频参数
    private let videoWidth: Int = 1344
    private let videoHeight: Int = 376
    private let eyeWidth: Int = 672   // 1344/2
    private let eyeHeight: Int = 376

    // 显示参数
    private let planeWidth: Float = 2.0
    private let planeHeight: Float = 1.12  // 保持 672:376 的宽高比
    private let distance: Float = 1.5      // 前方距离
    private let ipd: Float = 0.032         // 瞳距一半 (64mm / 2)

    var body: some View {
        RealityView { content in
            // 左眼平面 — 略偏左
            let leftMesh = MeshResource.generatePlane(width: planeWidth, height: planeHeight)
            var leftMat = UnlitMaterial()
            leftMat.color = .init(tint: .white)
            let left = ModelEntity(mesh: leftMesh, materials: [leftMat])
            left.position = SIMD3(-ipd, 1.5, -distance)
            content.add(left)
            leftEntity = left

            // 右眼平面 — 略偏右
            let rightMesh = MeshResource.generatePlane(width: planeWidth, height: planeHeight)
            var rightMat = UnlitMaterial()
            rightMat.color = .init(tint: .white)
            let right = ModelEntity(mesh: rightMesh, materials: [rightMat])
            right.position = SIMD3(ipd, 1.5, -distance)
            content.add(right)
            rightEntity = right

            // 启动帧更新
            startFrameUpdater()
        }
    }

    private func startFrameUpdater() {
        Task {
            let ciContext = CIContext(options: [.useSoftwareRenderer: false])
            var frameCount = 0

            while true {
                try? await Task.sleep(for: .milliseconds(16))

                guard let frameData = frameHandler.consumeLatestFrame() else { continue }

                frameCount += 1
                let pb = frameData.buffer
                let fullImage = CIImage(cvPixelBuffer: pb)

                // 裁剪左半（左眼）
                let leftCrop = fullImage.cropped(to: CGRect(x: 0, y: 0, width: eyeWidth, height: eyeHeight))
                // 裁剪右半（右眼）
                let rightCrop = fullImage.cropped(to: CGRect(x: eyeWidth, y: 0, width: eyeWidth, height: eyeHeight))
                    .transformed(by: CGAffineTransform(translationX: CGFloat(-eyeWidth), y: 0))

                guard let leftCG = ciContext.createCGImage(leftCrop, from: CGRect(x: 0, y: 0, width: eyeWidth, height: eyeHeight)),
                      let rightCG = ciContext.createCGImage(rightCrop, from: CGRect(x: 0, y: 0, width: eyeWidth, height: eyeHeight))
                else { continue }

                if frameCount <= 3 || frameCount % 300 == 0 {
                    print("[RealityKit] 帧 #\(frameCount): left=\(leftCG.width)x\(leftCG.height) right=\(rightCG.width)x\(rightCG.height)")
                }

                // 更新左眼纹理
                if let leftEnt = leftEntity {
                    do {
                        let tex = try await TextureResource.generate(from: leftCG, options: .init(semantic: .color))
                        var mat = UnlitMaterial()
                        mat.color = .init(texture: .init(tex))
                        await MainActor.run {
                            leftEnt.model?.materials = [mat]
                        }
                    } catch {}
                }

                // 更新右眼纹理
                if let rightEnt = rightEntity {
                    do {
                        let tex = try await TextureResource.generate(from: rightCG, options: .init(semantic: .color))
                        var mat = UnlitMaterial()
                        mat.color = .init(texture: .init(tex))
                        await MainActor.run {
                            rightEnt.model?.materials = [mat]
                        }
                    } catch {}
                }
            }
        }
    }
}
