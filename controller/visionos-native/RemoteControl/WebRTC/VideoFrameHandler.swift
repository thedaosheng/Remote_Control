import Foundation
import CoreVideo
import LiveKit

// 接收 LiveKit 解码后的视频帧，缓存最新一帧供 Metal 渲染循环消费。
// 实现 LiveKit SDK 2.x 的 VideoRenderer 协议。
final class VideoFrameHandler: NSObject, VideoRenderer, @unchecked Sendable {

    private let lock = NSLock()
    private var _latestPixelBuffer: CVPixelBuffer?
    private var _frameCount: Int = 0
    private var _isFullRange: Bool = false
    private var loggedFirstFrame = false

    // MARK: - Calibration
    private var _calibration = Calibration()
    var calibration: Calibration {
        get { lock.withLock { _calibration } }
        set { lock.withLock { _calibration = newValue } }
    }

    // MARK: - 渲染循环消费接口
    nonisolated func consumeLatestFrame()
        -> (buffer: CVPixelBuffer, isFullRange: Bool, calibration: Calibration)?
    {
        lock.withLock {
            guard let buf = _latestPixelBuffer else { return nil }
            return (buf, _isFullRange, _calibration)
        }
    }

    var frameCount: Int { lock.withLock { _frameCount } }

    // MARK: - VideoRenderer 协议
    @MainActor var isAdaptiveStreamEnabled: Bool { false }
    @MainActor var adaptiveStreamSize: CGSize { .zero }
    nonisolated func set(size: CGSize) { }

    nonisolated func render(frame: VideoFrame) {
        // 首帧日志：打印像素格式用于调试
        if !loggedFirstFrame {
            loggedFirstFrame = true
            if let cvBuf = frame.buffer as? CVPixelVideoBuffer {
                let pb = cvBuf.pixelBuffer
                let fmt = CVPixelBufferGetPixelFormatType(pb)
                let planes = CVPixelBufferGetPlaneCount(pb)
                let w = CVPixelBufferGetWidth(pb)
                let h = CVPixelBufferGetHeight(pb)
                print("[VideoFrame] ★ FIRST FRAME! \(w)x\(h) format=0x\(String(fmt, radix:16)) planes=\(planes)")
            } else {
                print("[VideoFrame] ★ FIRST FRAME! \(frame.dimensions.width)x\(frame.dimensions.height) buffer=\(type(of: frame.buffer))")
            }
        }

        let count = lock.withLock { _frameCount }
        if count > 0 && count % 300 == 0 {
            print("[VideoFrame] #\(count) frames received")
        }

        // 路径1：CVPixelVideoBuffer
        if let cvBuffer = frame.buffer as? CVPixelVideoBuffer {
            let pb = cvBuffer.pixelBuffer
            let fmt = CVPixelBufferGetPixelFormatType(pb)
            let full = (fmt == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange)
            lock.withLock {
                _latestPixelBuffer = pb
                _isFullRange = full
                _frameCount += 1
            }
            return
        }

        // 路径2：I420VideoBuffer
        if let i420 = frame.buffer as? I420VideoBuffer {
            if let pb = i420.toPixelBuffer() {
                lock.withLock {
                    _latestPixelBuffer = pb
                    _isFullRange = true
                    _frameCount += 1
                }
            }
            return
        }

        print("[VideoFrame] ⚠ Unknown buffer: \(type(of: frame.buffer))")
    }
}
