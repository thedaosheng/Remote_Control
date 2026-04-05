import Foundation
import CoreVideo
import LiveKitWebRTC

// Receives decoded video frames from WebRTC and stores the latest one
// for the Metal render loop to consume.
final class VideoFrameHandler: NSObject, LKRTCVideoRenderer, @unchecked Sendable {

    private let lock = NSLock()

    private var _latestPixelBuffer: CVPixelBuffer?
    private var _frameCount: Int = 0
    private var _isFullRange: Bool = false

    // Calibration — written by UI (@MainActor), read by render loop (background)
    private var _calibration = Calibration()
    var calibration: Calibration {
        get { lock.withLock { _calibration } }
        set { lock.withLock { _calibration = newValue } }
    }

    // Called by the render loop on its dedicated thread
    nonisolated func consumeLatestFrame()
        -> (buffer: CVPixelBuffer, isFullRange: Bool, calibration: Calibration)?
    {
        lock.withLock {
            guard let buf = _latestPixelBuffer else { return nil }
            return (buf, _isFullRange, _calibration)
        }
    }

    var frameCount: Int { lock.withLock { _frameCount } }

    // MARK: - LKRTCVideoRenderer

    nonisolated func setSize(_ size: CGSize) { }

    private var loggedFirstFrame = false

    nonisolated func renderFrame(_ frame: LKRTCVideoFrame?) {
        guard let frame else { return }

        if !loggedFirstFrame {
            loggedFirstFrame = true
            print("[VideoFrame] ★ FIRST FRAME RECEIVED! width=\(frame.width) height=\(frame.height) buffer=\(type(of: frame.buffer))")
        }

        let count = lock.withLock { _frameCount }
        if count > 0 && count % 300 == 0 {
            print("[VideoFrame] #\(count) frames received")
        }

        if let cvBuffer = frame.buffer as? LKRTCCVPixelBuffer {
            let pb  = cvBuffer.pixelBuffer
            let fmt = CVPixelBufferGetPixelFormatType(pb)
            let full = (fmt == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange)
            lock.withLock {
                _latestPixelBuffer = pb
                _isFullRange = full
                _frameCount += 1
            }
        } else if let i420 = frame.buffer as? LKRTCI420Buffer {
            if let pb = i420ToCVPixelBuffer(i420) {
                lock.withLock {
                    _latestPixelBuffer = pb
                    _isFullRange = true
                    _frameCount += 1
                }
            }
        }
    }

    // MARK: - I420 → NV12 CVPixelBuffer

    private func i420ToCVPixelBuffer(_ i420: LKRTCI420Buffer) -> CVPixelBuffer? {
        let w = Int(i420.width)
        let h = Int(i420.height)
        var pb: CVPixelBuffer?
        let attrs: [CFString: Any] = [
            kCVPixelBufferIOSurfacePropertiesKey: [:] as CFDictionary
        ]
        guard CVPixelBufferCreate(
            nil, w, h,
            kCVPixelFormatType_420YpCbCr8BiPlanarFullRange,
            attrs as CFDictionary, &pb
        ) == kCVReturnSuccess, let pb else { return nil }

        CVPixelBufferLockBaseAddress(pb, [])
        defer { CVPixelBufferUnlockBaseAddress(pb, []) }

        // Copy Y plane
        if let dst = CVPixelBufferGetBaseAddressOfPlane(pb, 0) {
            let stride = CVPixelBufferGetBytesPerRowOfPlane(pb, 0)
            var src = i420.dataY
            for row in 0..<h {
                memcpy(dst.advanced(by: row * stride), src, w)
                src = src.advanced(by: Int(i420.strideY))
            }
        }

        // Interleave Cb+Cr into NV12 UV plane
        if let dst = CVPixelBufferGetBaseAddressOfPlane(pb, 1) {
            let stride = CVPixelBufferGetBytesPerRowOfPlane(pb, 1)
            let uvH = h / 2
            let uvW = w / 2
            var srcU = i420.dataU
            var srcV = i420.dataV
            for row in 0..<uvH {
                let rowPtr = dst.advanced(by: row * stride)
                    .bindMemory(to: UInt8.self, capacity: uvW * 2)
                for col in 0..<uvW {
                    rowPtr[col * 2]     = srcU[col]
                    rowPtr[col * 2 + 1] = srcV[col]
                }
                srcU = srcU.advanced(by: Int(i420.strideU))
                srcV = srcV.advanced(by: Int(i420.strideV))
            }
        }

        return pb
    }
}
