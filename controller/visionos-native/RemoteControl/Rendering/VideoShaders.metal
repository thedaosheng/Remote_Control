#include <metal_stdlib>
using namespace metal;

// =============================================================================
// VideoShaders.metal
//
// visionOS 26 + CompositorServices + Vertex Amplification 的 NV12 视频着色器。
//
// 渲染思路:
//   - 一次 draw 调用绘制 4 顶点 quad,GPU 通过 vertex amplification 把它复制
//     成左右眼两份,分别写到 layered render target 的 slice 0 / slice 1
//   - vertex shader 用 [[amplification_id]] 区分左右眼,从 EyePairUniforms.eyes[id]
//     取 SBS UV 偏移,把 [0,1] 的本地 uv 映射到 SBS 纹理上的对应半边
//   - vertex shader 同时设置 [[render_target_array_index]] = id 让 GPU 把
//     片段写到正确的 slice
//   - fragment shader 直接采样 NV12 双平面,做 BT.601 YCbCr -> RGB 转换
// =============================================================================

// -----------------------------------------------------------------------------
// 顶点输入 / 输出结构体
// -----------------------------------------------------------------------------

struct VertexIn {
    float2 position [[attribute(0)]];   // NDC [-1,1]
    float2 uv       [[attribute(1)]];   // 单位 UV [0,1]
};

struct VertexOut {
    float4 position [[position]];                      // 裁剪空间位置
    float2 uv;                                         // 已做 SBS 偏移的 UV
    uint   layer    [[render_target_array_index]];    // 写入哪个 array slice
};

// -----------------------------------------------------------------------------
// 与 Swift 端 EyeParams / EyePairUniforms 完全匹配的 uniform 结构
// -----------------------------------------------------------------------------

struct EyeParams {
    float uvOffsetX;
    float uvScaleX;
    float uvOffsetY;
    float uvScaleY;
    float _pad0;
    float _pad1;
    float _pad2;
    float _pad3;          // 总大小 32 字节,16 字节对齐
};

struct EyePairUniforms {
    EyeParams eyes[2];    // 0 = 左眼, 1 = 右眼
    uint  isFullRange;
    uint  _pad0;
    uint  _pad1;
    uint  _pad2;
};

// -----------------------------------------------------------------------------
// 顶点着色器 —— 用 amplification_id 选择左右眼参数
// -----------------------------------------------------------------------------

vertex VertexOut vs_video(
    VertexIn  in        [[stage_in]],
    ushort    amp_id    [[amplification_id]],         // 0 = 左眼, 1 = 右眼
    constant  EyePairUniforms& u [[buffer(0)]]
) {
    VertexOut out;

    // quad 已经在 NDC,直接输出
    out.position = float4(in.position, 0.0, 1.0);

    // 选择左右眼参数 (gather load,性能 OK)
    EyeParams eye = u.eyes[amp_id];

    // 把 [0,1] 的 quad uv 映射到 SBS 纹理的对应半边
    out.uv = float2(
        eye.uvOffsetX + in.uv.x * eye.uvScaleX,
        eye.uvOffsetY + in.uv.y * eye.uvScaleY
    );

    // 把片段写到对应的 array slice
    // 这是 layered 布局下让 GPU 知道目标 slice 的关键
    out.layer = uint(amp_id);

    return out;
}

// -----------------------------------------------------------------------------
// 片段着色器 —— NV12 (双平面 YCbCr) 转 RGB
// -----------------------------------------------------------------------------

fragment float4 fs_video_nv12(
    VertexOut          in       [[stage_in]],
    texture2d<float>   yTex     [[texture(0)]],   // Y plane (r8Unorm)
    texture2d<float>   cbcrTex  [[texture(1)]],   // CbCr plane (rg8Unorm)
    constant EyePairUniforms& u [[buffer(0)]]
) {
    constexpr sampler s(filter::linear, address::clamp_to_edge);

    // 采样亮度 / 色度
    float  y_raw    = yTex.sample(s, in.uv).r;
    float2 cbcr_raw = cbcrTex.sample(s, in.uv).rg;

    float y, cb, cr;
    if (u.isFullRange) {
        // kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
        // Y/Cb/Cr 都是 [0,255], Cb/Cr 中心 = 128
        y  = y_raw;
        cb = cbcr_raw.r - 0.5;
        cr = cbcr_raw.g - 0.5;
    } else {
        // kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange (BT.601)
        // Y [16,235], Cb/Cr [16,240]
        y  = (y_raw       - 16.0  / 255.0) / (219.0 / 255.0);
        cb = (cbcr_raw.r - 128.0 / 255.0) / (112.0 / 255.0);
        cr = (cbcr_raw.g - 128.0 / 255.0) / (112.0 / 255.0);
    }

    // ★ DEBUG: 左眼绿色，右眼蓝色 — 验证 amplification 是否工作
    if (in.layer == 0) {
        return float4(0.0, 1.0, 0.0, 1.0);  // 左眼 = 绿色
    } else {
        return float4(0.0, 0.0, 1.0, 1.0);  // 右眼 = 蓝色
    }
    // BT.601 矩阵（注释掉调试时不会执行）
    // float r = saturate(y + 1.402  * cr);
    // float g = saturate(y - 0.3441 * cb - 0.7141 * cr);
    // float b = saturate(y + 1.772  * cb);
    // return float4(r, g, b, 1.0);
}
