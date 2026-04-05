#include <metal_stdlib>
using namespace metal;

// ---------------------------------------------------------------------------
// Shared types
// ---------------------------------------------------------------------------

struct VertexIn {
    float2 position [[attribute(0)]];
    float2 uv       [[attribute(1)]];
};

struct VertexOut {
    float4 position [[position]];
    float2 uv;
};

// Per-eye UV transform + YCbCr range flag.
// Passed to both vertex and fragment stages via buffer(0).
struct EyeUniforms {
    float uvOffsetX;   // horizontal offset into the SBS texture
    float uvScaleX;    // horizontal scale  (= 0.5 * calibration.scale)
    float uvOffsetY;   // vertical offset
    float uvScaleY;    // vertical scale    (= calibration.vscale)
    uint  isFullRange; // 1 = full range, 0 = video range BT.601
    float3 _pad;
};

// ---------------------------------------------------------------------------
// Vertex shader — maps NDC quad UV to per-eye texture UV
// ---------------------------------------------------------------------------

vertex VertexOut vs_video(VertexIn in           [[stage_in]],
                          constant EyeUniforms& u [[buffer(0)]]) {
    VertexOut out;
    out.position = float4(in.position, 0.0, 1.0);
    out.uv = float2(
        u.uvOffsetX + in.uv.x * u.uvScaleX,
        u.uvOffsetY + in.uv.y * u.uvScaleY
    );
    return out;
}

// ---------------------------------------------------------------------------
// Fragment shader — NV12 (biplanar YCbCr) → linear RGB
// ---------------------------------------------------------------------------

fragment float4 fs_video_nv12(VertexOut         in      [[stage_in]],
                               texture2d<float>  yTex    [[texture(0)]],
                               texture2d<float>  cbcrTex [[texture(1)]],
                               constant EyeUniforms& u   [[buffer(0)]]) {
    constexpr sampler s(filter::linear, address::clamp_to_edge);

    float  y_raw    = yTex.sample(s, in.uv).r;
    float2 cbcr_raw = cbcrTex.sample(s, in.uv).rg;

    float y, cb, cr;
    if (u.isFullRange) {
        // kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
        y  = y_raw;
        cb = cbcr_raw.r - 0.5;
        cr = cbcr_raw.g - 0.5;
    } else {
        // kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange  (BT.601)
        y  = (y_raw          - 16.0  / 255.0) / (219.0 / 255.0);
        cb = (cbcr_raw.r - 128.0 / 255.0) / (112.0 / 255.0);
        cr = (cbcr_raw.g - 128.0 / 255.0) / (112.0 / 255.0);
    }

    float r = saturate(y + 1.402  * cr);
    float g = saturate(y - 0.3441 * cb - 0.7141 * cr);
    float b = saturate(y + 1.772  * cb);

    return float4(r, g, b, 1.0);
}
