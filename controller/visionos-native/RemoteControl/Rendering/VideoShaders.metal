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
    float  eyeFlag;                                    // 0 = 左眼, 1 = 右眼 (用于 fragment debug)
    // 注意: 不要输出 [[render_target_array_index]]!
    //   viewMapping 里的 renderTargetArrayIndexOffset 已经负责 slice 路由,
    //   如果 shader 再写 amp_id,effective slice = amp_id + offset = 2 (越界),
    //   Metal 会丢弃右眼的所有片段 → 黑屏。
    //   这与 Apple 官方 Compositor Services 模板 (Shaders.metal) 的行为完全一致。
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
// ViewProjectionUniforms —— 每帧 CPU 更新的 world → clip 变换矩阵
// 顶点输出必须通过这组矩阵变换到真正的 clip space,compositor 才能正确 reproject。
// 布局与 Swift 端 struct ViewProjectionUniforms 完全一致(按字段顺序)。
// -----------------------------------------------------------------------------
struct ViewProjectionUniforms {
    float4x4 vpLeft;    // 左眼 world → clip
    float4x4 vpRight;   // 右眼 world → clip
    float4x4 model;     // quad local → world (含 head 位姿 + 前向平移 + 尺寸缩放)
};

// -----------------------------------------------------------------------------
// 顶点着色器 —— 用 amplification_id 选择左右眼参数
// -----------------------------------------------------------------------------

vertex VertexOut vs_video(
    VertexIn  in                   [[stage_in]],
    ushort    amp_id               [[amplification_id]],    // 0 = 左眼, 1 = 右眼
    constant  EyePairUniforms&        u   [[buffer(0)]],
    constant  ViewProjectionUniforms& vpu [[buffer(2)]]
) {
    VertexOut out;

    // 本地 quad 坐标: [-1, 1] 的 2D 矩形,放在 z=0 平面
    // modelMatrix 会把它缩放到物理尺寸并放到头前方 world 位置
    float4 localPos = float4(in.position.x, in.position.y, 0.0, 1.0);

    // 根据 amp_id 选左/右眼的 viewProjection 矩阵
    // (Metal 不支持 constant float4x4 的 index 运算,所以用三元)
    float4x4 vp = (amp_id == 0) ? vpu.vpLeft : vpu.vpRight;

    // 完整变换链: local → world → eye → clip
    //   ★ 这是修"黑屏"的核心 —— 顶点输出必须代表真实的 world-space 位置,
    //     compositor 才能用 deviceAnchor 正确 reproject。
    out.position = vp * vpu.model * localPos;

    // 选择左右眼的 SBS UV 偏移参数 (gather load,性能 OK)
    EyeParams eye = u.eyes[amp_id];

    // 把 [0,1] 的 quad uv 映射到 SBS 纹理的对应半边
    out.uv = float2(
        eye.uvOffsetX + in.uv.x * eye.uvScaleX,
        eye.uvOffsetY + in.uv.y * eye.uvScaleY
    );

    // 把眼睛索引作为一般 varying 传给 fragment(插值后是 0.0 或 1.0)
    // 之前写的 out.layer = amp_id 是 bug —— 会和 viewMapping offset 叠加到 slice 2。
    out.eyeFlag = float(amp_id);

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

    // NV12 双平面采样:
    //   Y 平面   → luminance (r8Unorm 单通道)
    //   CbCr 平面 → chrominance (rg8Unorm 双通道,水平/垂直都是 1/2 分辨率,
    //                            sampler 会做 bilinear 插值自动 upsample)
    float  y_raw    = yTex.sample(s, in.uv).r;
    float2 cbcr_raw = cbcrTex.sample(s, in.uv).rg;

    // YCbCr → 归一化 Y/Cb/Cr(Cb/Cr 已去中心,范围 ~[-0.5, 0.5])
    float y, cb, cr;
    if (u.isFullRange) {
        // kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
        // Y/Cb/Cr 全部 [0,255], Cb/Cr 中心 = 128
        y  = y_raw;
        cb = cbcr_raw.r - 0.5;
        cr = cbcr_raw.g - 0.5;
    } else {
        // kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange (BT.601 video range)
        // Y [16,235], Cb/Cr [16,240]
        y  = (y_raw      - 16.0  / 255.0) / (219.0 / 255.0);
        cb = (cbcr_raw.r - 128.0 / 255.0) / (224.0 / 255.0);
        cr = (cbcr_raw.g - 128.0 / 255.0) / (224.0 / 255.0);
    }

    // BT.601 YCbCr → RGB 变换矩阵
    //   摄像头 / H.264 常用 BT.601,zed mini 默认也是
    //   如果颜色偏 → 切换到 BT.709 矩阵: 1.5748/0.1873/0.4681/1.8556
    float r = saturate(y + 1.402    * cr);
    float g = saturate(y - 0.344136 * cb - 0.714136 * cr);
    float b = saturate(y + 1.772    * cb);

    return float4(r, g, b, 1.0);
}
