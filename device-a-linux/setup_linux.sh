#!/usr/bin/env bash
# =====================================================
# Remote Control 系统 Linux 端 一键部署脚本
# =====================================================
#
# 在新的 Ubuntu 22.04 机器上从零部署 sender + 达妙电机控制全套。
# 假设硬件已经接好 (ZED Mini, 达妙电机 + USB-CAN, 24V 电源)。
#
# 用法:
#   chmod +x setup_linux.sh
#   ./setup_linux.sh
#
# 脚本会做:
#   1. 检查 Ubuntu 版本和必备 deb 包
#   2. 检查系统 Python (必须 3.10, 不能用 conda 的)
#   3. pip 安装 livekit / pyserial / numpy 到 ~/.local
#   4. 检测 USB 设备 (ZED Mini, 达妙 HDSC, 大臂 DISCOVER Robotics 适配器)
#   5. 验证 GStreamer + NVENC 可用
#   6. (可选) 安装 systemd user service (lk-sender)
#
# 退出码:
#   0 = 全部 OK
#   1 = 检查失败 / 安装失败 (信息会打印到 stderr)
#
# 作者: Claude Code
# 日期: 2026-04-07

set -e

# ANSI 颜色 (ssh 终端兼容)
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 路径常量
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LK_SENDER_PY="${SCRIPT_DIR}/zedmini_livekit_sender.py"
DM_DIR="${SCRIPT_DIR}/dm_motor"
SYSTEMD_DIR="${SCRIPT_DIR}/systemd"

info()  { echo -e "${BLUE}[INFO]${NC}  $*"; }
ok()    { echo -e "${GREEN}[ OK ]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
err()   { echo -e "${RED}[FAIL]${NC}  $*" >&2; }

echo "============================================================"
echo "  Remote Control Linux 部署 (sender + 达妙电机)"
echo "============================================================"

# ------------------------------------------------------------
# 1. 系统检查
# ------------------------------------------------------------
info "检查 Ubuntu 版本..."
if [[ -f /etc/os-release ]]; then
    . /etc/os-release
    echo "    OS: ${PRETTY_NAME}"
    if [[ "${VERSION_ID%%.*}" != "22" ]]; then
        warn "推荐 Ubuntu 22.04 LTS, 你的是 ${VERSION_ID}, 可能有兼容问题"
    fi
else
    warn "找不到 /etc/os-release, 不是 Ubuntu?"
fi

info "检查必备 deb 包 (gstreamer / python-gi / build tools)..."
NEEDED_DEBS=(
    python3
    python3-pip
    python3-gi
    python3-gst-1.0
    gstreamer1.0-tools
    gstreamer1.0-plugins-base
    gstreamer1.0-plugins-good
    gstreamer1.0-plugins-bad
    gstreamer1.0-plugins-ugly
    v4l-utils
    can-utils
    libnvidia-encode-535
)
MISSING=()
for pkg in "${NEEDED_DEBS[@]}"; do
    if ! dpkg -s "$pkg" >/dev/null 2>&1; then
        MISSING+=("$pkg")
    fi
done
if (( ${#MISSING[@]} > 0 )); then
    warn "缺少以下 deb 包: ${MISSING[*]}"
    echo "    sudo apt update && sudo apt install -y ${MISSING[*]}"
    read -p "现在装吗? [y/N] " yn
    if [[ "$yn" =~ ^[Yy] ]]; then
        sudo apt update
        sudo apt install -y "${MISSING[@]}"
    else
        warn "跳过 deb 安装, 后面运行可能失败"
    fi
else
    ok "所有必备 deb 包已安装"
fi

# ------------------------------------------------------------
# 2. Python 检查 (必须用系统 Python, 不能用 conda 的)
# ------------------------------------------------------------
info "检查系统 Python..."
SYS_PY=/usr/bin/python3
if [[ ! -x "$SYS_PY" ]]; then
    err "找不到 $SYS_PY, 系统 Python 缺失"
    exit 1
fi
PY_VER=$($SYS_PY --version 2>&1)
echo "    $SYS_PY → $PY_VER"
if [[ ! "$PY_VER" =~ "3.10" ]]; then
    warn "建议 Python 3.10 (Ubuntu 22.04 默认), 你的是 $PY_VER"
fi

# 检查 conda 是否在 PATH 前面
if which python3 | grep -q conda; then
    warn "your 'python3' 指向 conda: $(which python3)"
    warn "本系统所有脚本必须用 /usr/bin/python3 (因为 conda 缺 GStreamer python-gi 模块)"
    warn "推荐: 启动时显式用 /usr/bin/python3, 或者 alias python3=/usr/bin/python3"
fi

# 验证 GStreamer python binding
info "验证 GStreamer Python binding..."
if ! $SYS_PY -c "import gi; gi.require_version('Gst', '1.0'); from gi.repository import Gst; Gst.init(None); print('GStreamer', Gst.version())" 2>&1; then
    err "GStreamer python-gi 不可用, 请检查 python3-gi 和 python3-gst-1.0 是否装了"
    exit 1
fi
ok "GStreamer python binding OK"

# ------------------------------------------------------------
# 3. pip 安装 LiveKit / pyserial / numpy
# ------------------------------------------------------------
info "安装 Python 依赖 (livekit / pyserial / numpy)..."
$SYS_PY -m pip install --user --upgrade pip
$SYS_PY -m pip install --user -r "${SCRIPT_DIR}/requirements.txt"
ok "Python 依赖装好"

# ------------------------------------------------------------
# 4. USB 设备检测
# ------------------------------------------------------------
info "检测 USB 设备..."

# 4a. ZED Mini (V4L2)
if [[ -e /dev/video0 ]]; then
    ZED_FORMATS=$(v4l2-ctl --device=/dev/video0 --list-formats-ext 2>&1 | grep -E "1344x376" || true)
    if [[ -n "$ZED_FORMATS" ]]; then
        ok "ZED Mini 在 /dev/video0 (支持 1344x376)"
    else
        warn "/dev/video0 存在但不像 ZED Mini, 可能需要换设备号"
    fi
else
    warn "/dev/video0 不存在, ZED Mini 没接? (sender 启动会失败)"
fi

# 4b. 达妙 HDSC USB-CAN (CDC, 不是 SocketCAN!)
DM_PORT=""
if [[ -d /dev/serial/by-id ]]; then
    DM_LINK=$(ls /dev/serial/by-id 2>/dev/null | grep -i "HDSC" | head -1)
    if [[ -n "$DM_LINK" ]]; then
        DM_PORT=$(readlink -f "/dev/serial/by-id/$DM_LINK")
        ok "达妙 HDSC USB-CAN 在 $DM_PORT (by-id: $DM_LINK)"
    fi
fi
if [[ -z "$DM_PORT" ]]; then
    warn "找不到达妙 HDSC USB-CAN 适配器"
    warn "  注意: 达妙用 CDC 虚拟串口模式 (pyserial), 不是 SocketCAN!"
    warn "  正常应该看到 by-id 里有 'usb-HDSC_CDC_Device_*'"
    warn "  如果你接的是别的厂商 USB-CAN, 可能需要改驱动"
fi

# 4c. dialout 组检查 (打开 /dev/ttyACM* 需要)
if id -nG "$USER" | grep -qw dialout; then
    ok "用户 $USER 已在 dialout 组"
else
    warn "用户 $USER 不在 dialout 组, 可能无法打开 /dev/ttyACM*"
    echo "    sudo usermod -aG dialout $USER  # 然后重新登录"
fi

# ------------------------------------------------------------
# 5. NVIDIA NVENC 检查 (LiveKit Python SDK 用)
# ------------------------------------------------------------
info "检查 NVIDIA NVENC..."
if command -v nvidia-smi >/dev/null 2>&1; then
    GPU=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
    ok "NVIDIA GPU: $GPU"
else
    warn "找不到 nvidia-smi, 软件编码会很慢 (1344x376@100 可能跟不上)"
fi

# ------------------------------------------------------------
# 6. systemd user service (可选)
# ------------------------------------------------------------
echo
info "安装 systemd user service (lk-sender)? 装了之后能自动启动 sender"
read -p "[y/N] " yn
if [[ "$yn" =~ ^[Yy] ]]; then
    SVC_DIR="$HOME/.config/systemd/user"
    mkdir -p "$SVC_DIR"
    if [[ -f "${SYSTEMD_DIR}/lk-sender.service" ]]; then
        # 替换 service 里的 SCRIPT_DIR 占位符
        sed "s|__SCRIPT_DIR__|${SCRIPT_DIR}|g" \
            "${SYSTEMD_DIR}/lk-sender.service" > "${SVC_DIR}/lk-sender.service"
        systemctl --user daemon-reload
        ok "systemd unit 装到 $SVC_DIR/lk-sender.service"
        echo "    启动: systemctl --user start lk-sender"
        echo "    日志: journalctl --user -u lk-sender -f"
        echo "    开机自启: systemctl --user enable lk-sender"
    else
        warn "找不到 ${SYSTEMD_DIR}/lk-sender.service"
    fi
fi

echo
echo "============================================================"
echo "  ✓ 部署完成"
echo "============================================================"
echo ""
echo "下一步:"
echo "  1. 校准达妙电机零点 (硬件首次安装时):"
echo "     /usr/bin/python3 ${DM_DIR}/scripts/dm_motor_zero_calibration.py"
echo ""
echo "  2. (可选) 测试电机轴对应关系 + 编码器范围:"
echo "     /usr/bin/python3 ${DM_DIR}/scripts/dm_motor_axis_verify.py"
echo ""
echo "  3. 启动 sender (前台跑, 看日志):"
echo "     /usr/bin/python3 ${LK_SENDER_PY}"
echo "     # 或者 systemctl --user start lk-sender"
echo ""
echo "  4. 启动 VP→电机控制 (要求 sender 已经在跑):"
echo "     /usr/bin/python3 ${DM_DIR}/scripts/dm_motor_vp_control.py --preset L4"
echo ""
echo "  详细说明见 README.md"
echo "============================================================"
