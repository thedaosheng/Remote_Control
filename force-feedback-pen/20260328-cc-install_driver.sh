#!/bin/bash
# ============================================================================
# 20260328-cc-install_driver.sh - 3D Systems Touch 驱动安装脚本
# 需要 sudo 权限执行
# ============================================================================

set -e

echo "=== 3D Systems Touch 驱动安装脚本 ==="
echo ""

HAPTICS_HOME="/home/rhz/haptics_install"

# 1. 安装驱动二进制文件
echo "[1/4] 安装 Touch 驱动程序..."
sudo cp "${HAPTICS_HOME}/TouchDriver2022_04_04/bin/Touch_Setup" /usr/bin/
sudo cp "${HAPTICS_HOME}/TouchDriver2022_04_04/bin/Touch_Diagnostic" /usr/bin/
sudo chmod +x /usr/bin/Touch_Setup /usr/bin/Touch_Diagnostic

# 2. 安装库文件
echo "[2/4] 安装共享库文件..."
sudo cp "${HAPTICS_HOME}/TouchDriver2022_04_04/usr/lib/libPhantomIOLib42.so" /usr/lib/
sudo cp "${HAPTICS_HOME}/local/lib/libHD.so"* /usr/lib/
sudo cp "${HAPTICS_HOME}/local/lib/libHL.so"* /usr/lib/
sudo ldconfig

# 3. 创建配置目录
echo "[3/4] 创建配置目录..."
sudo mkdir -p /usr/share/3DSystems/config
sudo chmod 777 /usr/share/3DSystems/config

# 4. 设置环境变量
echo "[4/4] 配置环境变量..."
echo 'export GTDD_HOME=/usr/share/3DSystems' | sudo tee /etc/profile.d/3ds-touch-drivers.sh > /dev/null

echo ""
echo "=== 安装完成 ==="
echo "请执行以下命令使环境变量生效:"
echo "  source /etc/profile.d/3ds-touch-drivers.sh"
echo ""
echo "可运行 Touch_Setup 配置设备:"
echo "  Touch_Setup"
