# ============================================================================
# Makefile - 3D Systems Touch 力反馈笔演示程序编译脚本
# ============================================================================

# OpenHaptics SDK 路径
HAPTICS_HOME = /home/rhz/haptics_install/local

# 编译器和标志
CC = gcc
CFLAGS = -Wall -Wextra -O2 -I$(HAPTICS_HOME)/include
LDFLAGS = -L$(HAPTICS_HOME)/lib -Wl,-rpath,$(HAPTICS_HOME)/lib
LIBS = -lHD -lPhantomIOLib42 -lm -lpthread -lrt -ldl

# 目标文件
TARGET = 20260328-cc-haptic_demo
SOURCE = 20260328-cc-haptic_demo.c

# 默认目标：编译程序
all: $(TARGET)

$(TARGET): $(SOURCE)
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS) $(LIBS)

# 运行程序（自动设置环境变量）
run: $(TARGET)
	@echo "正在启动力反馈笔演示程序..."
	@GTDD_HOME=$(shell pwd)/config \
	 LD_LIBRARY_PATH=$(HAPTICS_HOME)/lib:$$LD_LIBRARY_PATH \
	 ./$(TARGET)

# 清理编译产物
clean:
	rm -f $(TARGET)

# 查看设备信息
check-device:
	@echo "=== USB 设备 ==="
	@lsusb | grep -i "3D Systems" || echo "未发现 3D Systems 设备"
	@echo ""
	@echo "=== 串口设备 ==="
	@ls -la /dev/ttyACM* 2>/dev/null || echo "无 ttyACM 设备"
	@echo ""
	@echo "=== 用户组 ==="
	@groups | grep dialout && echo "OK: 用户在 dialout 组" || echo "警告: 用户不在 dialout 组"

.PHONY: all run clean check-device
