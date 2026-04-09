# MC02 micro-ROS 嵌入式通信节点

达妙 MC02 (STM32H723VGT6) 通过 USB CDC 虚拟串口与主机 ROS2 通信的完整方案。

## 硬件

| 项目 | 规格 |
|------|------|
| 开发板 | 达妙 DM-MC-Board02 (MC02) |
| 主控 | STM32H723VGT6, Cortex-M7, 550MHz |
| USB | Type-C → PA11(DM)/PA12(DP), OTG HS (FS mode) |
| 调试器 | STLink V2 (SWD) |
| 通信协议 | micro-ROS (XRCE-DDS over USB CDC serial) |

## 架构

```
MC02 (STM32H723)                         Linux PC
┌────────────────┐                    ┌──────────────────┐
│ FreeRTOS Task  │                    │ micro-ROS Agent  │
│  mc02_node     │     USB CDC        │  (serial bridge) │
│  ↓             │ ←──────────────→   │       ↓          │
│ rcl_publish()  │   /dev/ttyACMx     │   ROS2 DDS       │
│  std_msgs/Str  │   Type-C cable     │       ↓          │
│  LED 心跳      │                    │ ros2 topic echo  │
└────────────────┘                    │  /mc02_hello     │
                                      └──────────────────┘
```

## 快速开始

### 1. 安装 PlatformIO CLI

```bash
pip3 install --user platformio
export PATH="$HOME/.local/bin:$PATH"  # 加到 .bashrc
```

### 2. 安装 udev rules（一次性）

```bash
sudo cp ~/.local/lib/python3.10/site-packages/platformio/assets/system/99-platformio-udev.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 3. 编译 micro-ROS Agent（一次性）

```bash
mkdir -p ~/microros_ws/src && cd ~/microros_ws/src
git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git
git clone -b humble https://github.com/micro-ROS/micro_ros_msgs.git
cd ~/microros_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 4. 首次编译固件（自动下载 micro-ROS 库，约 3 分钟）

```bash
cd /path/to/RemoteControl/mc02_microros
platformio run
```

> ⚠️ 首次编译会自动 clone + 交叉编译 micro-ROS 静态库，期间可能遇到 `rosidl_typesupport_cpp` 错误，见下方"踩坑记录"。

### 5. 烧录 + 启动

```bash
# 烧录（STLink V2 通过 SWD 连接）
platformio run --target upload

# 烧录后立即启动 Agent（时序关键！）
source /opt/ros/humble/setup.bash
source ~/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM4 -b 115200

# 另一个终端验证
ros2 topic echo /mc02_hello
# 输出: data: 'Hello from MC02 #52'
```

> ⚠️ Agent 必须在 MCU 复位后 **立即启动**。MCU 有 2s USB 枚举 + 5s 稳定期，Agent 会自动等待设备出现。

## 文件结构

```
mc02_microros/
├── platformio.ini              ← PlatformIO 项目配置
├── Inc/
│   ├── stm32h7xx_hal_conf.h    ← HAL 模块选择（启用 GPIO/UART/PCD/TIM）
│   └── FreeRTOSConfig.h        ← FreeRTOS 参数（96KB 堆, 32 优先级）
├── src/
│   ├── main.c                  ← 主程序：时钟/LED/USB/FreeRTOS/micro-ROS
│   ├── custom_transport.c      ← micro-ROS 传输层（USB CDC 读写）
│   ├── custom_transport.h
│   ├── clock_gettime.c         ← POSIX clock_gettime（基于 HAL_GetTick）
│   └── syscalls.c              ← _sbrk 等 newlib 桩函数
├── lib/
│   ├── FreeRTOS/               ← FreeRTOS 内核（ARM_CM3 port, heap_4）
│   │   ├── include/            ← FreeRTOS 头文件
│   │   ├── src/                ← 内核源码 + port.c + heap_4.c
│   │   └── library.json
│   └── USB_CDC/                ← STM32 USB Device CDC 类
│       ├── include/            ← USB Core + CDC 类头文件
│       ├── src/                ← USB 底层驱动 + CDC 接口 + 描述符
│       └── library.json
└── README.md
```

## 踩坑记录（按严重程度排序）

### 1. `transport_close` 不能关闭 USB（最致命）

**现象**：`rmw_uros_ping_agent` 失败后 USB 立即断开，设备从 PC 消失。

**根因**：`custom_transport_close` 中调用了 `USBD_Stop()` + `USBD_DeInit()`，直接关闭了 USB OTG 外设。micro-ROS 在每次 ping 失败后都会调用 close → open 循环。

**修复**：`transport_close` 中不做任何操作（USB 是全局资源，在 main 中初始化后永不关闭）。

### 2. `_sbrk` 在 FreeRTOS 任务中返回 -1

**现象**：micro-ROS 内部 `malloc` 全部失败，触发 HardFault。

**根因**：默认 `_sbrk` 实现使用 SP 寄存器作为堆上界检查。但 FreeRTOS 任务的 SP 指向 heap_4 分配的任务栈（低地址），而 `_end`（堆起点）在 BSS 末尾（高地址）。`heap_end > sp` 总是成立，`_sbrk` 永远返回 -1。

**修复**：用 AXI SRAM 末尾固定地址 `0x24050000` 作为堆上界，不依赖 SP。

### 3. FreeRTOS 堆与 malloc 堆的空间竞争

**现象**：FreeRTOS heap_4 设为 256KB，AXI SRAM 只有 320KB，`_sbrk` 无空间可分。

**修复**：FreeRTOS 堆缩小到 96KB，任务栈缩小到 4096 words (16KB)，留约 200KB 给 micro-ROS 的 malloc。

### 4. `rosidl_typesupport_cpp` 编译错误

**现象**：micro-ROS 库首次编译时报 `No 'rosidl_typesupport_cpp' found`。

**根因**：micro_ros_platformio 的构建脚本在隔离环境中编译，`--packages-ignore-regex=.*_cpp` 忽略了 cpp 包，但 `rosidl_default_generators` 的 cmake 模板中仍硬编码了对 cpp typesupport 的查找。

**修复**：手动 patch `.pio/libdeps/mc02/micro_ros_platformio/build/mcu/src/rosidl_defaults/rosidl_default_generators/rosidl_default_generators-extras.cmake.in`，移除 cpp typesupport 引用。构建脚本 `library_builder.py` 中增加自动 patch 逻辑。

### 5. STM32H723VGT6 没有 USB_OTG_FS

**现象**：编译报 `USB_OTG_FS undeclared`。

**根因**：H723VGT6 (LQFP100) 只有 `USB_OTG_HS` 外设。PA11/PA12 连接到 OTG HS，但使用内嵌 FS PHY 工作在 Full Speed 模式。

**修复**：所有代码中 `USB_OTG_FS` → `USB_OTG_HS`，`OTG_FS_IRQn` → `OTG_HS_IRQn`，时钟宏用 `__HAL_RCC_USB1_OTG_HS_CLK_ENABLE()`。

### 6. USB 需要 48MHz 时钟源

**现象**：USB 枚举失败（PC 看不到设备）。

**修复**：在 `SystemClock_Config` 中启用 HSI48 振荡器，并设置 `RCC_USBCLKSOURCE_HSI48`。

### 7. libmicroros.a 使用 soft float

**现象**：链接时 `VFP register arguments` 不匹配。

**修复**：统一使用 `-mfloat-abi=soft`，FreeRTOS 改用 ARM_CM3 port（无 FPU 上下文切换）。

### 8. Agent 时序敏感

**现象**：Agent 后启动时收不到 MCU 数据，先启动时能成功。

**根因**：Agent 打开 CDC 端口时可能触发 USB 设备 re-init，打断正在进行的 XRCE 帧传输。

**修复**：MCU 侧增加 5 秒初始等待（`vTaskDelay(5000)`），ping 超时设为 1000ms / 3 次重试。Agent 在 MCU 复位后立即启动，利用 `Serial port not found` 自动重连机制。

### 9. 调试文本污染 XRCE 帧流

**现象**：Agent 报 `invalid client key: 0x00000000` 或完全不响应。

**根因**：在 micro-ROS 任务中通过 `CDC_Transmit_FS()` 发送调试文本，这些文本和 XRCE-DDS 二进制帧混在同一个 CDC 端口上，破坏了 Agent 的帧解析。

**修复**：**绝对不要**在 micro-ROS 运行期间通过同一个 CDC 端口发送任何非 XRCE 数据。调试信息用 LED 闪烁代替，或使用独立的 UART 端口。

## 下一步

- [ ] 替换 `std_msgs/String` 为 `sensor_msgs/JointState`（双臂关节状态）
- [ ] 添加订阅者接收关节指令
- [ ] 集成 CAN 总线驱动（MC02 → 达妙电机）
- [ ] 实现 Agent 自动重连（MCU 侧状态机）
