# Touch 力反馈笔 — Agent 集成指南

> 本文档面向 AI Agent。如果你需要操控 Touch 设备、编写力渲染程序、或者集成到遥操作系统中，读这个。

## 1. 部署前置条件

```bash
# 确认设备在线
ls /dev/ttyACM* | head -5
lsusb | grep 2988:0302   # 应有输出

# 确认驱动库存在
ls /usr/lib/libHD.so /usr/lib/libPhantomIOLib42.so

# 部署 (补丁库 + 编译 demo)
bash TouchUsage/scripts/deploy.sh
```

## 2. 编程接口 (C, dlopen 方式)

Touch 驱动是闭源的 `libHD.so`，必须通过 `dlopen` + 函数指针调用。**不能用 Python ctypes** (hdGetError 返回结构体会导致 core dump)。

### 最小代码框架

```c
#include <dlfcn.h>

// 1. 加载库 (顺序重要!)
dlopen("/tmp/fakelibs/libncurses.so.5", RTLD_NOW|RTLD_GLOBAL);  // 假库
void* lib = dlopen("/usr/lib/libHD.so", RTLD_NOW|RTLD_GLOBAL);

// 2. 获取函数指针
typedef unsigned int HHD;
HHD (*hdInit)(const char*) = dlsym(lib, "hdInitDevice");
void (*hdBegin)(HHD)       = dlsym(lib, "hdBeginFrame");
void (*hdEnd)(HHD)         = dlsym(lib, "hdEndFrame");
void (*hdGetD)(unsigned,double*) = dlsym(lib, "hdGetDoublev");
void (*hdSetD)(unsigned,const double*) = dlsym(lib, "hdSetDoublev");
void (*hdEn)(unsigned)     = dlsym(lib, "hdEnable");
// ... 完整列表见 src/force_background.c

// 3. 初始化
HHD hHD = hdInit(NULL);  // 使用 ~/.3dsystems/config/Default Device.config
hdEn(0x4000);            // HD_FORCE_OUTPUT, 必须! 否则无力输出

// 4. 注册 1kHz 伺服回调
unsigned int my_servo(void* data) {
    hdBegin(hHD);
    double pos[3]; hdGetD(0x2050, pos);     // HD_CURRENT_POSITION
    double force[3] = {0, 0, 1.0};         // +Z 方向 1N
    hdSetD(0x2700, force);                  // HD_CURRENT_FORCE
    hdEnd(hHD);
    return 1;  // HD_CALLBACK_CONTINUE
}

// 5. 启动调度器
hdScheduleAsynchronous(my_servo, NULL, 500);
hdStartScheduler();  // 返回 0x103 可忽略
```

### 关键常量

```
HD_CURRENT_BUTTONS       = 0x2000  // int, bit0=白色按钮 bit1=灰色按钮
HD_CURRENT_POSITION      = 0x2050  // double[3], mm
HD_CURRENT_VELOCITY      = 0x2051  // double[3], mm/s
HD_CURRENT_JOINT_ANGLES  = 0x2100  // double[3], rad (腰/肩/肘)
HD_CURRENT_GIMBAL_ANGLES = 0x2150  // double[3], rad (腕扭转/俯仰/笔杆旋转)
HD_CURRENT_FORCE         = 0x2700  // double[3], N (写入, 最大 3.3N)
HD_FORCE_OUTPUT          = 0x4000  // hdEnable() 参数
```

### 安全规则

- **力限幅**: 单轴不超 3.0N, 合力不超 3.3N
- **力变化率**: 避免突然跳变 (>1N/ms), 否则用户会被吓到
- **退出方式**: 必须 SIGINT/SIGTERM, 不能 SIGKILL
- **LD_LIBRARY_PATH**: 必须 `/tmp/patched_lib:/tmp/fakelibs:/usr/lib`

## 3. 已知坑 (必读)

### 3.1 Channel 号不固定
每次拔插 USB，`/dev/ttyACM` 编号可能变。部署脚本会自动检测并更新 config。
手动修复: `~/.3dsystems/config/Default Device.config` 的 `Channel=N`

### 3.2 0x303 错误 (设备通信故障)
原因: 上次进程非正常退出 (kill -9, timeout, core dump)
修复: serial break 重置
```python
import serial, time
s = serial.Serial('/dev/ttyACMx', 115200, timeout=0.5)
s.sendBreak(0.25)
time.sleep(1)
s.close()
```
或者拔插 USB。`deploy.sh` 内置了自动恢复。

### 3.3 补丁库说明
`libPhantomIOLib42.so` 闭源驱动有 3 个 bug，需打补丁:
- Patch 1 (0x3f2c2): CommunicationReadData 无数据时返回 0 而非 -28
- Patch 3 (0x32446): 跳过序列号校验
- Patch 4 (0x3b166): 绕过 RTAI 硬件定时器失败

补丁在 `/tmp/patched_lib/`，重启后丢失，`deploy.sh` 会重建。

### 3.4 hdStartScheduler 返回 0x103
正常现象。伺服线程用 clock_nanosleep 软定时，不需要硬件定时器。

## 4. 后台运行 (Agent 推荐方式)

```bash
# 启动
bash TouchUsage/scripts/run_background.sh

# 切换力效果 (0-8)
echo 5 > /tmp/force_mode

# 读取实时状态
cat /tmp/force_status
# 输出: mode=5 name=MAGNET
#        pos=10.2 -5.3 15.0
#        force=-0.82 0.43 -1.21 mag=1.53
#        btn=0 servo=1000Hz ticks=12345

# 停止
echo q > /tmp/force_mode

# 查看日志
cat /tmp/force_auto.log
```

## 5. ROS2 集成路径

```
[Touch 力反馈笔]
    ↓ 6-DOF 位置/关节角/按钮
[touch_bridge ROS 节点] ← 基于 force_background.c 改造
    ↓ 发布                    ← 订阅
    /touch/pose (PoseStamped)   /touch/force_cmd (Vector3)
    /touch/joints (JointState)
    /touch/buttons (Int32)
```
