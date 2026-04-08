/*
 * Touch 力反馈笔完整功能测试
 * 测试内容:
 *   1. hdInitDevice 初始化设备
 *   2. 启动调度器 (servo loop)
 *   3. 读取末端位置 (XYZ)
 *   4. 读取关节角度
 *   5. 读取按钮状态
 *   6. 施加测试力
 *
 * 编译: gcc -o test_haptic_full test_haptic_full.c -ldl
 * 运行: LD_LIBRARY_PATH=/tmp/patched_lib:/tmp/fakelibs:/usr/lib ./test_haptic_full
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <dlfcn.h>
#include <math.h>
#include <signal.h>

/* ===== HD 类型定义 ===== */
typedef unsigned int  HHD;          /* 设备句柄 */
typedef unsigned int  HDuint;
typedef int           HDint;
typedef unsigned long HDulong;
typedef double        HDdouble;
typedef float         HDfloat;
typedef unsigned int  HDboolean;
typedef unsigned long HDSchedulerHandle;

/* HD 状态参数常量 */
#define HD_CURRENT_BUTTONS         0x2000
#define HD_CURRENT_SAFETY_SWITCH   0x2001
#define HD_CURRENT_INKWELL_SWITCH  0x2002
#define HD_CURRENT_ENCODER_VALUES  0x2010
#define HD_CURRENT_POSITION        0x2050  /* 3 HDdouble: x,y,z (mm) */
#define HD_CURRENT_VELOCITY        0x2051  /* 3 HDdouble: vx,vy,vz (mm/s) */
#define HD_CURRENT_TRANSFORM       0x2052  /* 16 HDdouble: 4x4 변환행렬 */
#define HD_CURRENT_JOINT_ANGLES    0x2100  /* 3 HDdouble: 关节角 (rad) */
#define HD_CURRENT_GIMBAL_ANGLES   0x2150  /* 3 HDdouble: 万向节角 (rad) */
#define HD_DEVICE_SERIAL_NUMBER    0x2504
#define HD_UPDATE_RATE             0x2600
#define HD_NOMINAL_MAX_FORCE       0x2603
#define HD_CURRENT_FORCE           0x2700  /* 3 HDdouble: fx,fy,fz (N) */
#define HD_CURRENT_TORQUE          0x2701  /* 3 HDdouble */

#define HD_SUCCESS                 0x0000
#define HD_COMM_ERROR              0x0302
#define HD_INVALID_HANDLE          0xFFFFFFFF

/* 调度器回调返回值 */
#define HD_CALLBACK_CONTINUE       0
#define HD_CALLBACK_DONE           1

/* 调度器优先级 */
#define HD_DEFAULT_SCHEDULER_PRIORITY 500

/* ===== 全局函数指针 ===== */
static void* g_libhd = NULL;

typedef HHD (*hdInitDevice_t)(const char*);
typedef void (*hdDisableDevice_t)(HHD);
typedef void (*hdMakeCurrentDevice_t)(HHD);
typedef void (*hdBeginFrame_t)(HHD);
typedef void (*hdEndFrame_t)(HHD);
typedef void (*hdGetDoublev_t)(unsigned int, double*);
typedef void (*hdGetIntegerv_t)(unsigned int, int*);
typedef void (*hdGetStringv_t)(unsigned int, char*);
typedef void (*hdSetDoublev_t)(unsigned int, const double*);
typedef struct { int errorCode; int internalCode; } HDErrorInfo;
typedef HDErrorInfo (*hdGetError_t)(void);
typedef const char* (*hdGetErrorString_t)(int);
typedef HDSchedulerHandle (*hdScheduleAsynchronous_t)(unsigned int(*)(void*), void*, unsigned int);
typedef HDSchedulerHandle (*hdScheduleSynchronous_t)(unsigned int(*)(void*), void*, unsigned int);
typedef void (*hdStartScheduler_t)(void);
typedef void (*hdStopScheduler_t)(void);
typedef int (*hdWaitForCompletion_t)(HDSchedulerHandle, int);

static hdInitDevice_t          f_hdInitDevice = NULL;
static hdDisableDevice_t       f_hdDisableDevice = NULL;
static hdMakeCurrentDevice_t   f_hdMakeCurrentDevice = NULL;
static hdBeginFrame_t          f_hdBeginFrame = NULL;
static hdEndFrame_t            f_hdEndFrame = NULL;
static hdGetDoublev_t          f_hdGetDoublev = NULL;
static hdGetIntegerv_t         f_hdGetIntegerv = NULL;
static hdGetStringv_t          f_hdGetString = NULL;
static hdSetDoublev_t          f_hdSetDoublev = NULL;
static hdGetError_t            f_hdGetError = NULL;
static hdGetErrorString_t      f_hdGetErrorString = NULL;
static hdScheduleAsynchronous_t f_hdScheduleAsynchronous = NULL;
static hdScheduleSynchronous_t  f_hdScheduleSynchronous = NULL;
static hdStartScheduler_t      f_hdStartScheduler = NULL;
static hdStopScheduler_t       f_hdStopScheduler = NULL;
static hdWaitForCompletion_t   f_hdWaitForCompletion = NULL;

/* ===== 全局设备状态 ===== */
static volatile int g_running = 1;
static HHD g_hHD = HD_INVALID_HANDLE;

/* 共享数据结构 (servo loop 写入, 主线程读取) */
typedef struct {
    double position[3];      /* 末端位置 (mm): x,y,z */
    double joint_angles[3];  /* 关节角 (rad): q1,q2,q3 */
    double gimbal_angles[3]; /* 万向节角 (rad): q4,q5,q6 */
    double velocity[3];      /* 末端速度 (mm/s) */
    int    buttons;          /* 按钮状态位 */
    int    inkwell;          /* 墨水架开关 */
    double transform[16];    /* 4x4 变换矩阵 */
    double force[3];         /* 当前施加的力 (N) */
    int    valid;            /* 数据是否有效 */
} DeviceState;

static volatile DeviceState g_state;
static volatile double g_cmd_force[3] = {0.0, 0.0, 0.0};  /* 力指令 (主线程写) */

/* ===== Servo Loop 回调 ===== */
/* 在独立的伺服线程中执行，频率约 1kHz */
static unsigned int servo_callback(void* user_data) {
    if (!g_running) return HD_CALLBACK_DONE;

    /* 开始帧 - 必须在读写设备状态前调用 */
    f_hdBeginFrame(g_hHD);

    /* 读取末端位置 */
    f_hdGetDoublev(HD_CURRENT_POSITION, (double*)g_state.position);

    /* 读取末端速度 */
    f_hdGetDoublev(HD_CURRENT_VELOCITY, (double*)g_state.velocity);

    /* 读取关节角 (肩/肘/腕) */
    f_hdGetDoublev(HD_CURRENT_JOINT_ANGLES, (double*)g_state.joint_angles);

    /* 读取万向节角 (roll/pitch/yaw) */
    f_hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, (double*)g_state.gimbal_angles);

    /* 读取按钮状态 (bit0=白按钮, bit1=灰按钮) */
    f_hdGetIntegerv(HD_CURRENT_BUTTONS, (int*)&g_state.buttons);

    /* 读取墨水架检测开关 */
    f_hdGetIntegerv(HD_CURRENT_INKWELL_SWITCH, (int*)&g_state.inkwell);

    /* 读取变换矩阵 */
    f_hdGetDoublev(HD_CURRENT_TRANSFORM, (double*)g_state.transform);

    /* 施加力指令 */
    double force[3];
    force[0] = g_cmd_force[0];
    force[1] = g_cmd_force[1];
    force[2] = g_cmd_force[2];
    f_hdSetDoublev(HD_CURRENT_FORCE, force);

    /* 结束帧 - 提交力输出到设备 */
    f_hdEndFrame(g_hHD);

    g_state.valid = 1;
    return HD_CALLBACK_CONTINUE;
}

/* Ctrl+C 处理 */
static void signal_handler(int sig) {
    (void)sig;
    g_running = 0;
}

/* ===== 加载 HD 库 ===== */
static int load_hd_lib(void) {
    /* 先加载依赖 */
    dlopen("/tmp/fakelibs/libncurses.so.5", RTLD_NOW | RTLD_GLOBAL);

    g_libhd = dlopen("/usr/lib/libHD.so", RTLD_NOW | RTLD_GLOBAL);
    if (!g_libhd) {
        fprintf(stderr, "dlopen libHD.so 失败: %s\n", dlerror());
        return -1;
    }

/* 加载函数指针的宏 */
#define LOAD(name) do { \
    f_##name = dlsym(g_libhd, #name); \
    if (!f_##name) { fprintf(stderr, "dlsym %s 失败\n", #name); return -1; } \
} while(0)

    LOAD(hdInitDevice);
    LOAD(hdDisableDevice);
    LOAD(hdMakeCurrentDevice);
    LOAD(hdBeginFrame);
    LOAD(hdEndFrame);
    LOAD(hdGetDoublev);
    LOAD(hdGetIntegerv);
    LOAD(hdGetError);
    LOAD(hdGetErrorString);
    LOAD(hdSetDoublev);
    LOAD(hdScheduleAsynchronous);
    LOAD(hdStartScheduler);
    LOAD(hdStopScheduler);
    LOAD(hdWaitForCompletion);
#undef LOAD

    /* hdGetString 是可选的 */
    f_hdGetString = dlsym(g_libhd, "hdGetString");

    return 0;
}

int main(void) {
    printf("===== Touch 力反馈笔测试 =====\n");
    printf("设备序列号: 220-390-00866\n\n");

    /* 信号处理 */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* 加载库 */
    if (load_hd_lib() != 0) return 1;

    /* 初始化设备 */
    printf("正在初始化设备...\n");
    g_hHD = f_hdInitDevice(NULL);

    if (g_hHD == HD_INVALID_HANDLE) {
        HDErrorInfo err = f_hdGetError();
        fprintf(stderr, "初始化失败! errorCode=0x%x internalCode=%d\n",
                err.errorCode, err.internalCode);
        if (f_hdGetErrorString)
            fprintf(stderr, "描述: %s\n", f_hdGetErrorString(err.errorCode));
        return 1;
    }

    printf("设备初始化成功! hHD=%u\n", g_hHD);

    /* 打印设备信息 */
    if (f_hdGetString) {
        char buf[256] = {0};
        f_hdGetString(HD_DEVICE_SERIAL_NUMBER, buf);
        printf("设备序列号 (固件): %s\n", buf);
    }

    {
        double update_rate = 0;
        f_hdGetDoublev(HD_UPDATE_RATE, &update_rate);
        printf("伺服更新率: %.1f Hz\n", update_rate);

        double max_force = 0;
        f_hdGetDoublev(HD_NOMINAL_MAX_FORCE, &max_force);
        printf("最大额定力: %.2f N\n", max_force);
    }

    /* 注册异步 servo 回调 (1kHz 伺服环) */
    printf("\n启动伺服调度器...\n");
    HDSchedulerHandle hSchedule = f_hdScheduleAsynchronous(
        servo_callback, NULL, HD_DEFAULT_SCHEDULER_PRIORITY);

    /* 启动调度器 */
    f_hdStartScheduler();

    {
        HDErrorInfo err = f_hdGetError();
        if (err.errorCode != HD_SUCCESS) {
            fprintf(stderr, "启动调度器失败: 0x%x / %d\n",
                    err.errorCode, err.internalCode);
            return 1;
        }
    }
    printf("调度器已启动!\n\n");

    /* ===== 主循环: 打印位置数据 ===== */
    printf("%-8s %-10s %-10s %-10s | %-8s %-8s %-8s | BTN\n",
           "Time(s)", "X(mm)", "Y(mm)", "Z(mm)",
           "Q1(deg)", "Q2(deg)", "Q3(deg)");
    printf("%s\n", "------------------------------------------------------------------------");

    int tick = 0;
    double test_force_phase = 0.0;   /* 用于生成振荡测试力 */

    while (g_running && tick < 200) {  /* 运行约 5 秒 */
        usleep(25000);  /* 25ms 间隔 */

        if (!g_state.valid) {
            tick++;
            continue;
        }

        /* 每 0.5s 打印一次 */
        if (tick % 20 == 0) {
            printf("%-8.2f %-10.2f %-10.2f %-10.2f | %-8.2f %-8.2f %-8.2f | %d\n",
                   tick * 0.025,
                   g_state.position[0],
                   g_state.position[1],
                   g_state.position[2],
                   g_state.joint_angles[0] * 180.0 / M_PI,
                   g_state.joint_angles[1] * 180.0 / M_PI,
                   g_state.joint_angles[2] * 180.0 / M_PI,
                   g_state.buttons);
        }

        /* 测试力: 施加一个正弦波 Z 方向力 (2Hz, 0.3N 峰值) */
        /* 注意: 这会让笔杆产生轻微阻尼感 */
        test_force_phase += 0.025 * 2.0 * M_PI * 2.0;
        if (tick >= 40 && tick < 120) {
            /* 从第1秒到第3秒施加测试力 */
            g_cmd_force[0] = 0.0;
            g_cmd_force[1] = 0.0;
            g_cmd_force[2] = 0.3 * sin(test_force_phase);  /* 0.3N 正弦力 */
        } else {
            /* 其他时间零力 */
            g_cmd_force[0] = 0.0;
            g_cmd_force[1] = 0.0;
            g_cmd_force[2] = 0.0;
        }

        tick++;
    }

    /* ===== 停止 ===== */
    printf("\n停止调度器...\n");
    g_running = 0;

    /* 清除力指令 */
    g_cmd_force[0] = g_cmd_force[1] = g_cmd_force[2] = 0.0;

    f_hdStopScheduler();
    f_hdDisableDevice(g_hHD);

    printf("测试完成!\n");
    printf("最终位置: X=%.2f Y=%.2f Z=%.2f mm\n",
           g_state.position[0],
           g_state.position[1],
           g_state.position[2]);
    printf("关节角: Q1=%.2f Q2=%.2f Q3=%.2f deg\n",
           g_state.joint_angles[0] * 180.0 / M_PI,
           g_state.joint_angles[1] * 180.0 / M_PI,
           g_state.joint_angles[2] * 180.0 / M_PI);

    return 0;
}
