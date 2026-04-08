/*
 * Touch 力反馈笔完整功能测试 v4
 * 基于 dlopen 方式加载库, 避免静态链接带来的初始化顺序问题
 * 测试: 位置读取 + 力反馈输出
 *
 * 编译: gcc -o test_haptic_v4 20260329-cc-test_haptic_v4.c -ldl -lm -lpthread
 * 运行: LD_LIBRARY_PATH=/tmp/patched_lib:/tmp/fakelibs:/usr/lib ./test_haptic_v4
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <dlfcn.h>
#include <math.h>
#include <signal.h>
#include <pthread.h>

/* ===== HD 类型和常量 ===== */
typedef unsigned int   HHD;
typedef unsigned int   HDuint;
typedef int            HDint;
typedef unsigned long  HDulong;
typedef unsigned short HDushort;
typedef double         HDdouble;
typedef float          HDfloat;
typedef unsigned int   HDboolean;
typedef unsigned long  HDSchedulerHandle;
typedef unsigned int   HDCallbackCode;
typedef const char*    HDstring;

/* 回调返回值: 0=完成(取消调度), 1=继续 */
#define HD_CALLBACK_DONE     0
#define HD_CALLBACK_CONTINUE 1

/* 等待代码 */
#define HD_WAIT_CHECK_STATUS 0
#define HD_WAIT_INFINITE     1

/* 调度器优先级 */
#define HD_DEFAULT_SCHEDULER_PRIORITY ((HDushort)500)

/* 状态参数枚举 */
#define HD_CURRENT_BUTTONS          0x2000
#define HD_CURRENT_SAFETY_SWITCH    0x2001
#define HD_CURRENT_INKWELL_SWITCH   0x2002
#define HD_CURRENT_ENCODER_VALUES   0x2010
#define HD_CURRENT_POSITION         0x2050  /* 3 double: X,Y,Z (mm) */
#define HD_CURRENT_VELOCITY         0x2051  /* 3 double (mm/s) */
#define HD_CURRENT_TRANSFORM        0x2052  /* 16 double (4x4 矩阵) */
#define HD_CURRENT_JOINT_ANGLES     0x2100  /* 3 double (rad): 肩/肘/腕 */
#define HD_CURRENT_GIMBAL_ANGLES    0x2150  /* 3 double (rad): 万向节 */
#define HD_DEVICE_SERIAL_NUMBER     0x2504
#define HD_NOMINAL_MAX_FORCE        0x2603
#define HD_UPDATE_RATE              0x2600
#define HD_CURRENT_FORCE            0x2700  /* 3 double: Fx,Fy,Fz (N) */

/* 错误码 */
#define HD_SUCCESS          0x0000
#define HD_COMM_ERROR       0x0302
#define HD_INVALID_HANDLE   0xFFFFFFFFu

/* ===== 函数指针类型 ===== */
typedef HHD (*hdInitDevice_t)(const char*);
typedef void (*hdDisableDevice_t)(HHD);
typedef void (*hdBeginFrame_t)(HHD);
typedef void (*hdEndFrame_t)(HHD);
typedef void (*hdGetDoublev_t)(unsigned int, double*);
typedef void (*hdGetIntegerv_t)(unsigned int, int*);
typedef HDstring (*hdGetString_t)(unsigned int);
typedef void (*hdSetDoublev_t)(unsigned int, const double*);
typedef struct { int errorCode; int internalErrorCode; } HDErrorInfo;
typedef HDErrorInfo (*hdGetError_t)(void);
typedef HDstring (*hdGetErrorString_t)(int);
typedef HDSchedulerHandle (*hdScheduleAsynchronous_t)(
    HDCallbackCode (*)(void*), void*, HDushort);
typedef void (*hdScheduleSynchronous_t)(
    HDCallbackCode (*)(void*), void*, HDushort);
typedef void (*hdStartScheduler_t)(void);
typedef void (*hdStopScheduler_t)(void);
typedef HDboolean (*hdWaitForCompletion_t)(HDSchedulerHandle, unsigned int);

/* ===== 全局函数指针 ===== */
static hdInitDevice_t           f_init;
static hdDisableDevice_t        f_disable;
static hdBeginFrame_t           f_begin;
static hdEndFrame_t             f_end;
static hdGetDoublev_t           f_getd;
static hdGetIntegerv_t          f_geti;
static hdGetString_t            f_gets;
static hdSetDoublev_t           f_setd;
static hdGetError_t             f_err;
static hdGetErrorString_t       f_errstr;
static hdScheduleAsynchronous_t f_schedA;
static hdStartScheduler_t       f_start;
static hdStopScheduler_t        f_stop;
static hdWaitForCompletion_t    f_wait;

/* ===== 全局状态 ===== */
static volatile int g_running = 1;
static HHD g_hHD = HD_INVALID_HANDLE;

/* 设备状态 (servo 线程写) */
static volatile double g_pos[3];        /* 末端位置 mm */
static volatile double g_joints[3];     /* 关节角 rad */
static volatile double g_gimbal[3];     /* 万向节角 rad */
static volatile double g_vel[3];        /* 速度 mm/s */
static volatile int    g_buttons;
static volatile int    g_inkwell;
static volatile int    g_ticks;
static volatile double g_transform[16]; /* 4x4 末端位姿矩阵 */

/* 力指令 (主线程写, servo 读) */
static volatile double g_force[3];

/* ===== Servo 回调 (~1kHz) ===== */
static HDCallbackCode servo_loop(void* data) {
    (void)data;
    if (!g_running) return HD_CALLBACK_DONE;

    /* 开始帧 */
    f_begin(g_hHD);

    /* 读取设备状态 */
    f_getd(HD_CURRENT_POSITION,     (double*)g_pos);
    f_getd(HD_CURRENT_VELOCITY,     (double*)g_vel);
    f_getd(HD_CURRENT_JOINT_ANGLES, (double*)g_joints);
    f_getd(HD_CURRENT_GIMBAL_ANGLES,(double*)g_gimbal);
    f_getd(HD_CURRENT_TRANSFORM,    (double*)g_transform);
    f_geti(HD_CURRENT_BUTTONS,      (int*)&g_buttons);
    f_geti(HD_CURRENT_INKWELL_SWITCH, (int*)&g_inkwell);

    /* 施加力 */
    double force[3] = { (double)g_force[0], (double)g_force[1], (double)g_force[2] };
    f_setd(HD_CURRENT_FORCE, force);

    g_ticks++;

    /* 结束帧 (提交力) */
    f_end(g_hHD);

    return HD_CALLBACK_CONTINUE;
}

/* Ctrl+C 处理 */
static void sig_handler(int s) { (void)s; g_running = 0; }

/* ===== 加载库 ===== */
static int load_libs(void) {
    /* 先加载 ncurses 依赖 */
    dlopen("/tmp/fakelibs/libncurses.so.5", RTLD_NOW | RTLD_GLOBAL);

    /* 加载 HD 主库 */
    void* lib = dlopen("/usr/lib/libHD.so", RTLD_NOW | RTLD_GLOBAL);
    if (!lib) {
        fprintf(stderr, "dlopen libHD.so 失败: %s\n", dlerror());
        return -1;
    }

/* 加载符号的宏 */
#define SYM(var, name) do { \
    (var) = dlsym(lib, #name); \
    if (!(var)) { fprintf(stderr, "dlsym %s 失败\n", #name); return -1; } \
} while(0)

    SYM(f_init,   hdInitDevice);
    SYM(f_disable,hdDisableDevice);
    SYM(f_begin,  hdBeginFrame);
    SYM(f_end,    hdEndFrame);
    SYM(f_getd,   hdGetDoublev);
    SYM(f_geti,   hdGetIntegerv);
    SYM(f_setd,   hdSetDoublev);
    SYM(f_err,    hdGetError);
    SYM(f_errstr, hdGetErrorString);
    SYM(f_schedA, hdScheduleAsynchronous);
    SYM(f_start,  hdStartScheduler);
    SYM(f_stop,   hdStopScheduler);
    SYM(f_wait,   hdWaitForCompletion);
#undef SYM

    f_gets = (hdGetString_t)dlsym(lib, "hdGetString");  /* 可选 */
    return 0;
}

/* ===== 错误检查并清空队列 ===== */
static int drain_errors(const char* ctx, int fatal) {
    HDErrorInfo err;
    int had = 0;
    while (1) {
        err = f_err();
        if (err.errorCode == HD_SUCCESS) break;
        fprintf(stderr, "[%s] 错误 0x%04x / %d: %s\n",
                ctx, err.errorCode, err.internalErrorCode,
                f_errstr ? f_errstr(err.errorCode) : "");
        had = 1;
        if (!fatal) break;  /* 非致命: 只打印第一个 */
    }
    return had;
}

int main(void) {
    printf("===== Touch 力反馈笔测试 v4 =====\n");
    printf("序列号: 220-390-00866\n\n");

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    /* ---- 加载库 ---- */
    printf("[0] 加载 HD 库...\n");
    if (load_libs() != 0) return 1;
    printf("    库已加载\n");

    /* 清空可能存在的旧错误 */
    drain_errors("startup", 1);

    /* ---- 1. 初始化设备 ---- */
    printf("[1] 初始化设备...\n");
    g_hHD = f_init(NULL);
    drain_errors("hdInitDevice", 1);

    if (g_hHD == HD_INVALID_HANDLE) {
        fprintf(stderr, "初始化失败!\n");
        return 1;
    }
    printf("    成功! hHD=%u\n", g_hHD);

    if (f_gets) {
        HDstring sn = f_gets(HD_DEVICE_SERIAL_NUMBER);
        if (sn) printf("    固件序列号: '%s'\n", sn);
    }
    {
        double max_force = 0;
        f_getd(HD_NOMINAL_MAX_FORCE, &max_force);
        printf("    额定最大力: %.2f N\n", max_force);
    }

    /* ---- 2. 注册 servo 回调 ---- */
    printf("[2] 注册 servo 回调...\n");
    HDSchedulerHandle hSched = f_schedA(servo_loop, NULL, HD_DEFAULT_SCHEDULER_PRIORITY);
    drain_errors("hdScheduleAsync", 1);
    printf("    调度句柄: %lu\n", hSched);

    /* ---- 3. 启动调度器 ---- */
    printf("[3] 启动调度器...\n");
    f_start();
    int start_err = drain_errors("hdStartScheduler", 1);
    if (start_err) {
        printf("    注意: 启动器有错误但继续...\n");
    }
    printf("    调度器命令已发送\n\n");

    /* 等待首帧 (最多 1秒) */
    for (int i = 0; i < 200 && g_ticks == 0; i++) usleep(5000);

    if (g_ticks == 0) {
        printf("警告: servo 回调还未运行! 检查调度器状态...\n");
        drain_errors("after_wait", 1);
    } else {
        printf("首帧已收到 (servo 正在运行, ticks=%d)\n\n", g_ticks);
    }

    /* ---- 4. 主循环 5秒 ---- */
    printf("%-7s | %-9s %-9s %-9s | %-8s %-8s %-8s | Btn  Force_Z(N)\n",
           "T(s)", "X(mm)", "Y(mm)", "Z(mm)",
           "Q1(°)", "Q2(°)", "Q3(°)");
    printf("%.90s\n", "-------------------------------"
           "-------------------------------"
           "-------------------------------");

    double t = 0.0;
    int prev_ticks = 0;

    for (int step = 0; step < 200 && g_running; step++) {
        usleep(25000);  /* 25ms */
        t += 0.025;

        /* 每 0.5s 打印 */
        if (step % 20 == 0) {
            int cur = g_ticks;
            int sr = (cur - prev_ticks) * 20;
            prev_ticks = cur;

            printf("%-7.2f | %-9.2f %-9.2f %-9.2f | %-8.2f %-8.2f %-8.2f | %d    %.2f\n",
                   t,
                   (double)g_pos[0], (double)g_pos[1], (double)g_pos[2],
                   (double)g_joints[0] * 180.0/M_PI,
                   (double)g_joints[1] * 180.0/M_PI,
                   (double)g_joints[2] * 180.0/M_PI,
                   g_buttons,
                   (double)g_force[2]);

            if (sr > 0 && step > 0) printf("  [servo rate ~%d Hz]\n", sr);
        }

        /* 第2~3秒: Z 方向正弦力 (让用户感受力反馈) */
        if (t >= 2.0 && t < 3.0) {
            g_force[0] = 0.0;
            g_force[1] = 0.0;
            g_force[2] = 0.5 * sin(2.0 * M_PI * 2.0 * t);  /* 2Hz, 0.5N */
        } else {
            g_force[0] = g_force[1] = g_force[2] = 0.0;
        }
    }

    /* ---- 5. 清理 ---- */
    printf("\n[5] 停止...\n");
    g_running = 0;
    g_force[0] = g_force[1] = g_force[2] = 0.0;

    f_stop();
    f_disable(g_hHD);

    printf("完成! 总 servo 帧: %d\n", g_ticks);
    printf("末端位置: X=%.2f Y=%.2f Z=%.2f mm\n",
           (double)g_pos[0], (double)g_pos[1], (double)g_pos[2]);
    return 0;
}
