/*
 * Touch 力反馈笔完整演示程序
 *
 * 功能:
 *   1. 初始化设备（应用补丁后可正常工作）
 *   2. 实时打印末端位置、关节角、万向节角、按钮状态
 *   3. 显示伺服频率
 *   4. 按白色按钮时施加弹簧力（朝向原点）
 *   5. 按灰色按钮时施加 Z 方向振动力
 *
 * 编译:
 *   gcc -o /tmp/touch_demo /tmp/20260329-cc-touch_demo.c -ldl -lm
 *
 * 运行:
 *   LD_LIBRARY_PATH=/tmp/patched_lib:/tmp/fakelibs:/usr/lib /tmp/touch_demo
 *
 * 补丁库位置:
 *   /tmp/patched_lib/libPhantomIOLib42.so (Patch1+3+4)
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <dlfcn.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <time.h>

/* ===== HD 类型定义 ===== */
typedef unsigned int   HHD;
typedef unsigned int   HDuint;
typedef int            HDint;
typedef unsigned short HDushort;
typedef double         HDdouble;
typedef unsigned long  HDSchedulerHandle;
typedef unsigned int   HDCallbackCode;
typedef const char*    HDstring;

/* 回调返回值（注意与直觉相反！） */
#define HD_CALLBACK_DONE     0   /* 结束回调 */
#define HD_CALLBACK_CONTINUE 1   /* 继续执行 */

/* 调度器优先级 */
#define HD_DEFAULT_SCHEDULER_PRIORITY ((HDushort)500)

/* 状态参数常量 */
#define HD_CURRENT_BUTTONS          0x2000
#define HD_CURRENT_INKWELL_SWITCH   0x2002
#define HD_CURRENT_POSITION         0x2050  /* 3 double: x,y,z (mm) */
#define HD_CURRENT_VELOCITY         0x2051  /* 3 double: vx,vy,vz (mm/s) */
#define HD_CURRENT_TRANSFORM        0x2052  /* 16 double: 4x4 矩阵 */
#define HD_CURRENT_JOINT_ANGLES     0x2100  /* 3 double: 关节角 (rad) */
#define HD_CURRENT_GIMBAL_ANGLES    0x2150  /* 3 double: 万向节角 (rad) */
#define HD_DEVICE_SERIAL_NUMBER     0x2504
#define HD_NOMINAL_MAX_FORCE        0x2603
#define HD_CURRENT_FORCE            0x2700  /* 3 double: fx,fy,fz (N) */

#define HD_INVALID_HANDLE   0xFFFFFFFF

/* ===== 函数指针类型 ===== */
typedef HHD (*hdInitDevice_t)(const char*);
typedef void (*hdDisableDevice_t)(HHD);
typedef void (*hdBeginFrame_t)(HHD);
typedef void (*hdEndFrame_t)(HHD);
typedef void (*hdGetDoublev_t)(unsigned int, double*);
typedef void (*hdGetIntegerv_t)(unsigned int, int*);
typedef HDstring (*hdGetString_t)(unsigned int);
typedef void (*hdSetDoublev_t)(unsigned int, const double*);
typedef struct { int errorCode; int internalCode; } HDErrorInfo;
typedef HDErrorInfo (*hdGetError_t)(void);
typedef HDstring (*hdGetErrorString_t)(int);
typedef HDSchedulerHandle (*hdScheduleAsynchronous_t)(
    HDCallbackCode (*cb)(void*), void*, HDushort);
typedef void (*hdStartScheduler_t)(void);
typedef void (*hdStopScheduler_t)(void);

/* ===== 全局函数指针 ===== */
static hdInitDevice_t    f_init;
static hdDisableDevice_t f_disable;
static hdBeginFrame_t    f_begin;
static hdEndFrame_t      f_end;
static hdGetDoublev_t    f_getd;
static hdGetIntegerv_t   f_geti;
static hdGetString_t     f_gets;
static hdSetDoublev_t    f_setd;
static hdGetError_t      f_err;
static hdGetErrorString_t f_errstr;
static hdScheduleAsynchronous_t f_schedA;
static hdStartScheduler_t f_start;
static hdStopScheduler_t  f_stop;

/* ===== 全局状态 ===== */
static volatile int g_running = 1;
static HHD g_hHD = HD_INVALID_HANDLE;

/* 设备状态（伺服线程写，主线程读） */
static volatile double g_pos[3];        /* 末端位置 mm */
static volatile double g_joints[3];     /* 关节角 rad */
static volatile double g_gimbal[3];     /* 万向节角 rad */
static volatile double g_vel[3];        /* 速度 mm/s */
static volatile int    g_buttons = 0;
static volatile int    g_inkwell = 0;
static volatile long   g_ticks = 0;     /* 伺服帧计数 */

/* 力指令（主线程写，伺服线程读） */
static volatile double g_force[3];

/* 力模式 */
#define FORCE_OFF     0   /* 零力 */
#define FORCE_SPRING  1   /* 弹簧力（朝向原点）*/
#define FORCE_VIBRATE 2   /* Z 方向振动力 */
static volatile int g_force_mode = FORCE_OFF;

/* ===== Servo 回调（约 1kHz）===== */
static HDCallbackCode servo_loop(void* data) {
    (void)data;

    /* 退出条件 */
    if (!g_running) return HD_CALLBACK_DONE;

    /* 开始帧 */
    f_begin(g_hHD);

    /* 读取末端位置 */
    double pos[3], vel[3], joints[3], gimbal[3];
    f_getd(HD_CURRENT_POSITION, pos);
    f_getd(HD_CURRENT_VELOCITY, vel);
    f_getd(HD_CURRENT_JOINT_ANGLES, joints);
    f_getd(HD_CURRENT_GIMBAL_ANGLES, gimbal);

    int buttons = 0, inkwell = 0;
    f_geti(HD_CURRENT_BUTTONS, &buttons);
    f_geti(HD_CURRENT_INKWELL_SWITCH, &inkwell);

    /* 存储到全局（volatile 写） */
    for (int i = 0; i < 3; i++) {
        g_pos[i]    = pos[i];
        g_vel[i]    = vel[i];
        g_joints[i] = joints[i];
        g_gimbal[i] = gimbal[i];
    }
    g_buttons = buttons;
    g_inkwell = inkwell;
    g_ticks++;

    /* 计算力输出 */
    double force[3] = {0.0, 0.0, 0.0};

    if (g_force_mode == FORCE_SPRING) {
        /* 弹簧力：F = -k * pos，将笔拉向工作空间中心 */
        /* k = 0.03 N/mm，最大约 3N（100mm 处） */
        double k = 0.03;
        force[0] = -k * pos[0];
        force[1] = -k * (pos[1] - 100.0);  /* Y 中心约 100mm */
        force[2] = -k * pos[2];
        /* 限幅 3N */
        for (int i = 0; i < 3; i++) {
            if (force[i] >  3.0) force[i] =  3.0;
            if (force[i] < -3.0) force[i] = -3.0;
        }
    } else if (g_force_mode == FORCE_VIBRATE) {
        /* Z 方向振动：2Hz, 1N 峰值 */
        double t = g_ticks * 0.001;  /* 秒（假设 1kHz）*/
        force[2] = 1.0 * sin(2.0 * M_PI * 2.0 * t);
    }

    /* 也读取用户设置的力（叠加） */
    force[0] += g_force[0];
    force[1] += g_force[1];
    force[2] += g_force[2];

    f_setd(HD_CURRENT_FORCE, force);

    /* 结束帧，提交力输出 */
    f_end(g_hHD);

    return HD_CALLBACK_CONTINUE;
}

/* Ctrl+C 信号处理 */
static void sig_handler(int s) { (void)s; g_running = 0; }

/* ===== 加载库 ===== */
static int load_libs(void) {
    /* 加载 ncurses stub（防止 libHD.so 找不到） */
    dlopen("/tmp/fakelibs/libncurses.so.5", RTLD_NOW | RTLD_GLOBAL);

    /* 加载 HD 库（会先找 LD_LIBRARY_PATH 里的 /tmp/patched_lib/libPhantomIOLib42.so）*/
    void* lib = dlopen("/usr/lib/libHD.so", RTLD_NOW | RTLD_GLOBAL);
    if (!lib) { fprintf(stderr, "dlopen libHD.so 失败: %s\n", dlerror()); return -1; }

/* 加载符号的宏 */
#define SYM(var, name) do { \
    var = dlsym(lib, #name); \
    if (!var) { fprintf(stderr, "找不到符号: %s\n", #name); return -1; } \
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
#undef SYM

    /* 可选符号 */
    f_gets = dlsym(lib, "hdGetString");
    return 0;
}

/* ===== 辅助：清空错误队列 ===== */
static void drain_errors(void) {
    HDErrorInfo e;
    while ((e = f_err()).errorCode != 0) {
        /* 静默丢弃初始化阶段的错误 */
    }
}

int main(void) {
    printf("=================================================\n");
    printf("  Touch 力反馈笔演示程序\n");
    printf("  设备序列号: 220-390-00866\n");
    printf("=================================================\n\n");

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    /* 加载库 */
    if (load_libs() != 0) return 1;

    /* 清空库加载期间的错误 */
    drain_errors();

    /* ---- 1. 初始化设备 ---- */
    printf("[1/3] 初始化设备...\n");
    g_hHD = f_init(NULL);
    if (g_hHD == HD_INVALID_HANDLE) {
        HDErrorInfo e = f_err();
        fprintf(stderr, "初始化失败: 0x%x / %d\n", e.errorCode, e.internalCode);
        return 1;
    }
    printf("      成功! hHD=%u\n", g_hHD);
    if (f_gets) {
        HDstring sn = f_gets(HD_DEVICE_SERIAL_NUMBER);
        if (sn) printf("      固件序列号: %s\n", sn);
    }

    /* ---- 2. 注册伺服回调 ---- */
    printf("[2/3] 注册伺服回调...\n");
    HDSchedulerHandle h = f_schedA(servo_loop, NULL, HD_DEFAULT_SCHEDULER_PRIORITY);
    drain_errors();  /* 清掉可能的非致命错误 */
    printf("      句柄: %lu\n", h);

    /* ---- 3. 启动调度器 ---- */
    printf("[3/3] 启动调度器...\n");
    f_start();
    drain_errors();  /* 0x103 错误可忽略，伺服线程会运行 */
    printf("      调度器已启动!\n\n");

    /* 等待伺服首帧 */
    for (int i = 0; i < 200 && g_ticks == 0; i++) usleep(5000);
    if (g_ticks == 0) {
        fprintf(stderr, "警告: 伺服回调未启动!\n");
        return 1;
    }
    printf("伺服循环运行中!\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("操作说明:\n");
    printf("  • 移动笔杆 → 实时位置更新\n");
    printf("  • 白色按钮(bit0=1) → 激活弹簧力（朝向中心）\n");
    printf("  • 灰色按钮(bit1=1) → 激活振动力（2Hz Z方向）\n");
    printf("  • Ctrl+C   → 退出\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");

    /* 表头 */
    printf("%-6s | %-7s %-7s %-7s | %-6s %-6s %-6s | %-6s %-6s %-6s | BTN | SR(Hz)\n",
           "T(s)", "X", "Y", "Z", "Q1°", "Q2°", "Q3°", "G1°", "G2°", "G3°");
    printf("%.90s\n",
        "------------------------------------------------------------------------------------------");

    /* ---- 主循环 ---- */
    long prev_ticks = 0;
    int frame = 0;
    double t = 0.0;

    while (g_running) {
        usleep(50000);  /* 50ms 刷新 */
        t += 0.05;
        frame++;

        /* 读取当前状态 */
        int btns = g_buttons;
        long cur_ticks = g_ticks;

        /* 根据按钮设置力模式 */
        if (btns & 0x01) {
            g_force_mode = FORCE_SPRING;
        } else if (btns & 0x02) {
            g_force_mode = FORCE_VIBRATE;
        } else {
            g_force_mode = FORCE_OFF;
        }

        /* 计算实际伺服频率 */
        long sr = (cur_ticks - prev_ticks) * 20;  /* 50ms * 20 = 1s */
        prev_ticks = cur_ticks;

        /* 每帧打印一次 */
        printf("%-6.1f | %-7.1f %-7.1f %-7.1f | %-6.1f %-6.1f %-6.1f | %-6.1f %-6.1f %-6.1f |",
               t,
               (double)g_pos[0], (double)g_pos[1], (double)g_pos[2],
               (double)g_joints[0] * 180.0/M_PI,
               (double)g_joints[1] * 180.0/M_PI,
               (double)g_joints[2] * 180.0/M_PI,
               (double)g_gimbal[0] * 180.0/M_PI,
               (double)g_gimbal[1] * 180.0/M_PI,
               (double)g_gimbal[2] * 180.0/M_PI);

        /* 按钮状态 + 力模式指示 */
        const char* mode_str = (g_force_mode == FORCE_SPRING)  ? " [弹簧]" :
                               (g_force_mode == FORCE_VIBRATE) ? " [振动]" : "";
        printf(" %d%s | %ld\n", btns, mode_str, sr);
        fflush(stdout);
    }

    /* ---- 停止 ---- */
    printf("\n停止...\n");
    g_running = 0;
    g_force[0] = g_force[1] = g_force[2] = 0.0;
    g_force_mode = FORCE_OFF;

    f_stop();
    f_disable(g_hHD);

    printf("完成! 总伺服帧: %ld (平均 %.0f Hz)\n",
           g_ticks, g_ticks / t);
    return 0;
}
