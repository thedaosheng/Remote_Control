/**
 * Touch 力反馈笔 ROS 节点
 *
 * 功能:
 *   1. 读取 Touch 力反馈笔的末端位置和关节角
 *   2. 发布位置到 /touch/state (用于 UR 遥操作)
 *   3. 订阅力指令 /touch/force 并施加到设备
 *   4. 发布按钮状态 /touch/buttons
 *
 * 话题:
 *   发布:
 *     /touch/state    (geometry_msgs/PoseStamped): 末端位姿
 *     /touch/joints   (sensor_msgs/JointState): 关节角
 *     /touch/buttons  (std_msgs/Int32): 按钮状态位掩码
 *     /touch/velocity (geometry_msgs/Vector3Stamped): 末端速度
 *   订阅:
 *     /touch/force    (geometry_msgs/WrenchStamped): 期望力/力矩
 *
 * 编译 (在ROS工作空间中):
 *   catkin_make 或 colcon build
 *
 * 手动编译测试:
 *   g++ -o touch_haptic_node touch_haptic_node.cpp \
 *       -I/home/rhz/haptics_install/local/include \
 *       -L/home/rhz/haptics_install/local/lib \
 *       -lHD -lm -lpthread -ldl
 *
 * 运行:
 *   export LD_LIBRARY_PATH=/tmp/patched_lib:/tmp/fakelibs:/usr/lib
 *   rosrun your_package touch_haptic_node
 *
 * 序列号: 220-390-00866
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <dlfcn.h>
#include <pthread.h>

/* ===== HD API 类型 (通过 dlopen 加载以避免静态链接问题) ===== */
typedef unsigned int   HHD;
typedef unsigned int   HDuint;
typedef int            HDint;
typedef unsigned long  HDulong;
typedef unsigned short HDushort;
typedef double         HDdouble;
typedef unsigned int   HDboolean;
typedef unsigned long  HDSchedulerHandle;
typedef unsigned int   HDCallbackCode;
typedef const char*    HDstring;

#define HD_CALLBACK_DONE     0
#define HD_CALLBACK_CONTINUE 1

#define HD_DEFAULT_SCHEDULER_PRIORITY ((HDushort)500)

/* 状态参数 */
#define HD_CURRENT_BUTTONS          0x2000
#define HD_CURRENT_INKWELL_SWITCH   0x2002
#define HD_CURRENT_POSITION         0x2050  /* 3 double: X,Y,Z mm */
#define HD_CURRENT_VELOCITY         0x2051  /* 3 double: mm/s */
#define HD_CURRENT_TRANSFORM        0x2052  /* 16 double: 4x4 */
#define HD_CURRENT_JOINT_ANGLES     0x2100  /* 3 double: rad */
#define HD_CURRENT_GIMBAL_ANGLES    0x2150  /* 3 double: rad */
#define HD_NOMINAL_MAX_FORCE        0x2603
#define HD_CURRENT_FORCE            0x2700  /* 3 double: N */
#define HD_CURRENT_TORQUE           0x2701  /* 3 double */
#define HD_CURRENT_JOINT_TORQUE     0x2703  /* 3 double */

#define HD_SUCCESS          0x0000
#define HD_INVALID_HANDLE   0xFFFFFFFFu

/* ===== 函数指针 ===== */
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
typedef void (*hdStartScheduler_t)(void);
typedef void (*hdStopScheduler_t)(void);

/* 全局函数指针 */
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

/* ===== 设备状态 ===== */
static volatile int g_running = 1;
static HHD g_hHD = HD_INVALID_HANDLE;

/* 从设备读取的状态 (Servo 线程写, 主线程读) */
typedef struct {
    double position[3];       /* 末端位置 mm (设备坐标系) */
    double velocity[3];       /* 末端速度 mm/s */
    double joint_angles[3];   /* 关节角 rad: 肩/肘/腕 */
    double gimbal_angles[3];  /* 万向节角 rad: roll/pitch/yaw */
    double transform[16];     /* 末端变换矩阵 4x4 (列主序) */
    int    buttons;           /* 按钮位掩码: bit0=白色,bit1=灰色 */
    int    inkwell;           /* 墨水架开关 */
    int    ticks;             /* Servo 计数 */
    double max_force;         /* 额定最大力 N */
} TouchState;

static volatile TouchState g_touch;

/* 力指令 (主线程写, Servo 线程读) */
typedef struct {
    double force[3];   /* 期望力 N (设备坐标系) */
    double torque[3];  /* 期望力矩 Nm */
} TouchForceCmd;

static volatile TouchForceCmd g_force_cmd;
static pthread_mutex_t g_force_mutex = PTHREAD_MUTEX_INITIALIZER;

/* ===== 坐标系变换 =====
 * Touch 设备坐标系:
 *   X: 右
 *   Y: 上
 *   Z: 朝向用户
 * UR 机器人坐标系: 根据安装方式确定
 * 工作空间: 约 ±150mm x ±150mm x ±100mm
 */
#define TOUCH_TO_ROBOT_SCALE 0.001   /* mm → m */

/* ===== Servo 回调 (~1kHz) ===== */
static HDCallbackCode servo_loop(void* data) {
    (void)data;
    if (!g_running) return HD_CALLBACK_DONE;

    /* 开始帧 */
    f_begin(g_hHD);

    /* 读取状态 */
    f_getd(HD_CURRENT_POSITION,      (double*)g_touch.position);
    f_getd(HD_CURRENT_VELOCITY,      (double*)g_touch.velocity);
    f_getd(HD_CURRENT_JOINT_ANGLES,  (double*)g_touch.joint_angles);
    f_getd(HD_CURRENT_GIMBAL_ANGLES, (double*)g_touch.gimbal_angles);
    f_getd(HD_CURRENT_TRANSFORM,     (double*)g_touch.transform);
    f_geti(HD_CURRENT_BUTTONS,       (int*)&g_touch.buttons);
    f_geti(HD_CURRENT_INKWELL_SWITCH,(int*)&g_touch.inkwell);
    g_touch.ticks++;

    /* 施加力 (读取力指令) */
    double force[3], torque[3];
    pthread_mutex_lock((pthread_mutex_t*)&g_force_mutex);
    force[0]  = g_force_cmd.force[0];
    force[1]  = g_force_cmd.force[1];
    force[2]  = g_force_cmd.force[2];
    torque[0] = g_force_cmd.torque[0];
    torque[1] = g_force_cmd.torque[1];
    torque[2] = g_force_cmd.torque[2];
    pthread_mutex_unlock((pthread_mutex_t*)&g_force_mutex);

    /* 安全限制: 不超过额定最大力 */
    double max_f = 3.0;  /* N, 保守值 */
    double mag = sqrt(force[0]*force[0] + force[1]*force[1] + force[2]*force[2]);
    if (mag > max_f) {
        double scale = max_f / mag;
        force[0] *= scale;
        force[1] *= scale;
        force[2] *= scale;
    }

    f_setd(HD_CURRENT_FORCE, force);
    /* 注: 力矩 (gimbal torque) 可通过 HD_CURRENT_GIMBAL_TORQUE 设置 */

    /* 结束帧 */
    f_end(g_hHD);

    return HD_CALLBACK_CONTINUE;
}

/* ===== 加载 HD 库 ===== */
static int load_hd_lib(void) {
    dlopen("/tmp/fakelibs/libncurses.so.5", RTLD_NOW | RTLD_GLOBAL);
    void* lib = dlopen("/usr/lib/libHD.so", RTLD_NOW | RTLD_GLOBAL);
    if (!lib) {
        fprintf(stderr, "[touch_haptic] dlopen libHD.so: %s\n", dlerror());
        return -1;
    }
/* C++ 中 void* 不能隐式转为函数指针, 用 reinterpret_cast */
#define SYM(var, name) do { \
    (var) = reinterpret_cast<decltype(var)>(dlsym(lib, #name)); \
    if (!(var)) { fprintf(stderr, "[touch_haptic] dlsym %s 失败\n", #name); return -1; } \
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
    f_gets = (hdGetString_t)dlsym(lib, "hdGetString");
    return 0;
}

/* ===== 清空错误队列 ===== */
static void drain_errors(const char* ctx) {
    HDErrorInfo e;
    while (1) {
        e = f_err();
        if (e.errorCode == HD_SUCCESS) break;
        fprintf(stderr, "[touch_haptic/%s] 警告: 0x%04x/%d %s\n",
                ctx, e.errorCode, e.internalErrorCode,
                f_errstr ? f_errstr(e.errorCode) : "");
    }
}

/* ===== 信号处理 ===== */
static void sig_handler(int s) { (void)s; g_running = 0; }

/* ===== 设置力指令 ===== */
void touch_set_force(double fx, double fy, double fz) {
    pthread_mutex_lock(&g_force_mutex);
    g_force_cmd.force[0] = fx;
    g_force_cmd.force[1] = fy;
    g_force_cmd.force[2] = fz;
    pthread_mutex_unlock(&g_force_mutex);
}

/* ===== 获取状态 ===== */
void touch_get_position(double* x, double* y, double* z) {
    *x = (double)g_touch.position[0];
    *y = (double)g_touch.position[1];
    *z = (double)g_touch.position[2];
}

void touch_get_joint_angles(double* q1, double* q2, double* q3) {
    *q1 = (double)g_touch.joint_angles[0];
    *q2 = (double)g_touch.joint_angles[1];
    *q3 = (double)g_touch.joint_angles[2];
}

int touch_get_buttons(void) {
    return g_touch.buttons;
}

/* ===== 初始化 ===== */
int touch_init(void) {
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    fprintf(stderr, "[touch_haptic] 初始化 Touch 力反馈笔...\n");
    fprintf(stderr, "[touch_haptic] 序列号: 220-390-00866\n");

    /* 加载库 */
    if (load_hd_lib() != 0) return -1;
    drain_errors("load");

    /* 初始化设备 */
    g_hHD = f_init(NULL);
    drain_errors("init");

    if (g_hHD == HD_INVALID_HANDLE) {
        fprintf(stderr, "[touch_haptic] hdInitDevice 失败!\n");
        return -1;
    }
    fprintf(stderr, "[touch_haptic] 设备初始化成功 hHD=%u\n", g_hHD);

    /* 读取设备信息 */
    f_getd(HD_NOMINAL_MAX_FORCE, (double*)&g_touch.max_force);
    fprintf(stderr, "[touch_haptic] 额定最大力: %.2f N\n", (double)g_touch.max_force);

    if (f_gets) {
        HDstring sn = f_gets(0x2504);  /* HD_DEVICE_SERIAL_NUMBER */
        fprintf(stderr, "[touch_haptic] 固件序列号: '%s'\n", sn ? sn : "N/A");
    }

    /* 注册 Servo 回调 */
    HDSchedulerHandle h = f_schedA(servo_loop, NULL, HD_DEFAULT_SCHEDULER_PRIORITY);
    drain_errors("schedA");
    fprintf(stderr, "[touch_haptic] Servo 回调已注册, 句柄=%lu\n", h);

    /* 启动调度器 */
    f_start();
    drain_errors("start");
    fprintf(stderr, "[touch_haptic] 调度器已启动\n");

    /* 等待首帧 */
    int waited = 0;
    while ((int)g_touch.ticks == 0 && waited < 100) {
        usleep(10000);
        waited++;
    }
    if ((int)g_touch.ticks > 0) {
        fprintf(stderr, "[touch_haptic] Servo 运行中 (首帧 in %dms)\n", waited * 10);
    } else {
        fprintf(stderr, "[touch_haptic] 警告: Servo 未启动!\n");
    }

    return 0;
}

/* ===== 停止 ===== */
void touch_cleanup(void) {
    fprintf(stderr, "[touch_haptic] 停止...\n");
    g_running = 0;
    touch_set_force(0, 0, 0);
    usleep(50000);  /* 等待最后一帧 */
    if (f_stop) f_stop();
    if (f_disable && g_hHD != HD_INVALID_HANDLE) f_disable(g_hHD);
    fprintf(stderr, "[touch_haptic] 已停止, 总帧数=%d\n", (int)g_touch.ticks);
}

/* ===== 独立运行模式 (不用 ROS) ===== */
int main(void) {
    printf("===== Touch 力反馈笔 ROS 节点 (独立运行模式) =====\n");

    if (touch_init() != 0) return 1;

    printf("\n实时位置输出 (按 Ctrl+C 停止):\n");
    printf("%-7s | %-9s %-9s %-9s | Q1(°)  Q2(°)  Q3(°)  | Btn\n",
           "T(s)", "X(mm)", "Y(mm)", "Z(mm)");
    printf("%.75s\n", "-------------------------------------------------------------------"
           "-------------------------------------------------------------------");

    double t = 0.0;
    int prev_ticks = 0;

    while (g_running) {
        usleep(50000);
        t += 0.05;

        int cur = (int)g_touch.ticks;
        if (cur == prev_ticks) continue;  /* 无新数据 */

        if ((int)(t * 10) % 5 == 0) {  /* 约每 0.5s 打印 */
            double x, y, z, q1, q2, q3;
            touch_get_position(&x, &y, &z);
            touch_get_joint_angles(&q1, &q2, &q3);
            int btn = touch_get_buttons();

            printf("%-7.1f | %-9.2f %-9.2f %-9.2f | %-6.2f %-6.2f %-6.2f | %d\n",
                   t, x, y, z,
                   q1 * 180.0 / M_PI,
                   q2 * 180.0 / M_PI,
                   q3 * 180.0 / M_PI,
                   btn);

            /* 力反馈演示: 按下白色按钮时施加 Z 方向弹力 */
            if (btn & 0x01) {
                /* 白色按钮: Z 方向 0.5N 恒力 (提示已按下) */
                touch_set_force(0.0, 0.0, 0.5);
            } else if (z < -100.0) {
                /* Z < -100mm: 虚拟墙, 产生阻力 */
                double wall_force = (z + 100.0) * (-0.01);  /* 弹性系数 0.01 N/mm */
                touch_set_force(0.0, 0.0, wall_force > 2.0 ? 2.0 : wall_force);
            } else {
                touch_set_force(0.0, 0.0, 0.0);
            }
        }
        prev_ticks = cur;
    }

    touch_cleanup();
    return 0;
}
