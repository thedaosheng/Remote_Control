/*
 * Touch 力反馈笔 — 8 种力渲染效果 (基于已验证可用的 touch_demo 框架)
 *
 * 编译:
 *   gcc -o /tmp/force_interactive THIS_FILE -ldl -lm
 * 运行:
 *   LD_LIBRARY_PATH=/tmp/patched_lib:/tmp/fakelibs:/usr/lib /tmp/force_interactive
 *
 * 操作: 键盘 0-8 切效果, q 退出
 *
 * 注意: 本文件的库加载/初始化/调度器代码 100% 复用自
 *       20260329-cc-touch_demo.c (已在真机上验证通过)
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
#include <termios.h>

/* ===== HD 类型定义 (与 touch_demo 完全一致) ===== */
typedef unsigned int   HHD;
typedef unsigned int   HDuint;
typedef int            HDint;
typedef unsigned short HDushort;
typedef double         HDdouble;
typedef unsigned long  HDSchedulerHandle;
typedef unsigned int   HDCallbackCode;
typedef const char*    HDstring;

#define HD_CALLBACK_DONE     0
#define HD_CALLBACK_CONTINUE 1
#define HD_DEFAULT_SCHEDULER_PRIORITY ((HDushort)500)

#define HD_CURRENT_BUTTONS          0x2000
#define HD_CURRENT_INKWELL_SWITCH   0x2002
#define HD_CURRENT_POSITION         0x2050
#define HD_CURRENT_VELOCITY         0x2051
#define HD_CURRENT_JOINT_ANGLES     0x2100
#define HD_CURRENT_GIMBAL_ANGLES    0x2150
#define HD_DEVICE_SERIAL_NUMBER     0x2504
#define HD_CURRENT_FORCE            0x2700
#define HD_INVALID_HANDLE           0xFFFFFFFF
#define HD_FORCE_OUTPUT             0x4000

/* ===== 函数指针 (与 touch_demo 完全一致) ===== */
typedef HHD (*hdInitDevice_t)(const char*);
typedef void (*hdDisableDevice_t)(HHD);
typedef void (*hdBeginFrame_t)(HHD);
typedef void (*hdEndFrame_t)(HHD);
typedef void (*hdGetDoublev_t)(unsigned int, double*);
typedef void (*hdGetIntegerv_t)(unsigned int, int*);
typedef HDstring (*hdGetString_t)(unsigned int);
typedef void (*hdSetDoublev_t)(unsigned int, const double*);
typedef void (*hdEnable_t)(unsigned int);
typedef struct { int errorCode; int internalCode; } HDErrorInfo;
typedef HDErrorInfo (*hdGetError_t)(void);
typedef HDSchedulerHandle (*hdScheduleAsynchronous_t)(
    HDCallbackCode (*cb)(void*), void*, HDushort);
typedef void (*hdStartScheduler_t)(void);
typedef void (*hdStopScheduler_t)(void);

static hdInitDevice_t    f_init;
static hdDisableDevice_t f_disable;
static hdBeginFrame_t    f_begin;
static hdEndFrame_t      f_end;
static hdGetDoublev_t    f_getd;
static hdGetIntegerv_t   f_geti;
static hdGetString_t     f_gets;
static hdSetDoublev_t    f_setd;
static hdEnable_t        f_enable;
static hdGetError_t      f_err;
static hdScheduleAsynchronous_t f_schedA;
static hdStartScheduler_t f_start;
static hdStopScheduler_t  f_stop;

/* ===== 全局状态 ===== */
static volatile int g_running = 1;
static HHD g_hHD = HD_INVALID_HANDLE;
static volatile double g_pos[3], g_vel[3];
static volatile int g_buttons = 0;
static volatile long g_ticks = 0;
static volatile int g_mode = 0;
static volatile double g_out_f[3];

#define MAX_FORCE 2.5
static void clamp(double f[3]) {
    double m = sqrt(f[0]*f[0]+f[1]*f[1]+f[2]*f[2]);
    if (m > MAX_FORCE) { double s=MAX_FORCE/m; f[0]*=s; f[1]*=s; f[2]*=s; }
}

/* ===== 8 种力渲染 ===== */

/*
 * 真实工作空间 (手持笔自然移动时的实测范围):
 *   X: -77 ~ +38 mm   中心 ≈ 0
 *   Y: -213 ~ -8 mm   中心 ≈ -100   (全程负值! Y 越大=越高)
 *   Z: -249 ~ -77 mm  中心 ≈ -170   (全程负值! Z 越大=越靠近用户)
 *
 * 工作空间中心: (0, -100, -170)
 */
#define CX 0.0
#define CY (-100.0)
#define CZ (-170.0)

/* 刚度墙高度: 手持中位偏下一点，从上往下压能碰到 */
#define WALL_Y (-130.0)

static void calc_force(int mode, const double p[3], const double v[3], double f[3]) {
    f[0]=f[1]=f[2]=0;
    switch(mode) {
    case 0: break; /* OFF */

    case 1: /* 刚度墙: 从上往下压碰到硬面, 越压越硬, 往上抬就自由 */
        if(p[1] < WALL_Y) f[1] = 0.4 * (WALL_Y - p[1]);
        break;

    case 2: /* 弹簧回中: 拉向工作空间中心 */
        { double k=0.010, b=0.001;
          f[0] = -k*(p[0]-CX) - b*v[0];
          f[1] = -k*(p[1]-CY) - b*v[1];
          f[2] = -k*(p[2]-CZ) - b*v[2]; }
        break;

    case 3: /* 粘滞力场: 速度越快阻力越大 */
        { double e=0.008;
          f[0]=-e*v[0]; f[1]=-e*v[1]; f[2]=-e*v[2]; }
        break;

    case 4: /* 表面摩擦: 在 WALL_Y 处碰面后水平滑动有摩擦 */
        if(p[1] < WALL_Y) {
            double fn = 0.6*(WALL_Y - p[1]);
            f[1] = fn;  /* 法向力向上 */
            double vt = sqrt(v[0]*v[0] + v[2]*v[2]);
            if(vt > 1.0) {
                double fr = 0.5 * fn;
                f[0] -= fr*v[0]/vt;
                f[2] -= fr*v[2]/vt;
            }
        }
        break;

    case 5: /* 磁吸: 靠近工作空间中心被吸住 */
        { double rx=CX-p[0], ry=CY-p[1], rz=CZ-p[2];
          double d=sqrt(rx*rx+ry*ry+rz*rz);
          if(d<3) d=3; if(d>100) break;
          double dc=1.0-(d/100)*(d/100); if(dc<0) dc=0;
          double fm=2000.0/(d*d)*dc;
          f[0]=fm*rx/d; f[1]=fm*ry/d; f[2]=fm*rz/d; }
        break;

    case 6: /* 重力井: 温和引力拉向中心 */
        { double rx=CX-p[0], ry=CY-p[1], rz=CZ-p[2];
          double d=sqrt(rx*rx+ry*ry+rz*rz);
          if(d<5) d=5; if(d>120) break;
          double fm=1500.0/(d*d);
          f[0]=fm*rx/d; f[1]=fm*ry/d; f[2]=fm*rz/d; }
        break;

    case 7: /* 振动纹理: 下压碰面后左右滑动有搓衣板感 */
        if(p[1] < WALL_Y) {
            double base = 0.4*(WALL_Y - p[1]);
            double tex = 0.5*sin(2.0*M_PI*p[0]/10.0);
            f[1] = base + tex;
        }
        break;

    case 8: /* 引导槽: 只能沿 X 轴(左右)自由移动, Y/Z 偏离中心有恢复力 */
        { double k=0.15, hw=15.0;
          double dy = p[1] - CY;
          if(dy >  hw) f[1] = -k*(dy - hw);
          if(dy < -hw) f[1] = -k*(dy + hw);
          double dz = p[2] - CZ;
          if(dz >  hw) f[2] = -k*(dz - hw);
          if(dz < -hw) f[2] = -k*(dz + hw); }
        break;
    }
}

static const char* MODE_NAMES[] = {
    "OFF   关闭","WALL  刚度墙","SPRING 弹簧","VISC  粘滞",
    "FRIC  摩擦","MAG   磁吸","GRAV  重力井","TEX   纹理","CHAN  引导槽"
};

/* ===== Servo 回调 (与 touch_demo 结构一致) ===== */
static HDCallbackCode servo_loop(void* data) {
    (void)data;
    if (!g_running) return HD_CALLBACK_DONE;
    f_begin(g_hHD);

    double pos[3], vel[3];
    int buttons = 0;
    f_getd(HD_CURRENT_POSITION, pos);
    f_getd(HD_CURRENT_VELOCITY, vel);
    f_geti(HD_CURRENT_BUTTONS, &buttons);

    double force[3] = {0,0,0};
    calc_force(g_mode, pos, vel, force);
    clamp(force);
    f_setd(HD_CURRENT_FORCE, force);

    f_end(g_hHD);

    for(int i=0;i<3;i++){g_pos[i]=pos[i]; g_vel[i]=vel[i]; g_out_f[i]=force[i];}
    g_buttons = buttons;
    g_ticks++;
    return HD_CALLBACK_CONTINUE;
}

static void sig_handler(int s) { (void)s; g_running = 0; }

/* ===== load_libs (与 touch_demo 完全一致) ===== */
static int load_libs(void) {
    dlopen("/tmp/fakelibs/libncurses.so.5", RTLD_NOW | RTLD_GLOBAL);
    void* lib = dlopen("/usr/lib/libHD.so", RTLD_NOW | RTLD_GLOBAL);
    if (!lib) { fprintf(stderr, "dlopen libHD.so: %s\n", dlerror()); return -1; }

#define SYM(var, name) do { \
    var = dlsym(lib, #name); \
    if (!var) { fprintf(stderr, "缺符号: %s\n", #name); return -1; } \
} while(0)
    SYM(f_init,   hdInitDevice);
    SYM(f_disable,hdDisableDevice);
    SYM(f_begin,  hdBeginFrame);
    SYM(f_end,    hdEndFrame);
    SYM(f_getd,   hdGetDoublev);
    SYM(f_geti,   hdGetIntegerv);
    SYM(f_setd,   hdSetDoublev);
    SYM(f_err,    hdGetError);
    SYM(f_schedA, hdScheduleAsynchronous);
    SYM(f_start,  hdStartScheduler);
    SYM(f_stop,   hdStopScheduler);
#undef SYM
    f_gets = dlsym(lib, "hdGetString");
    f_enable = dlsym(lib, "hdEnable");
    return 0;
}

static void drain_errors(void) {
    HDErrorInfo e;
    while ((e = f_err()).errorCode != 0) {}
}

/* 终端 */
static struct termios g_old_tio;
static void term_raw(void) {
    struct termios t; tcgetattr(0,&g_old_tio); t=g_old_tio;
    t.c_lflag &= ~(ICANON|ECHO); t.c_cc[VMIN]=0; t.c_cc[VTIME]=0;
    tcsetattr(0,TCSANOW,&t);
}
static void term_restore(void) { tcsetattr(0,TCSANOW,&g_old_tio); }

/* ===== main (初始化流程与 touch_demo 完全一致) ===== */
int main(void) {
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    if (load_libs() != 0) return 1;
    drain_errors();

    /* 初始化 (与 touch_demo 一致) */
    g_hHD = f_init(NULL);
    if (g_hHD == HD_INVALID_HANDLE) {
        HDErrorInfo e = f_err();
        fprintf(stderr, "初始化失败: 0x%x / %d\n", e.errorCode, e.internalCode);
        return 1;
    }
    drain_errors();

    /* 启用力输出 */
    if (f_enable) f_enable(HD_FORCE_OUTPUT);

    /* 注册伺服 + 启动 (与 touch_demo 一致) */
    f_schedA(servo_loop, NULL, HD_DEFAULT_SCHEDULER_PRIORITY);
    drain_errors();
    f_start();
    drain_errors();

    /* 等待伺服 */
    for (int i = 0; i < 200 && g_ticks == 0; i++) usleep(5000);
    if (g_ticks == 0) { fprintf(stderr, "伺服未启动!\n"); return 1; }

    /* UI */
    printf("\033[2J\033[H");
    printf("Touch 力反馈 — 按 0-8 切效果, q 退出\n\n");
    for(int i=0;i<9;i++) printf("  [%d] %s\n",i,MODE_NAMES[i]);
    printf("\n▶ [0] %s\n\n", MODE_NAMES[0]);

    term_raw();

    long prev=0;
    while(g_running) {
        usleep(50000);
        char ch;
        if(read(0,&ch,1)==1) {
            if(ch=='q'||ch=='Q') break;
            if(ch>='0'&&ch<='8') {
                g_mode=ch-'0';
                printf("\033[2K\r▶ [%d] %s\n",g_mode,MODE_NAMES[g_mode]);
            }
        }
        double fm=sqrt(g_out_f[0]*g_out_f[0]+g_out_f[1]*g_out_f[1]+g_out_f[2]*g_out_f[2]);
        long ct=g_ticks,sr=(ct-prev)*20; prev=ct;
        printf("\033[2K\r  [%d] X=%6.1f Y=%6.1f Z=%6.1f | F=%5.2fN [%+.2f %+.2f %+.2f] | %ldHz",
               g_mode,g_pos[0],g_pos[1],g_pos[2],fm,g_out_f[0],g_out_f[1],g_out_f[2],sr);
        fflush(stdout);
    }

    term_restore();
    printf("\n\n停止中...\n");
    g_mode=0; g_running=0; usleep(200000);
    f_stop(); f_disable(g_hHD);
    printf("完成! 总帧: %ld\n", g_ticks);
    return 0;
}
