/*
 * Touch 力反馈笔 — 8 种力渲染效果 Demo (v2 - 带日志调试)
 *
 * 编译:
 *   gcc -o /tmp/force_8modes touch/demos/20260409-cc-force_rendering_8modes.c -ldl -lm
 *
 * 运行:
 *   LD_LIBRARY_PATH=/tmp/patched_lib:/tmp/fakelibs:/usr/lib /tmp/force_8modes
 *
 * 日志文件: /tmp/force_8modes.log (Agent 可以实时读取调试)
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <dlfcn.h>
#include <math.h>
#include <signal.h>
#include <termios.h>
#include <fcntl.h>
#include <time.h>

/* ===== 日志 ===== */
static FILE* g_log = NULL;
#define LOG(fmt, ...) do { \
    if (g_log) { fprintf(g_log, fmt "\n", ##__VA_ARGS__); fflush(g_log); } \
} while(0)

/* ===== HD 类型 ===== */
typedef unsigned int HHD, HDuint, HDCallbackCode;
typedef unsigned short HDushort;
typedef unsigned long HDSchedulerHandle;
typedef struct { int errorCode; int internalCode; } HDErrorInfo;

#define HD_CALLBACK_DONE     0
#define HD_CALLBACK_CONTINUE 1
#define HD_DEFAULT_SCHEDULER_PRIORITY ((HDushort)500)

#define HD_CURRENT_BUTTONS       0x2000
#define HD_CURRENT_POSITION      0x2050
#define HD_CURRENT_VELOCITY      0x2051
#define HD_CURRENT_FORCE         0x2700
#define HD_FORCE_OUTPUT          0x4000
#define HD_INVALID_HANDLE        0xFFFFFFFF

/* ===== 函数指针 ===== */
typedef HHD (*fn_init)(const char*);
typedef void (*fn_disable)(HHD);
typedef void (*fn_begin)(HHD);
typedef void (*fn_end)(HHD);
typedef void (*fn_getd)(unsigned int, double*);
typedef void (*fn_geti)(unsigned int, int*);
typedef void (*fn_setd)(unsigned int, const double*);
typedef void (*fn_enable)(unsigned int);
typedef HDErrorInfo (*fn_err)(void);
typedef HDSchedulerHandle (*fn_sched)(HDCallbackCode(*)(void*), void*, HDushort);
typedef void (*fn_startsched)(void);
typedef void (*fn_stopsched)(void);

static fn_init     hd_init;
static fn_disable  hd_disable;
static fn_begin    hd_begin;
static fn_end      hd_end;
static fn_getd     hd_getd;
static fn_geti     hd_geti;
static fn_setd     hd_setd;
static fn_enable   hd_enable;
static fn_err      hd_err;
static fn_sched    hd_sched;
static fn_startsched hd_start;
static fn_stopsched  hd_stop;

/* ===== 全局状态 ===== */
static volatile int g_running = 1;
static HHD g_hHD = HD_INVALID_HANDLE;
static volatile double g_pos[3], g_vel[3];
static volatile int g_buttons = 0;
static volatile long g_ticks = 0;
static volatile int g_mode = 0;
static volatile double g_out_f[3];

#define MAX_FORCE 2.5

static double vlen(const double v[3]) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}
static void clamp(double f[3]) {
    double m = vlen(f);
    if (m > MAX_FORCE) { double s=MAX_FORCE/m; f[0]*=s; f[1]*=s; f[2]*=s; }
}

/* ===== 8 种力渲染 ===== */

static void f0_off(const double p[3], const double v[3], double f[3]) {
    f[0]=f[1]=f[2]=0;
}

static void f1_wall(const double p[3], const double v[3], double f[3]) {
    /* Y=80mm 刚度墙: 下压碰到硬面 */
    f[0]=f[1]=f[2]=0;
    if (p[1] < 80.0) f[1] = 0.5 * (80.0 - p[1]);
}

static void f2_spring(const double p[3], const double v[3], double f[3]) {
    /* 弹簧拉向 (0,120,0) */
    double k=0.012, b=0.001;
    f[0] = -k*p[0] - b*v[0];
    f[1] = -k*(p[1]-120.0) - b*v[1];
    f[2] = -k*p[2] - b*v[2];
}

static void f3_viscous(const double p[3], const double v[3], double f[3]) {
    /* 粘滞力: 蜂蜜感 */
    double eta=0.008;
    f[0]=-eta*v[0]; f[1]=-eta*v[1]; f[2]=-eta*v[2];
}

static void f4_friction(const double p[3], const double v[3], double f[3]) {
    /* Y=80mm 表面 + 库仑摩擦 */
    f[0]=f[1]=f[2]=0;
    if (p[1] < 80.0) {
        double fn = 0.8*(80.0-p[1]);
        f[1] = fn;
        double vt = sqrt(v[0]*v[0]+v[2]*v[2]);
        if (vt > 1.0) {
            double fr = 0.6*fn;
            f[0] -= fr*v[0]/vt;
            f[2] -= fr*v[2]/vt;
        }
    }
}

static void f5_magnet(const double p[3], const double v[3], double f[3]) {
    /* 磁吸: 靠近(0,120,0)被吸住 */
    f[0]=f[1]=f[2]=0;
    double rx=0-p[0], ry=120.0-p[1], rz=0-p[2];
    double d=sqrt(rx*rx+ry*ry+rz*rz);
    if(d<2)d=2; if(d>40)return;
    double decay=1.0-(d/40)*(d/40);
    if(decay<0)decay=0;
    double fm=3000.0/(d*d)*decay;
    f[0]=fm*rx/d; f[1]=fm*ry/d; f[2]=fm*rz/d;
}

static void f6_gravity(const double p[3], const double v[3], double f[3]) {
    /* 重力井: 引力陷阱 */
    f[0]=f[1]=f[2]=0;
    double rx=0-p[0], ry=120.0-p[1], rz=0-p[2];
    double d=sqrt(rx*rx+ry*ry+rz*rz);
    if(d<3)d=3; if(d>60)return;
    double fm=2000.0/(d*d);
    f[0]=fm*rx/d; f[1]=fm*ry/d; f[2]=fm*rz/d;
}

static void f7_texture(const double p[3], const double v[3], double f[3]) {
    /* 振动纹理: Y=80mm 表面 + 正弦凹凸 */
    f[0]=f[1]=f[2]=0;
    if (p[1] < 80.0) {
        double base = 0.5*(80.0-p[1]);
        double tex = 0.4*sin(2.0*M_PI*p[0]/8.0);
        f[1] = base + tex;
    }
}

static void f8_channel(const double p[3], const double v[3], double f[3]) {
    /* 引导槽: 只能沿X轴走 */
    f[0]=f[1]=f[2]=0;
    double k=0.3, hw=5.0;
    double dy=p[1]-120.0;
    if(dy> hw) f[1]=-k*(dy-hw);
    if(dy<-hw) f[1]=-k*(dy+hw);
    double dz=p[2];
    if(dz> hw) f[2]=-k*(dz-hw);
    if(dz<-hw) f[2]=-k*(dz+hw);
}

typedef void (*force_fn)(const double*,const double*,double*);
static const force_fn FNS[] = {f0_off,f1_wall,f2_spring,f3_viscous,f4_friction,f5_magnet,f6_gravity,f7_texture,f8_channel};
static const char* NAMES[] = {
    "OFF   关闭力输出",
    "WALL  刚度墙(下压Y<80mm碰硬面)",
    "SPRING 弹簧回中(拉向中心)",
    "VISC  粘滞力场(蜂蜜感)",
    "FRIC  表面摩擦(平面+滑动阻力)",
    "MAG   磁吸(靠近中心吸住)",
    "GRAV  重力井(引力陷阱)",
    "TEX   振动纹理(左右搓衣板)",
    "CHAN  引导槽(只能左右走)",
};
#define N_MODES 9

/* ===== 伺服回调 ===== */
static HDCallbackCode servo(void* d) {
    (void)d;
    if (!g_running) return HD_CALLBACK_DONE;
    hd_begin(g_hHD);

    double pos[3],vel[3]; int btn=0;
    hd_getd(HD_CURRENT_POSITION,pos);
    hd_getd(HD_CURRENT_VELOCITY,vel);
    hd_geti(HD_CURRENT_BUTTONS,&btn);

    double force[3]={0,0,0};
    int m=g_mode;
    if(m>=0 && m<N_MODES) FNS[m](pos,vel,force);
    clamp(force);
    hd_setd(HD_CURRENT_FORCE,force);

    hd_end(g_hHD);
    for(int i=0;i<3;i++){g_pos[i]=pos[i];g_vel[i]=vel[i];g_out_f[i]=force[i];}
    g_buttons=btn; g_ticks++;
    return HD_CALLBACK_CONTINUE;
}

static void sighandler(int s){(void)s; g_running=0;}
static struct termios g_tio;
static void term_raw(void){struct termios t; tcgetattr(0,&g_tio); t=g_tio; t.c_lflag&=~(ICANON|ECHO); t.c_cc[VMIN]=0;t.c_cc[VTIME]=0; tcsetattr(0,TCSANOW,&t);}
static void term_restore(void){tcsetattr(0,TCSANOW,&g_tio);}

int main(void) {
    /* 打开日志文件 */
    g_log = fopen("/tmp/force_8modes.log","w");
    LOG("=== Touch 8 modes demo started ===");

    signal(SIGINT,sighandler);
    signal(SIGTERM,sighandler);

    /* === 加载库 === */
    LOG("Loading fakelibs...");
    void* nc = dlopen("/tmp/fakelibs/libncurses.so.5", RTLD_NOW|RTLD_GLOBAL);
    LOG("  ncurses: %p (%s)", nc, nc?"ok":dlerror());

    LOG("Loading libHD.so...");
    void* lib = dlopen("/usr/lib/libHD.so", RTLD_NOW|RTLD_GLOBAL);
    LOG("  libHD: %p (%s)", lib, lib?"ok":dlerror());
    if(!lib){fprintf(stderr,"加载 libHD.so 失败\n");return 1;}

    /* 加载符号 */
#define S(v,n) v=dlsym(lib,#n); LOG("  %s: %p",#n,(void*)v); if(!v){fprintf(stderr,"缺: %s\n",#n);return 1;}
    S(hd_init,hdInitDevice); S(hd_disable,hdDisableDevice);
    S(hd_begin,hdBeginFrame); S(hd_end,hdEndFrame);
    S(hd_getd,hdGetDoublev); S(hd_geti,hdGetIntegerv);
    S(hd_setd,hdSetDoublev); S(hd_enable,hdEnable);
    S(hd_err,hdGetError); S(hd_sched,hdScheduleAsynchronous);
    S(hd_start,hdStartScheduler); S(hd_stop,hdStopScheduler);
#undef S

    /* 诊断: 打印 LD_LIBRARY_PATH 和实际加载的库路径 */
    LOG("ENV LD_LIBRARY_PATH=%s", getenv("LD_LIBRARY_PATH") ? getenv("LD_LIBRARY_PATH") : "<null>");
    LOG("Dumping /proc/self/maps for PhantomIO...");
    {
        FILE* maps = fopen("/proc/self/maps","r");
        if(maps){ char line[512]; while(fgets(line,sizeof(line),maps)){
            if(strstr(line,"PhantomIO") || strstr(line,"libHD") || strstr(line,"ncurses"))
                LOG("  MAP: %s", line);
        } fclose(maps);}
    }

    /* 清空残留错误 */
    LOG("Draining errors...");
    HDErrorInfo e;
    int drain_count = 0;
    while((e=hd_err()).errorCode!=0) {
        LOG("  drained error: 0x%x / %d", e.errorCode, e.internalCode);
        drain_count++;
        if(drain_count>20) break;  /* 防死循环 */
    }
    LOG("  drained %d errors", drain_count);

    /* === 初始化设备 === */
    LOG("hdInitDevice(NULL)...");
    g_hHD = hd_init(NULL);
    e = hd_err();
    LOG("  handle=%u (0x%x), error=0x%x/%d", g_hHD, g_hHD, e.errorCode, e.internalCode);

    if(g_hHD == HD_INVALID_HANDLE) {
        LOG("INIT FAILED (0x%x/%d), attempting recovery via reset_device.sh", e.errorCode, e.internalCode);
        fprintf(stderr,"\n⚠️  初始化失败 (0x%x), 尝试自动恢复...\n", e.errorCode);

        /* 调用独立的重置脚本 */
        int rc = system("bash /home/rhz/teleop/TouchUsage/scripts/reset_device.sh");
        LOG("reset_device.sh exit: %d", rc);

        if(rc == 0) {
            fprintf(stderr,"   重置完成，重新初始化...\n");
            while((e=hd_err()).errorCode!=0) {}
            g_hHD = hd_init(NULL);
            e = hd_err();
            LOG("  retry handle=%u (0x%x), error=0x%x/%d", g_hHD, g_hHD, e.errorCode, e.internalCode);
        }

        if(g_hHD == HD_INVALID_HANDLE) {
            fprintf(stderr,"\n❌ 恢复失败! 请拔插 USB 后重试。\n");
            LOG("RECOVERY FAILED");
            if(g_log) fclose(g_log);
            return 1;
        }
        fprintf(stderr,"   ✅ 恢复成功!\n");
    }

    /* 清空 init 后的错误 */
    while((e=hd_err()).errorCode!=0) {
        LOG("  post-init error (ignored): 0x%x/%d", e.errorCode, e.internalCode);
    }

    /* 启用力输出 */
    LOG("hdEnable(HD_FORCE_OUTPUT)...");
    hd_enable(HD_FORCE_OUTPUT);
    LOG("  done");

    /* 注册伺服回调 */
    LOG("hdScheduleAsynchronous...");
    HDSchedulerHandle sh = hd_sched(servo, NULL, HD_DEFAULT_SCHEDULER_PRIORITY);
    LOG("  sched handle=%lu", sh);
    while((e=hd_err()).errorCode!=0) LOG("  sched error: 0x%x/%d",e.errorCode,e.internalCode);

    /* 启动调度器 */
    LOG("hdStartScheduler...");
    hd_start();
    while((e=hd_err()).errorCode!=0) LOG("  start error (ignored): 0x%x/%d",e.errorCode,e.internalCode);
    LOG("  scheduler started");

    /* 等待伺服首帧 */
    for(int i=0;i<200&&g_ticks==0;i++) usleep(5000);
    if(g_ticks==0){
        fprintf(stderr,"❌ 伺服回调未启动!\n");
        LOG("SERVO NOT RUNNING - exiting");
        if(g_log) fclose(g_log);
        return 1;
    }
    LOG("Servo running, ticks=%ld", g_ticks);

    /* === 终端界面 === */
    printf("\033[2J\033[H");  /* 清屏 */
    printf("╔══════════════════════════════════════════════════╗\n");
    printf("║   Touch 力反馈笔 — 8 种力渲染效果 Demo          ║\n");
    printf("╚══════════════════════════════════════════════════╝\n\n");
    for(int i=0;i<N_MODES;i++)
        printf("  [%d] %s\n",i,NAMES[i]);
    printf("\n  [q] 退出    日志: /tmp/force_8modes.log\n");
    printf("──────────────────────────────────────────────────\n");
    printf("▶ 当前: [0] %s\n\n",NAMES[0]);

    term_raw();

    long prev=0;
    while(g_running) {
        usleep(50000);

        /* 键盘 */
        char ch;
        if(read(0,&ch,1)==1) {
            if(ch=='q'||ch=='Q') break;
            if(ch>='0'&&ch<='8') {
                g_mode=ch-'0';
                printf("\033[2K\r▶ 切换到 [%d] %s\n", g_mode, NAMES[g_mode]);
                LOG("MODE SWITCH: %d %s", g_mode, NAMES[g_mode]);
            }
        }

        /* 状态行 */
        double fmag=sqrt(g_out_f[0]*g_out_f[0]+g_out_f[1]*g_out_f[1]+g_out_f[2]*g_out_f[2]);
        long ct=g_ticks, sr=(ct-prev)*20; prev=ct;
        printf("\033[2K\r  [%d] X=%6.1f Y=%6.1f Z=%6.1f | F=%5.2fN [%+5.2f %+5.2f %+5.2f] | btn=%d %ldHz",
               g_mode, g_pos[0],g_pos[1],g_pos[2],
               fmag, g_out_f[0],g_out_f[1],g_out_f[2],
               g_buttons, sr);
        fflush(stdout);
    }

    /* 清理 */
    term_restore();
    printf("\n\n正在安全关闭...\n");
    g_mode=0; usleep(100000);
    g_running=0; usleep(100000);
    hd_stop(); hd_disable(g_hHD);
    LOG("=== Clean shutdown, total ticks=%ld ===", g_ticks);
    printf("完成! 总帧: %ld\n", g_ticks);
    if(g_log) fclose(g_log);
    return 0;
}
