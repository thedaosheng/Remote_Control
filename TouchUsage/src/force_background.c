/*
 * Touch 力渲染 — 非交互自动版 (Agent 自己跑自己调)
 *
 * 模式控制: echo N > /tmp/force_mode  (N=0-8)
 * 状态输出: /tmp/force_status (实时更新)
 * 日志:     /tmp/force_auto.log
 * 停止:     echo q > /tmp/force_mode  或 kill -2 PID
 *
 * 编译: gcc -o /tmp/force_auto this_file.c -ldl -lm
 * 运行: LD_LIBRARY_PATH=/tmp/patched_lib:/tmp/fakelibs:/usr/lib /tmp/force_auto
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <dlfcn.h>
#include <math.h>
#include <signal.h>
#include <time.h>

typedef unsigned int HHD, HDuint, HDCallbackCode;
typedef unsigned short HDushort;
typedef unsigned long HDSchedulerHandle;
typedef struct { int errorCode; int internalCode; } HDErrorInfo;

#define HD_CALLBACK_DONE 0
#define HD_CALLBACK_CONTINUE 1
#define HD_CURRENT_BUTTONS 0x2000
#define HD_CURRENT_POSITION 0x2050
#define HD_CURRENT_VELOCITY 0x2051
#define HD_CURRENT_FORCE 0x2700
#define HD_FORCE_OUTPUT 0x4000
#define HD_INVALID_HANDLE 0xFFFFFFFF

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
typedef void (*fn_v)(void);

static fn_init hd_init; static fn_disable hd_disable;
static fn_begin hd_begin; static fn_end hd_end;
static fn_getd hd_getd; static fn_geti hd_geti;
static fn_setd hd_setd; static fn_enable hd_enable;
static fn_err hd_err; static fn_sched hd_sched;
static fn_v hd_start, hd_stop;

static volatile int g_running=1;
static HHD g_hHD=HD_INVALID_HANDLE;
static volatile double g_pos[3],g_vel[3],g_out_f[3];
static volatile int g_buttons=0;
static volatile long g_ticks=0;
static volatile int g_mode=0;
static FILE* g_log=NULL;
#define LOG(fmt,...) do{if(g_log){fprintf(g_log,fmt "\n",##__VA_ARGS__);fflush(g_log);}}while(0)
#define MAX_FORCE 2.5
static double vlen(const double v[3]){return sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);}
static void clamp(double f[3]){double m=vlen(f);if(m>MAX_FORCE){double s=MAX_FORCE/m;f[0]*=s;f[1]*=s;f[2]*=s;}}

/* 真实工作空间中心 (实测) */
#define CX 0.0
#define CY (-100.0)
#define CZ (-170.0)
/* 立方体 */
#define BOX_X 0.0
#define BOX_Y (-50.0)
#define BOX_Z (-200.0)
#define BOX_HALF 25.0
#define BOX_K 0.5

/* === 8 种力渲染 (参数与 force_interactive.c 同步) === */
static void f0(const double p[3],const double v[3],double f[3]){f[0]=f[1]=f[2]=0;}
static void f1(const double p[3],const double v[3],double f[3]){
    f[0]=f[1]=f[2]=0;
    double dx=p[0]-BOX_X,dy=p[1]-BOX_Y,dz=p[2]-BOX_Z,h=BOX_HALF;
    if(dx>-h&&dx<h&&dy>-h&&dy<h&&dz>-h&&dz<h){
        double pens[6]={h-dx,dx+h,h-dy,dy+h,h-dz,dz+h};
        int face=0; double mp=pens[0];
        for(int i=1;i<6;i++){if(pens[i]<mp){mp=pens[i];face=i;}}
        double fm=BOX_K*mp;
        switch(face){
            case 0:f[0]=fm;break; case 1:f[0]=-fm;break;
            case 2:f[1]=fm;break; case 3:f[1]=-fm;break;
            case 4:f[2]=fm;break; case 5:f[2]=-fm;break;
        }
    }
}
static void f2(const double p[3],const double v[3],double f[3]){
    double k=0.010,b=0.001;
    f[0]=-k*(p[0]-CX)-b*v[0]; f[1]=-k*(p[1]-CY)-b*v[1]; f[2]=-k*(p[2]-CZ)-b*v[2];}
static void f3(const double p[3],const double v[3],double f[3]){
    double e=0.008; f[0]=-e*v[0]; f[1]=-e*v[1]; f[2]=-e*v[2];}
static void f4(const double p[3],const double v[3],double f[3]){
    f[0]=f[1]=f[2]=0; double wz=-200.0; if(p[2]<wz){double fn=0.6*(wz-p[2]);f[2]=fn;
    double vt=sqrt(v[0]*v[0]+v[1]*v[1]);if(vt>1.0){double fr=0.5*fn;f[0]-=fr*v[0]/vt;f[1]-=fr*v[1]/vt;}}}
static void f5(const double p[3],const double v[3],double f[3]){
    f[0]=f[1]=f[2]=0; double rx=CX-p[0],ry=CY-p[1],rz=CZ-p[2];double d=sqrt(rx*rx+ry*ry+rz*rz);
    if(d<3)d=3;if(d>100)return;double dc=1.0-(d/100)*(d/100);if(dc<0)dc=0;
    double fm=2000.0/(d*d)*dc;f[0]=fm*rx/d;f[1]=fm*ry/d;f[2]=fm*rz/d;}
static void f6(const double p[3],const double v[3],double f[3]){
    f[0]=f[1]=f[2]=0; double rx=CX-p[0],ry=CY-p[1],rz=CZ-p[2];double d=sqrt(rx*rx+ry*ry+rz*rz);
    if(d<5)d=5;if(d>120)return;double fm=1500.0/(d*d);f[0]=fm*rx/d;f[1]=fm*ry/d;f[2]=fm*rz/d;}
static void f7(const double p[3],const double v[3],double f[3]){
    f[0]=f[1]=f[2]=0; double wz=-200.0; if(p[2]<wz){f[2]=0.4*(wz-p[2])+0.5*sin(2.0*M_PI*p[0]/10.0);}}
static void f8(const double p[3],const double v[3],double f[3]){
    f[0]=f[1]=f[2]=0; double k=0.15,hw=15.0;
    double dy=p[1]-CY; if(dy>hw)f[1]=-k*(dy-hw);if(dy<-hw)f[1]=-k*(dy+hw);
    double dz=p[2]-CZ; if(dz>hw)f[2]=-k*(dz-hw);if(dz<-hw)f[2]=-k*(dz+hw);}

typedef void(*ffn)(const double*,const double*,double*);
static const ffn FNS[]={f0,f1,f2,f3,f4,f5,f6,f7,f8};
static const char* NAMES[]={"OFF","WALL","SPRING","VISCOUS","FRICTION","MAGNET","GRAVITY","TEXTURE","CHANNEL"};
#define N_MODES 9

static HDCallbackCode servo(void*d){
    (void)d;if(!g_running)return HD_CALLBACK_DONE;
    hd_begin(g_hHD);
    double pos[3],vel[3];int btn=0;
    hd_getd(HD_CURRENT_POSITION,pos);hd_getd(HD_CURRENT_VELOCITY,vel);hd_geti(HD_CURRENT_BUTTONS,&btn);
    double force[3]={0,0,0};int m=g_mode;if(m>=0&&m<N_MODES)FNS[m](pos,vel,force);clamp(force);
    hd_setd(HD_CURRENT_FORCE,force);hd_end(g_hHD);
    for(int i=0;i<3;i++){g_pos[i]=pos[i];g_vel[i]=vel[i];g_out_f[i]=force[i];}
    g_buttons=btn;g_ticks++;return HD_CALLBACK_CONTINUE;
}
static void sighandler(int s){(void)s;g_running=0;}

int main(void){
    g_log=fopen("/tmp/force_auto.log","w");
    LOG("=== START ===");
    signal(SIGINT,sighandler);signal(SIGTERM,sighandler);

    /* 加载库 */
    dlopen("/tmp/fakelibs/libncurses.so.5",RTLD_NOW|RTLD_GLOBAL);
    void*lib=dlopen("/usr/lib/libHD.so",RTLD_NOW|RTLD_GLOBAL);
    if(!lib){LOG("FAIL: dlopen libHD.so: %s",dlerror());return 1;}
#define S(v,n) v=dlsym(lib,#n);if(!v){LOG("FAIL: sym %s",#n);return 1;}
    S(hd_init,hdInitDevice);S(hd_disable,hdDisableDevice);
    S(hd_begin,hdBeginFrame);S(hd_end,hdEndFrame);
    S(hd_getd,hdGetDoublev);S(hd_geti,hdGetIntegerv);
    S(hd_setd,hdSetDoublev);S(hd_enable,hdEnable);
    S(hd_err,hdGetError);S(hd_sched,hdScheduleAsynchronous);
    S(hd_start,hdStartScheduler);S(hd_stop,hdStopScheduler);
#undef S

    HDErrorInfo e;while((e=hd_err()).errorCode!=0){}
    g_hHD=hd_init(NULL);e=hd_err();
    LOG("init: handle=%u err=0x%x/%d",g_hHD,e.errorCode,e.internalCode);
    if(g_hHD==HD_INVALID_HANDLE){LOG("INIT FAILED");return 1;}
    while((e=hd_err()).errorCode!=0){}
    hd_enable(HD_FORCE_OUTPUT);
    hd_sched(servo,NULL,(HDushort)500);while((e=hd_err()).errorCode!=0){}
    hd_start();while((e=hd_err()).errorCode!=0){}
    for(int i=0;i<200&&g_ticks==0;i++)usleep(5000);
    if(g_ticks==0){LOG("SERVO NOT RUNNING");return 1;}
    LOG("RUNNING servo=%ldHz",g_ticks);

    /* 初始化模式控制文件 */
    FILE*mf=fopen("/tmp/force_mode","w");if(mf){fprintf(mf,"0\n");fclose(mf);}

    /* 主循环: 读 /tmp/force_mode 切换, 写 /tmp/force_status 状态 */
    long prev=0;int frame=0;
    while(g_running){
        usleep(100000); /* 10Hz */
        frame++;

        /* 读模式文件 */
        FILE*f=fopen("/tmp/force_mode","r");
        if(f){char buf[16]={0};if(fgets(buf,sizeof(buf),f)){
            if(buf[0]=='q'||buf[0]=='Q')g_running=0;
            else if(buf[0]>='0'&&buf[0]<='8'){int nm=buf[0]-'0';if(nm!=g_mode){g_mode=nm;LOG("MODE: %d %s",nm,NAMES[nm]);}}
        }fclose(f);}

        /* 写状态文件 */
        double fm=sqrt(g_out_f[0]*g_out_f[0]+g_out_f[1]*g_out_f[1]+g_out_f[2]*g_out_f[2]);
        long ct=g_ticks,sr=(ct-prev)*10;prev=ct;
        FILE*sf=fopen("/tmp/force_status","w");
        if(sf){fprintf(sf,"mode=%d name=%s\npos=%.1f %.1f %.1f\nforce=%.3f %.3f %.3f mag=%.3f\nbtn=%d servo=%ldHz ticks=%ld\n",
            g_mode,NAMES[g_mode],g_pos[0],g_pos[1],g_pos[2],
            g_out_f[0],g_out_f[1],g_out_f[2],fm,g_buttons,sr,ct);fclose(sf);}

        /* 每 5 秒写一次日志 */
        if(frame%50==0)LOG("t=%ds mode=%d pos=%.1f,%.1f,%.1f F=%.2fN",frame/10,g_mode,g_pos[0],g_pos[1],g_pos[2],fm);
    }

    g_mode=0;usleep(100000);g_running=0;usleep(100000);
    hd_stop();hd_disable(g_hHD);
    LOG("=== CLEAN STOP ticks=%ld ===",g_ticks);
    if(g_log)fclose(g_log);
    unlink("/tmp/force_mode");unlink("/tmp/force_status");
    return 0;
}
