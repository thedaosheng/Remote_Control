/*****************************************************************************
 * 20260328-cc-haptic_demo.c
 *
 * 3D Systems Touch 力反馈笔综合演示程序
 * 功能：
 *   1. 实时读取笔的位置、姿态、关节角、按钮状态
 *   2. 多种力反馈模式：重力井、弹簧、振动、阻尼、虚拟墙
 *   3. 交互式菜单控制切换模式和参数调节
 *
 * 编译方式见 Makefile
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <time.h>

#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>

/* =========================================================================
 * 全局常量和枚举：定义可用的力反馈模式
 * ========================================================================= */
typedef enum {
    MODE_NONE = 0,          /* 无力输出，仅读取数据 */
    MODE_GRAVITY_WELL,      /* 重力井：在原点附近产生吸引力 */
    MODE_SPRING,            /* 弹簧：将笔拉回原点的弹簧力 */
    MODE_VIBRATION,         /* 振动：Y 轴正弦振动 */
    MODE_DAMPING,           /* 阻尼：与速度方向相反的阻力 */
    MODE_VIRTUAL_WALL,      /* 虚拟墙：Z 方向平面墙，阻止笔穿过 */
    MODE_VIRTUAL_BOX,       /* 虚拟盒子：六面体约束，笔碰壁时反弹 */
    MODE_COUNT              /* 模式总数，用于循环切换 */
} ForceMode;

/* 各模式的名称字符串，用于终端显示 */
static const char* MODE_NAMES[] = {
    "无力输出 (仅读取)",
    "重力井 (Gravity Well)",
    "弹簧 (Spring)",
    "振动 (Vibration)",
    "阻尼 (Damping)",
    "虚拟墙 (Virtual Wall)",
    "虚拟盒子 (Virtual Box)"
};

/* =========================================================================
 * 全局状态结构体：在伺服线程和主线程之间共享
 * ========================================================================= */
typedef struct {
    /* === 输入数据（由伺服线程写入，主线程读取） === */
    hduVector3Dd position;      /* 笛卡尔位置 (mm) */
    hduVector3Dd velocity;      /* 笛卡尔速度 (mm/s) */
    HDdouble transform[16];     /* 4x4 变换矩阵（含姿态） */
    HDdouble jointAngles[3];    /* 关节角度 (rad) */
    HDdouble gimbalAngles[3];   /* 万向节角度 (rad) */
    HDlong   encoderValues[3];  /* 原始编码器值 */
    HDint    buttons;           /* 按钮位掩码 */
    HDboolean inkwellSwitch;    /* 墨水槽开关（笔是否在底座上） */

    /* === 力反馈参数（由主线程写入，伺服线程读取） === */
    ForceMode mode;             /* 当前力反馈模式 */
    HDdouble  stiffness;        /* 弹簧刚度 (N/mm) */
    HDdouble  damping;          /* 阻尼系数 (N·s/mm) */
    HDdouble  vibFreq;          /* 振动频率 (Hz) */
    HDdouble  vibAmplitude;     /* 振动幅值 (N) */
    HDdouble  wallZ;            /* 虚拟墙 Z 坐标 (mm) */
    HDdouble  boxSize;          /* 虚拟盒子半径 (mm) */

    /* === 输出数据（由伺服线程写入，供主线程显示） === */
    hduVector3Dd currentForce;  /* 当前输出的力 (N) */
    HDdouble updateRate;        /* 伺服循环更新率 (Hz) */

    /* === 控制标志 === */
    HDboolean running;          /* 程序运行标志 */
    HDErrorInfo error;          /* 最近一次错误 */
} HapticState;

/* 全局状态实例 */
static HapticState gState;

/* =========================================================================
 * 非阻塞键盘输入辅助函数
 * 用于在主循环中检测按键，不阻塞程序执行
 * ========================================================================= */
static struct termios gOrigTermios;
static int gTermiosModified = 0;

/* 设置终端为非阻塞模式：关闭回显和行缓冲 */
static void enableRawMode(void) {
    struct termios raw;
    tcgetattr(STDIN_FILENO, &gOrigTermios);
    gTermiosModified = 1;
    raw = gOrigTermios;
    raw.c_lflag &= ~(ECHO | ICANON);   /* 关闭回显和规范模式 */
    raw.c_cc[VMIN] = 0;                 /* 非阻塞读取 */
    raw.c_cc[VTIME] = 0;               /* 无超时 */
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

/* 恢复终端为原始模式 */
static void disableRawMode(void) {
    if (gTermiosModified) {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &gOrigTermios);
        gTermiosModified = 0;
    }
}

/* 检测是否有按键按下（非阻塞） */
static int kbhit(void) {
    char c;
    int n = read(STDIN_FILENO, &c, 1);
    if (n > 0) return (unsigned char)c;
    return 0;
}

/* =========================================================================
 * 信号处理：优雅退出
 * ========================================================================= */
static void signalHandler(int sig) {
    (void)sig;
    gState.running = HD_FALSE;
}

/* =========================================================================
 * 伺服回调函数（核心）
 * 在 ~1kHz 的伺服线程中执行，读取设备数据并计算/输出力
 * ========================================================================= */
HDCallbackCode HDCALLBACK servoCallback(void *pUserData) {
    (void)pUserData;
    HDErrorInfo error;
    hduVector3Dd force = {0, 0, 0};    /* 初始化力输出为零 */

    HHD hHD = hdGetCurrentDevice();

    /* --- 开始帧：获取设备最新状态 --- */
    hdBeginFrame(hHD);

    /* 读取笛卡尔空间位置 (x, y, z)，单位 mm */
    hdGetDoublev(HD_CURRENT_POSITION, gState.position);

    /* 读取笛卡尔速度 (vx, vy, vz)，单位 mm/s */
    hdGetDoublev(HD_CURRENT_VELOCITY, gState.velocity);

    /* 读取 4x4 变换矩阵（包含位置和旋转信息） */
    hdGetDoublev(HD_CURRENT_TRANSFORM, gState.transform);

    /* 读取关节角度（3 个主关节） */
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, gState.jointAngles);

    /* 读取万向节角度（笔尖的 3 个旋转自由度） */
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gState.gimbalAngles);

    /* 读取原始编码器值 */
    hdGetLongv(HD_CURRENT_ENCODER_VALUES, gState.encoderValues);

    /* 读取按钮状态（位掩码） */
    hdGetIntegerv(HD_CURRENT_BUTTONS, &gState.buttons);

    /* 读取墨水槽开关 */
    hdGetBooleanv(HD_CURRENT_INKWELL_SWITCH, &gState.inkwellSwitch);

    /* 读取伺服循环实际更新频率 */
    hdGetDoublev(HD_INSTANTANEOUS_UPDATE_RATE, &gState.updateRate);

    /* ---------------------------------------------------------------
     * 根据当前模式计算力输出
     * --------------------------------------------------------------- */
    switch (gState.mode) {

        case MODE_GRAVITY_WELL: {
            /* 重力井：当笔在原点附近一定范围内时，产生朝向原点的吸引力
             * F = k * (origin - position)，仅在距离 < influence 时生效 */
            const HDdouble influence = 60.0;    /* 影响半径 60mm */
            hduVector3Dd diff;
            /* diff = 原点(0,0,0) - 当前位置 */
            diff[0] = -gState.position[0];
            diff[1] = -gState.position[1];
            diff[2] = -gState.position[2];
            /* 计算到原点的距离 */
            HDdouble dist = sqrt(diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2]);
            if (dist < influence && dist > 0.001) {
                /* 在影响范围内，施加弹簧力 */
                force[0] = gState.stiffness * diff[0];
                force[1] = gState.stiffness * diff[1];
                force[2] = gState.stiffness * diff[2];
            }
            break;
        }

        case MODE_SPRING: {
            /* 弹簧：始终将笔拉回原点，力 = -k * position
             * 这会让用户感觉到持续的回复力 */
            force[0] = -gState.stiffness * gState.position[0];
            force[1] = -gState.stiffness * gState.position[1];
            force[2] = -gState.stiffness * gState.position[2];
            break;
        }

        case MODE_VIBRATION: {
            /* 振动：沿 Y 轴施加正弦振动力
             * F_y = amplitude * sin(2π * freq * time)
             * 使用 HD_INSTANTANEOUS_UPDATE_RATE 的倒数作为时间步长 */
            static HDdouble timer = 0.0;
            HDdouble instRate;
            hdGetDoublev(HD_INSTANTANEOUS_UPDATE_RATE, &instRate);
            if (instRate > 0) {
                timer += 1.0 / instRate;
            }
            force[0] = 0;
            force[1] = gState.vibAmplitude * sin(2.0 * M_PI * gState.vibFreq * timer);
            force[2] = 0;
            break;
        }

        case MODE_DAMPING: {
            /* 阻尼：产生与速度方向相反的力，模拟在粘稠液体中运动
             * F = -damping_coefficient * velocity */
            force[0] = -gState.damping * gState.velocity[0];
            force[1] = -gState.damping * gState.velocity[1];
            force[2] = -gState.damping * gState.velocity[2];
            break;
        }

        case MODE_VIRTUAL_WALL: {
            /* 虚拟墙：在 Z = wallZ 处放置一面墙，笔无法穿过
             * 当 Z < wallZ 时，施加正 Z 方向的反弹力 */
            if (gState.position[2] < gState.wallZ) {
                /* 穿透深度 */
                HDdouble penetration = gState.wallZ - gState.position[2];
                /* 弹簧力 + 阻尼力（减少振荡） */
                force[2] = gState.stiffness * penetration
                         - gState.damping * gState.velocity[2];
                /* 钳制最大力，防止发散 */
                if (force[2] > 3.0) force[2] = 3.0;
                if (force[2] < 0) force[2] = 0;
            }
            break;
        }

        case MODE_VIRTUAL_BOX: {
            /* 虚拟盒子：在 ±boxSize 范围内创建六面墙约束
             * 笔在盒子内自由移动，碰到壁面时产生反弹力 */
            HDdouble bs = gState.boxSize;
            int i;
            for (i = 0; i < 3; i++) {
                if (gState.position[i] > bs) {
                    /* 超出正向边界 */
                    HDdouble pen = gState.position[i] - bs;
                    force[i] = -gState.stiffness * pen
                             - gState.damping * gState.velocity[i];
                } else if (gState.position[i] < -bs) {
                    /* 超出负向边界 */
                    HDdouble pen = -bs - gState.position[i];
                    force[i] = gState.stiffness * pen
                             - gState.damping * gState.velocity[i];
                }
            }
            break;
        }

        default:
            /* MODE_NONE: 不输出任何力 */
            break;
    }

    /* 安全限幅：限制最大力为 3.0N，防止设备损坏或用户受伤 */
    {
        HDdouble maxForce = 3.0;
        int i;
        for (i = 0; i < 3; i++) {
            if (force[i] > maxForce) force[i] = maxForce;
            if (force[i] < -maxForce) force[i] = -maxForce;
        }
    }

    /* 将计算的力发送到设备 */
    hdSetDoublev(HD_CURRENT_FORCE, force);

    /* 保存当前输出的力，供主线程显示 */
    memcpy(gState.currentForce, force, sizeof(hduVector3Dd));

    /* --- 结束帧：将力命令发送给设备 --- */
    hdEndFrame(hHD);

    /* 检查错误 */
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        gState.error = error;
        if (error.errorCode == HD_COMM_ERROR ||
            error.errorCode == HD_DEVICE_FAULT) {
            /* 严重错误，停止回调 */
            return HD_CALLBACK_DONE;
        }
    }

    /* 继续下一个伺服循环 */
    return HD_CALLBACK_CONTINUE;
}

/* =========================================================================
 * 打印帮助菜单
 * ========================================================================= */
static void printHelp(void) {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║   3D Systems Touch 力反馈笔 - 交互演示程序         ║\n");
    printf("╠══════════════════════════════════════════════════════╣\n");
    printf("║  模式切换:                                         ║\n");
    printf("║   0 - 无力输出 (仅读取传感器数据)                  ║\n");
    printf("║   1 - 重力井 (原点附近吸引力)                      ║\n");
    printf("║   2 - 弹簧 (始终拉向原点)                          ║\n");
    printf("║   3 - 振动 (Y轴正弦波)                             ║\n");
    printf("║   4 - 阻尼 (模拟粘稠液体)                          ║\n");
    printf("║   5 - 虚拟墙 (Z方向平面)                           ║\n");
    printf("║   6 - 虚拟盒子 (六面体约束)                        ║\n");
    printf("║                                                    ║\n");
    printf("║  参数调节:                                         ║\n");
    printf("║   +/- : 增加/减少刚度                              ║\n");
    printf("║   [/] : 增加/减少振动频率                          ║\n");
    printf("║   </> : 减少/增加振动幅值                          ║\n");
    printf("║   d/D : 增加/减少阻尼                              ║\n");
    printf("║                                                    ║\n");
    printf("║  其他:                                             ║\n");
    printf("║   h - 显示此帮助                                   ║\n");
    printf("║   p - 暂停/恢复数据打印                            ║\n");
    printf("║   q - 退出程序                                     ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n");
    printf("\n");
}

/* =========================================================================
 * 打印设备信息
 * ========================================================================= */
static void printDeviceInfo(void) {
    HDdouble maxForce, maxContinuousForce, maxStiffness, maxDamping;
    HDdouble workspace[6];
    HDint inputDOF, outputDOF;

    printf("\n=== 设备信息 ===\n");
    printf("型号:       %s\n", hdGetString(HD_DEVICE_MODEL_TYPE));
    printf("驱动版本:   %s\n", hdGetString(HD_DEVICE_DRIVER_VERSION));
    printf("厂商:       %s\n", hdGetString(HD_DEVICE_VENDOR));
    printf("序列号:     %s\n", hdGetString(HD_DEVICE_SERIAL_NUMBER));
    printf("固件版本:   %s\n", hdGetString(HD_DEVICE_FIRMWARE_VERSION));

    hdGetDoublev(HD_NOMINAL_MAX_FORCE, &maxForce);
    hdGetDoublev(HD_NOMINAL_MAX_CONTINUOUS_FORCE, &maxContinuousForce);
    hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &maxStiffness);
    hdGetDoublev(HD_NOMINAL_MAX_DAMPING, &maxDamping);
    hdGetIntegerv(HD_INPUT_DOF, &inputDOF);
    hdGetIntegerv(HD_OUTPUT_DOF, &outputDOF);
    hdGetDoublev(HD_USABLE_WORKSPACE_DIMENSIONS, workspace);

    printf("输入自由度: %d\n", inputDOF);
    printf("输出自由度: %d\n", outputDOF);
    printf("最大瞬时力: %.2f N\n", maxForce);
    printf("最大连续力: %.2f N\n", maxContinuousForce);
    printf("最大刚度:   %.4f N/mm\n", maxStiffness);
    printf("最大阻尼:   %.4f N/(mm/s)\n", maxDamping);
    printf("工作空间:   X[%.1f, %.1f] Y[%.1f, %.1f] Z[%.1f, %.1f] mm\n",
           workspace[0], workspace[3], workspace[1], workspace[4],
           workspace[2], workspace[5]);
    printf("================\n\n");
}

/* =========================================================================
 * 主函数
 * ========================================================================= */
int main(int argc, char* argv[]) {
    HDErrorInfo error;
    HDSchedulerHandle hServoHandle;
    int printData = 1;              /* 是否打印实时数据 */
    int frameCounter = 0;           /* 用于控制打印频率 */
    const int PRINT_INTERVAL = 30;  /* 每 30 次主循环打印一次（约 ~30Hz） */

    /* --- 注册信号处理，支持 Ctrl+C 优雅退出 --- */
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    /* --- 初始化默认力反馈参数 --- */
    gState.mode = MODE_NONE;
    gState.stiffness = 0.075;       /* 默认刚度 0.075 N/mm */
    gState.damping = 0.002;         /* 默认阻尼 0.002 N·s/mm */
    gState.vibFreq = 5.0;           /* 默认振动频率 5 Hz（低频可感知） */
    gState.vibAmplitude = 0.5;      /* 默认振动幅值 0.5 N */
    gState.wallZ = 0.0;             /* 虚拟墙位于 Z=0 */
    gState.boxSize = 50.0;          /* 虚拟盒子半径 50mm */
    gState.running = HD_TRUE;

    printf("═══════════════════════════════════════════════════\n");
    printf("  3D Systems Touch 力反馈笔 - 综合演示程序\n");
    printf("═══════════════════════════════════════════════════\n");
    printf("正在初始化设备...\n");

    /* --- 初始化触觉设备 --- */
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        fprintf(stderr, "\n[错误] 设备初始化失败!\n");
        fprintf(stderr, "错误代码: 0x%04x\n", error.errorCode);
        fprintf(stderr, "错误描述: %s\n", hdGetErrorString(error.errorCode));
        fprintf(stderr, "\n请检查:\n");
        fprintf(stderr, "  1. 设备是否已连接 (lsusb | grep 2988)\n");
        fprintf(stderr, "  2. 串口权限 (ls -la /dev/ttyACM0)\n");
        fprintf(stderr, "  3. 用户是否在 dialout 组 (groups)\n");
        fprintf(stderr, "  4. GTDD_HOME 环境变量是否设置\n");
        return -1;
    }

    printf("设备初始化成功!\n");

    /* 打印设备详细信息 */
    printDeviceInfo();

    /* --- 检查校准状态 --- */
    {
        int calStyle = hdCheckCalibrationStyle();
        int calStatus = hdCheckCalibration();
        printf("校准风格: ");
        if (calStyle & HD_CALIBRATION_AUTO)
            printf("自动校准 ");
        if (calStyle & HD_CALIBRATION_INKWELL)
            printf("墨水槽校准 ");
        if (calStyle & HD_CALIBRATION_ENCODER_RESET)
            printf("编码器复位 ");
        printf("\n");

        if (calStatus == HD_CALIBRATION_OK) {
            printf("校准状态: OK\n");
        } else if (calStatus == HD_CALIBRATION_NEEDS_UPDATE) {
            printf("校准状态: 需要更新，正在执行自动校准...\n");
            hdUpdateCalibration(calStyle);
        } else {
            printf("校准状态: 需要手动输入\n");
        }
    }

    /* --- 启动伺服回调 --- */
    hServoHandle = hdScheduleAsynchronous(
        servoCallback, NULL, HD_MAX_SCHEDULER_PRIORITY);

    /* 启用力输出 */
    hdEnable(HD_FORCE_OUTPUT);

    /* 启动伺服调度器 */
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        fprintf(stderr, "[错误] 调度器启动失败: %s\n",
                hdGetErrorString(error.errorCode));
        hdDisableDevice(hHD);
        return -1;
    }

    printf("伺服调度器已启动\n");

    /* 设置终端为非阻塞模式 */
    enableRawMode();
    printHelp();

    /* =========== 主循环 =========== */
    while (gState.running) {
        /* 检测键盘输入 */
        int key = kbhit();
        if (key) {
            switch (key) {
                case 'q': case 'Q':
                    /* 退出程序 */
                    gState.running = HD_FALSE;
                    break;

                case '0': case '1': case '2': case '3':
                case '4': case '5': case '6':
                    /* 切换力反馈模式 */
                    gState.mode = (ForceMode)(key - '0');
                    printf("\r\033[K[模式切换] -> %s\n", MODE_NAMES[gState.mode]);
                    break;

                case '+': case '=':
                    /* 增加刚度 */
                    gState.stiffness += 0.01;
                    if (gState.stiffness > 0.5) gState.stiffness = 0.5;
                    printf("\r\033[K[参数] 刚度: %.3f N/mm\n", gState.stiffness);
                    break;

                case '-': case '_':
                    /* 减少刚度 */
                    gState.stiffness -= 0.01;
                    if (gState.stiffness < 0.0) gState.stiffness = 0.0;
                    printf("\r\033[K[参数] 刚度: %.3f N/mm\n", gState.stiffness);
                    break;

                case ']':
                    /* 增加振动频率 */
                    gState.vibFreq += 1.0;
                    if (gState.vibFreq > 200.0) gState.vibFreq = 200.0;
                    printf("\r\033[K[参数] 振动频率: %.1f Hz\n", gState.vibFreq);
                    break;

                case '[':
                    /* 减少振动频率 */
                    gState.vibFreq -= 1.0;
                    if (gState.vibFreq < 1.0) gState.vibFreq = 1.0;
                    printf("\r\033[K[参数] 振动频率: %.1f Hz\n", gState.vibFreq);
                    break;

                case '>': case '.':
                    /* 增加振动幅值 */
                    gState.vibAmplitude += 0.1;
                    if (gState.vibAmplitude > 2.0) gState.vibAmplitude = 2.0;
                    printf("\r\033[K[参数] 振动幅值: %.1f N\n", gState.vibAmplitude);
                    break;

                case '<': case ',':
                    /* 减少振动幅值 */
                    gState.vibAmplitude -= 0.1;
                    if (gState.vibAmplitude < 0.0) gState.vibAmplitude = 0.0;
                    printf("\r\033[K[参数] 振动幅值: %.1f N\n", gState.vibAmplitude);
                    break;

                case 'd':
                    /* 增加阻尼 */
                    gState.damping += 0.001;
                    if (gState.damping > 0.05) gState.damping = 0.05;
                    printf("\r\033[K[参数] 阻尼: %.4f N/(mm/s)\n", gState.damping);
                    break;

                case 'D':
                    /* 减少阻尼 */
                    gState.damping -= 0.001;
                    if (gState.damping < 0.0) gState.damping = 0.0;
                    printf("\r\033[K[参数] 阻尼: %.4f N/(mm/s)\n", gState.damping);
                    break;

                case 'p': case 'P':
                    /* 暂停/恢复数据打印 */
                    printData = !printData;
                    printf("\r\033[K[状态] 数据打印: %s\n",
                           printData ? "开启" : "暂停");
                    break;

                case 'h': case 'H':
                    printHelp();
                    break;
            }
        }

        /* 按固定频率打印实时数据 */
        if (printData && (++frameCounter >= PRINT_INTERVAL)) {
            frameCounter = 0;

            /* 使用 ANSI 转义序列在同一位置更新显示 */
            printf("\r\033[K");
            printf("位置(mm): [%7.2f, %7.2f, %7.2f] | ",
                   gState.position[0], gState.position[1], gState.position[2]);
            printf("力(N): [%6.3f, %6.3f, %6.3f] | ",
                   gState.currentForce[0], gState.currentForce[1], gState.currentForce[2]);
            printf("按钮:%s%s | ",
                   (gState.buttons & HD_DEVICE_BUTTON_1) ? "B1 " : "__ ",
                   (gState.buttons & HD_DEVICE_BUTTON_2) ? "B2" : "__");
            printf("底座:%s | ", gState.inkwellSwitch ? "是" : "否");
            printf("模式:%s | ", MODE_NAMES[gState.mode]);
            printf("%.0fHz", gState.updateRate);
            fflush(stdout);
        }

        /* 检查伺服回调是否异常退出 */
        if (!hdWaitForCompletion(hServoHandle, HD_WAIT_CHECK_STATUS)) {
            fprintf(stderr, "\n[错误] 伺服回调异常退出!\n");
            if (HD_DEVICE_ERROR(gState.error)) {
                fprintf(stderr, "错误: %s\n",
                        hdGetErrorString(gState.error.errorCode));
            }
            break;
        }

        /* 主循环休眠 ~33ms（约 30Hz 刷新率） */
        usleep(33000);
    }

    /* =========== 清理 =========== */
    printf("\n\n正在关闭...\n");

    /* 恢复终端设置 */
    disableRawMode();

    /* 停止调度器和反注册回调 */
    hdStopScheduler();
    hdUnschedule(hServoHandle);

    /* 禁用设备 */
    hdDisableDevice(hHD);

    printf("设备已安全关闭。再见!\n");
    return 0;
}
