/**
 * @file main.c
 * @brief 达妙 MC02 micro-ROS 主程序
 *
 * 功能：
 *   1. 初始化 STM32H723 时钟树（HSE 25MHz → PLL → 550MHz）
 *   2. 启动 FreeRTOS，创建 micro-ROS 任务
 *   3. micro-ROS 任务中：
 *      - 通过 USART1 连接 micro-ROS Agent
 *      - 创建节点 "mc02_node"
 *      - 每秒发布 std_msgs/String 到 "/mc02_hello"
 *      - 每次发布翻转板载 LED（PC13）
 *      - Agent 断连时自动重连
 *
 * 硬件：
 *   - MCU:  STM32H723VGT6 @ 550MHz
 *   - LED:  PC13（达妙 MC02 板载 LED）
 *   - UART: USART1 PA9(TX)/PA10(RX) 115200 8N1
 */

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include "custom_transport.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"

/* USB 句柄（在 custom_transport.c 中定义）*/
extern USBD_HandleTypeDef hUsbDeviceFS;

/* 达妙电机 CAN 驱动 */
#include "dm_can.h"

/* ========================= 宏定义 ========================= */

/** micro-ROS 任务栈大小（4096 words = 16KB）*/
#define MICROROS_TASK_STACK_SIZE    (4096)

/** micro-ROS 任务优先级 */
#define MICROROS_TASK_PRIORITY      (24)

/** 反馈发布周期 10ms (100Hz) */
#define FEEDBACK_PERIOD_MS          (10)

/** 指令超时保护 500ms */
#define CMD_TIMEOUT_MS              (500)

/** 舵向限幅 ±π rad */
#define STEER_LIMIT                 (3.14159265f)

/** 驱动限幅 ±25 rad/s */
#define DRIVE_LIMIT                 (25.0f)

/** 电机数量 */
#define NUM_STEER_MOTORS            4
#define NUM_DRIVE_MOTORS            4
#define NUM_MOTORS                  8

/** LED 引脚定义 —— MC02 板载 LED 在 PC13 */
#define LED_PORT    GPIOC
#define LED_PIN     GPIO_PIN_13

/* ========================= 前向声明 ========================= */

static void SystemClock_Config(void);
static void LED_Init(void);
static void LED_Toggle(void);
static void microros_task(void *arg);

/* ========================= FreeRTOS 静态分配支持 ========================= */

/**
 * FreeRTOS 要求 configSUPPORT_STATIC_ALLOCATION=1 时
 * 提供空闲任务和定时器任务的内存。
 */
static StaticTask_t xIdleTaskTCB;
static StackType_t  xIdleTaskStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = xIdleTaskStack;
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

static StaticTask_t xTimerTaskTCB;
static StackType_t  xTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer   = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = xTimerTaskStack;
    *pulTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
}

/**
 * @brief 栈溢出检测 hook
 *
 * 如果任何任务栈溢出，此函数被调用。
 * 生产环境应记录错误并复位；这里简单死循环便于调试。
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    /* 栈溢出！在调试器中检查 pcTaskName 确定是哪个任务 */
    for (;;) {
        __NOP();
    }
}

/* ========================= 时钟配置 ========================= */

/**
 * @brief 配置 STM32H723 时钟树
 *
 * 时钟路径：
 *   HSE 25MHz → PLL1 → SYSCLK 550MHz
 *   PLL1: M=5, N=110, P=1 → 25/5*110/1 = 550MHz
 *
 * AHB = 550MHz / 2 = 275MHz
 * APB1 = 275MHz / 2 = 137.5MHz（定时器 x2 = 275MHz）
 * APB2 = 275MHz / 2 = 137.5MHz（USART1 时钟源）
 *
 * Flash 等待周期：根据 H723 数据手册，550MHz 需要 5WS
 */
static void SystemClock_Config(void)
{
    /* H7 系列 PWR 时钟默认使能，无需手动调用 */

    /* H723 需要设置为 VOS0（最高性能模式）才能跑 550MHz */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /* 配置 PLL1 + HSI48（USB 48MHz 时钟源）*/
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI48;
    osc.HSEState       = RCC_HSE_ON;           /* 开启外部 25MHz 晶振 */
    osc.HSI48State     = RCC_HSI48_ON;         /* 开启 HSI48 for USB */
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = 5;                    /* 25MHz / 5 = 5MHz (PLL 输入) */
    osc.PLL.PLLN       = 110;                  /* 5MHz × 110 = 550MHz (VCO) */
    osc.PLL.PLLP       = 1;                    /* 550MHz / 1 = 550MHz (SYSCLK) */
    osc.PLL.PLLQ       = 4;                    /* 550MHz / 4 = 137.5MHz (USB/SDMMC 等) */
    osc.PLL.PLLR       = 2;                    /* 550MHz / 2 = 275MHz (备用) */
    osc.PLL.PLLRGE     = RCC_PLL1VCIRANGE_2;   /* PLL 输入 4-8MHz 范围 */
    osc.PLL.PLLVCOSEL  = RCC_PLL1VCOWIDE;      /* 宽 VCO 范围 (192-960MHz) */
    osc.PLL.PLLFRACN   = 0;                    /* 不使用分数 PLL */
    HAL_RCC_OscConfig(&osc);

    /* 配置总线时钟分频 */
    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                          RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2 |
                          RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_D3PCLK1;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;  /* SYSCLK = PLL = 550MHz */
    clk.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    clk.AHBCLKDivider  = RCC_HCLK_DIV2;            /* AHB = 275MHz */
    clk.APB1CLKDivider = RCC_APB1_DIV2;             /* APB1 = 137.5MHz */
    clk.APB2CLKDivider = RCC_APB2_DIV2;             /* APB2 = 137.5MHz */
    clk.APB3CLKDivider = RCC_APB3_DIV2;             /* APB3 = 137.5MHz */
    clk.APB4CLKDivider = RCC_APB4_DIV2;             /* APB4 = 137.5MHz */

    /* Flash 5 等待周期 @ 550MHz */
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5);

    /* 更新 SystemCoreClock 全局变量 */
    SystemCoreClockUpdate();

    /* 配置 USB 时钟源为 HSI48（48MHz）*/
    RCC_PeriphCLKInitTypeDef pclk = {0};
    pclk.PeriphClockSelection = RCC_PERIPHCLK_USB;
    pclk.UsbClockSelection    = RCC_USBCLKSOURCE_HSI48;
    HAL_RCCEx_PeriphCLKConfig(&pclk);
}

/* ========================= LED 控制 ========================= */

/**
 * @brief 初始化板载 LED GPIO（PC13 推挽输出）
 */
static void LED_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = LED_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;   /* 推挽输出 */
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;   /* LED 不需要高速 */
    HAL_GPIO_Init(LED_PORT, &gpio);

    /* 初始状态：LED 灭（MC02 上 PC13 低电平亮，高电平灭）*/
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

/**
 * @brief 翻转 LED 状态
 */
static void LED_Toggle(void)
{
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
}

/* ========================= 电机配置 ========================= */

/**
 * 舵轮底盘 8 电机配置：
 *   [0-3] 舵向 (POS_VEL): FL=1, FR=2, RL=3, RR=4
 *   [4-7] 驱动 (VEL):     FL=5, FR=6, RL=7, RR=8
 * MasterID = SlaveID + 0x10（反馈帧识别用）
 * TODO: 根据实际接线修改 SlaveID
 */
static dm_motor_t motors[NUM_MOTORS] = {
    { .slave_id = 1, .master_id = 0x11, .type = DM4310_PARAMS },
    { .slave_id = 2, .master_id = 0x12, .type = DM4310_PARAMS },
    { .slave_id = 3, .master_id = 0x13, .type = DM4310_PARAMS },
    { .slave_id = 4, .master_id = 0x14, .type = DM4310_PARAMS },
    { .slave_id = 5, .master_id = 0x15, .type = DM4310_PARAMS },
    { .slave_id = 6, .master_id = 0x16, .type = DM4310_PARAMS },
    { .slave_id = 7, .master_id = 0x17, .type = DM4310_PARAMS },
    { .slave_id = 8, .master_id = 0x18, .type = DM4310_PARAMS },
};

/* ========================= 指令缓冲 + 看门狗 ========================= */

/** 最新指令（由订阅回调写入，主循环读取）*/
static volatile double   cmd_data[NUM_MOTORS] = {0};
static volatile uint32_t cmd_last_tick = 0;

/**
 * @brief /mujoco/swerve_cmd 订阅回调
 *
 * 接收 8 double: [舵向FL,FR,RL,RR, 驱动FL,FR,RL,RR]
 * 限幅后存入 cmd_data，更新看门狗时间戳。
 */
static void swerve_cmd_callback(const void *msgin)
{
    const std_msgs__msg__Float64MultiArray *msg =
        (const std_msgs__msg__Float64MultiArray *)msgin;
    if (msg->data.size < NUM_MOTORS) return;

    for (int i = 0; i < NUM_STEER_MOTORS; i++) {
        double v = msg->data.data[i];
        if (v >  STEER_LIMIT) v =  STEER_LIMIT;
        if (v < -STEER_LIMIT) v = -STEER_LIMIT;
        cmd_data[i] = v;
    }
    for (int i = NUM_STEER_MOTORS; i < NUM_MOTORS; i++) {
        double v = msg->data.data[i];
        if (v >  DRIVE_LIMIT) v =  DRIVE_LIMIT;
        if (v < -DRIVE_LIMIT) v = -DRIVE_LIMIT;
        cmd_data[i] = v;
    }
    cmd_last_tick = HAL_GetTick();
}

/* ========================= micro-ROS 任务 ========================= */

/**
 * @brief micro-ROS 舵轮底盘控制任务
 *
 * 订阅 /mujoco/swerve_cmd → 限幅 → FDCAN 发电机指令
 * 读取电机反馈 → 发布 /mc02/motor_feedback (100Hz)
 * 500ms 无指令 → 驱动速度归零（安全看门狗）
 */
static void microros_task(void *arg)
{
    (void)arg;

    rcl_allocator_t       allocator;
    rclc_support_t        support;
    rcl_node_t            node;
    rcl_subscription_t    sub_cmd;
    rcl_publisher_t       pub_fb;
    rclc_executor_t       executor;

    /* 订阅消息缓冲区 */
    std_msgs__msg__Float64MultiArray msg_cmd;
    double cmd_buf[NUM_MOTORS];
    msg_cmd.data.data     = cmd_buf;
    msg_cmd.data.size     = 0;
    msg_cmd.data.capacity = NUM_MOTORS;

    /* 反馈消息缓冲区 */
    std_msgs__msg__Float64MultiArray msg_fb;
    double fb_buf[NUM_MOTORS];
    msg_fb.data.data     = fb_buf;
    msg_fb.data.size     = NUM_MOTORS;
    msg_fb.data.capacity = NUM_MOTORS;

    /* ==== 1. 初始化 FDCAN + 注册电机 ==== */
    /* TODO: dm_can_init() 暂时跳过——需确认 MC02 的 FDCAN 引脚后再启用 */
    /* dm_can_init(); */
    for (int i = 0; i < NUM_MOTORS; i++) {
        dm_motor_register(&motors[i]);
    }

    /* ==== 2. 设置 micro-ROS transport (USB CDC) ==== */
    rmw_uros_set_custom_transport(
        true, NULL,
        custom_transport_open, custom_transport_close,
        custom_transport_write, custom_transport_read
    );

    /* ==== 3. 等待 Agent 连接 ==== */
    vTaskDelay(pdMS_TO_TICKS(5000));
    while (rmw_uros_ping_agent(1000, 3) != RMW_RET_OK) {
        LED_Toggle();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    /* ==== 4. 创建节点 + 订阅 + 发布 ==== */
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "mc02_node", "", &support);

    rclc_subscription_init_default(
        &sub_cmd, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "/mujoco/swerve_cmd"
    );

    rclc_publisher_init_default(
        &pub_fb, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "/mc02/motor_feedback"
    );

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd,
                                    swerve_cmd_callback, ON_NEW_DATA);

    /* LED 快闪表示就绪 */
    for (int i = 0; i < 6; i++) { LED_Toggle(); HAL_Delay(100); }

    /* ==== 5. 使能电机（CAN 未接时跳过）==== */
    /* TODO: CAN 硬件就绪后取消注释
    for (int i = 0; i < NUM_MOTORS; i++) {
        dm_motor_enable(&motors[i]);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    */

    /* ==== 6. 主控制循环 (100Hz) ==== */
    uint32_t loop_count = 0;

    for (;;) {
        /* 处理 micro-ROS 订阅回调 */
        rclc_executor_spin_some(&executor, 0);

        /* 看门狗判断 */
        uint32_t now = HAL_GetTick();
        bool timed_out = (cmd_last_tick == 0) || ((now - cmd_last_tick) > CMD_TIMEOUT_MS);

        /* 发送电机指令（CAN 未接时用 mock 模式）*/
        if (!timed_out) {
            /* mock: 将指令直接复制到反馈（模拟电机跟随）*/
            for (int i = 0; i < NUM_STEER_MOTORS; i++)
                motors[i].fb_position = (float)cmd_data[i];
            for (int i = NUM_STEER_MOTORS; i < NUM_MOTORS; i++)
                motors[i].fb_velocity = (float)cmd_data[i];
        }

        /* TODO: CAN 硬件就绪后替换为实际 CAN 通信：
        if (timed_out) {
            for (int i = NUM_STEER_MOTORS; i < NUM_MOTORS; i++)
                dm_motor_vel(&motors[i], 0.0f);
        } else {
            for (int i = 0; i < NUM_STEER_MOTORS; i++)
                dm_motor_pos_vel(&motors[i], (float)cmd_data[i], 10.0f);
            for (int i = NUM_STEER_MOTORS; i < NUM_MOTORS; i++)
                dm_motor_vel(&motors[i], (float)cmd_data[i]);
        }
        dm_can_process_rx();
        */

        /* 每 10 次（100ms）发布反馈 */
        if (loop_count % 10 == 0) {
            for (int i = 0; i < NUM_STEER_MOTORS; i++)
                fb_buf[i] = (double)motors[i].fb_position;
            for (int i = NUM_STEER_MOTORS; i < NUM_MOTORS; i++)
                fb_buf[i] = (double)motors[i].fb_velocity;
            rcl_publish(&pub_fb, &msg_fb, NULL);
            LED_Toggle();
        }

        loop_count++;
        vTaskDelay(pdMS_TO_TICKS(FEEDBACK_PERIOD_MS));
    }
}

/* ========================= 入口点 ========================= */

/**
 * @brief 程序入口
 *
 * 流程：
 *   1. HAL 初始化（SysTick, Flash, 电源）
 *   2. 配置 550MHz 时钟树
 *   3. 初始化 LED
 *   4. 创建 micro-ROS FreeRTOS 任务
 *   5. 启动调度器（不再返回）
 */
int main(void)
{
    /* HAL 初始化：配置 Flash prefetch, 中断优先级分组, SysTick */
    HAL_Init();

    /* 配置时钟树：HSE 25MHz → PLL → 550MHz */
    SystemClock_Config();

    /* 初始化板载 LED（PC13）*/
    LED_Init();

    /* LED 亮一下表示系统启动 */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

    /* 初始化 USB CDC */
    USBD_Init(&hUsbDeviceFS, &FS_Desc, 0);
    CDC_Set_USBD_Handle(&hUsbDeviceFS);
    USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);
    USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);
    USBD_Start(&hUsbDeviceFS);

    /* 等待 USB 枚举完成（Host 需要时间识别设备）*/
    HAL_Delay(2000);

    /* 创建 micro-ROS 任务
     * 栈大小 4096 words = 16KB（64KB heap 限制）*/
    xTaskCreate(
        microros_task,
        "uros",
        4096,
        NULL,
        MICROROS_TASK_PRIORITY,
        NULL
    );

    /* 启动 FreeRTOS 调度器 —— 此函数不会返回 */
    vTaskStartScheduler();

    /* 理论上不应执行到这里；如果到了说明堆内存不足 */
    for (;;) {
        __NOP();
    }
}

/* ========================= 中断处理（HAL 需要）========================= */

/**
 * @brief SysTick 中断处理
 *
 * HAL 和 FreeRTOS 都需要 SysTick。
 * HAL_IncTick() 维护 HAL_GetTick() 时基。
 * FreeRTOS 的 xPortSysTickHandler 处理任务调度。
 */
/**
 * @brief HardFault 中断处理 - 调试用
 * 如果触发，LED 快速闪烁表示 HardFault
 */
void HardFault_Handler(void)
{
    /* 尝试用 USB 发 HardFault 信号（可能不工作）*/
    uint8_t msg[] = "!!! HARDFAULT !!!\r\n";
    CDC_Transmit_FS(msg, sizeof(msg) - 1);

    /* 快速闪烁 LED（可视化 HardFault）*/
    for (;;) {
        GPIOC->ODR ^= GPIO_PIN_13;
        for (volatile uint32_t i = 0; i < 1000000; i++) {}
    }
}

void MemManage_Handler(void)
{
    for (;;) {
        GPIOC->ODR ^= GPIO_PIN_13;
        for (volatile uint32_t i = 0; i < 500000; i++) {}
    }
}

void BusFault_Handler(void)
{
    for (;;) {
        GPIOC->ODR ^= GPIO_PIN_13;
        for (volatile uint32_t i = 0; i < 300000; i++) {}
    }
}

void UsageFault_Handler(void)
{
    for (;;) {
        GPIOC->ODR ^= GPIO_PIN_13;
        for (volatile uint32_t i = 0; i < 200000; i++) {}
    }
}

void SysTick_Handler(void)
{
    HAL_IncTick();

    /* 仅在调度器运行后才调用 FreeRTOS tick 处理 */
    extern void xPortSysTickHandler(void);
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}
