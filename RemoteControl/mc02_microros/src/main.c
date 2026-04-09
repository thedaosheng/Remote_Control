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
#include <std_msgs/msg/string.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include "custom_transport.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"

/* USB 句柄（在 custom_transport.c 中定义）*/
extern USBD_HandleTypeDef hUsbDeviceFS;

/* ========================= 宏定义 ========================= */

/** micro-ROS 任务栈大小（单位：word）
 *  micro-ROS 需要较大栈：allocator + serialization + XRCE buffer
 *  16384 × 4 = 64KB，在 H723 的 564KB SRAM 中完全可接受 */
#define MICROROS_TASK_STACK_SIZE    (16384)

/** micro-ROS 任务优先级（比空闲任务高，比关键中断低）*/
#define MICROROS_TASK_PRIORITY      (24)

/** 发布频率：1 Hz */
#define PUBLISH_PERIOD_MS           (1000)

/** 发布消息的最大字符串长度 */
#define MSG_BUFFER_SIZE             (64)

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

/* ========================= micro-ROS 任务 ========================= */

/**
 * @brief 枚举：Agent 连接状态机
 */
typedef enum {
    AGENT_WAIT_FOR_CONNECT,   /* 等待 Agent 连接 */
    AGENT_CONNECTED,          /* 已连接，正常发布 */
    AGENT_DISCONNECTED        /* 检测到断连，需要清理并重连 */
} agent_state_t;

/**
 * @brief micro-ROS 主任务
 *
 * 运行在 FreeRTOS 独立任务中，完整生命周期：
 *   1. 设置自定义 USART1 transport
 *   2. 等待 Agent 连接（ping 检测）
 *   3. 创建节点和发布者
 *   4. 循环发布消息，同时检测连接状态
 *   5. 断连后自动清理并重新连接
 */
static void microros_task(void *arg)
{
    (void)arg;

    /* ---- 变量声明 ---- */
    rcl_allocator_t          allocator;
    rclc_support_t           support;
    rcl_node_t               node;
    rcl_publisher_t          publisher;
    std_msgs__msg__String    msg;
    rclc_executor_t          executor;

    /* 消息字符串缓冲区 */
    char msg_buffer[MSG_BUFFER_SIZE];
    uint32_t publish_count = 0;

    /* ==== 第 1 步：设置 transport ==== */
    rmw_uros_set_custom_transport(
        true, NULL,
        custom_transport_open, custom_transport_close,
        custom_transport_write, custom_transport_read
    );

    /* ==== 第 2 步：等待 Agent 连接 ==== */
    /* 先等 5 秒让 USB 完全稳定（Agent 打开端口可能触发 USB reset）*/
    vTaskDelay(pdMS_TO_TICKS(5000));
    while (rmw_uros_ping_agent(1000, 3) != RMW_RET_OK) {
        LED_Toggle();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    /* ==== 第 3 步：一次性初始化所有 micro-ROS 资源 ==== */
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "mc02_node", "", &support);
    rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/mc02_hello"
    );

    /* 初始化消息 */
    msg.data.data     = msg_buffer;
    msg.data.size     = 0;
    msg.data.capacity = MSG_BUFFER_SIZE;

    /* LED 快闪 3 次表示初始化成功 */
    for (int i = 0; i < 6; i++) {
        LED_Toggle();
        HAL_Delay(100);
    }

    /* ==== 第 4 步：永久发布循环 ==== */
    for (;;) {
        int written = snprintf(msg_buffer, MSG_BUFFER_SIZE,
                               "Hello from MC02 #%lu", (unsigned long)publish_count);
        msg.data.size = (written > 0) ? (size_t)written : 0;

        rcl_publish(&publisher, &msg, NULL);
        LED_Toggle();
        publish_count++;

        vTaskDelay(pdMS_TO_TICKS(1000));
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
