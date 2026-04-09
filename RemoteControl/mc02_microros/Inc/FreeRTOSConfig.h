/**
 * @file FreeRTOSConfig.h
 * @brief FreeRTOS 配置文件 - 达妙 MC02 (STM32H723VGT6)
 *
 * 针对 micro-ROS 优化的 FreeRTOS 配置：
 * - 较大的任务栈（micro-ROS 需要 >= 16KB）
 * - 动态内存分配（heap_4 方案）
 * - 软件定时器支持
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* ========================= 核心配置 ========================= */

/* 使用抢占式调度 */
#define configUSE_PREEMPTION                    1
/* CPU 时钟频率 550MHz */
#define configCPU_CLOCK_HZ                      (550000000UL)
/* FreeRTOS 心跳频率 1kHz（1ms tick）*/
#define configTICK_RATE_HZ                      ((TickType_t)1000)
/* 最大优先级数（0 = 最低，≤32 以支持 optimized task selection）*/
#define configMAX_PRIORITIES                    (32)
/* 空闲任务栈大小（单位：word，128×4=512 字节）*/
#define configMINIMAL_STACK_SIZE                ((uint16_t)128)
/* FreeRTOS 总堆大小 96KB —— 平衡 FreeRTOS 任务栈和 _sbrk 堆
 * AXI SRAM 320KB: 96KB FreeRTOS + ~224KB for _sbrk (micro-ROS malloc) */
#define configTOTAL_HEAP_SIZE                   ((size_t)(96 * 1024))
/* 任务名最大长度 */
#define configMAX_TASK_NAME_LEN                 (16)
/* 使用 32 位 tick 计数器 */
#define configUSE_16_BIT_TICKS                  0
/* 允许同优先级任务使用空闲时间 */
#define configIDLE_SHOULD_YIELD                 1
/* 启用互斥锁（micro-ROS 内部使用）*/
#define configUSE_MUTEXES                       1
/* 启用递归互斥锁 */
#define configUSE_RECURSIVE_MUTEXES             1
/* 启用计数信号量 */
#define configUSE_COUNTING_SEMAPHORES           1
/* 队列注册表大小（调试用）*/
#define configQUEUE_REGISTRY_SIZE               8

/* ========================= 内存分配 ========================= */

/* 支持静态和动态创建 */
#define configSUPPORT_STATIC_ALLOCATION         1
#define configSUPPORT_DYNAMIC_ALLOCATION        1

/* ========================= Hook 函数 ========================= */

/* 空闲任务 hook - 可用于低功耗 */
#define configUSE_IDLE_HOOK                     0
/* tick hook */
#define configUSE_TICK_HOOK                     0
/* 栈溢出检测暂时关闭（调试 USB 稳定性）*/
#define configCHECK_FOR_STACK_OVERFLOW          0
/* malloc 失败 hook */
#define configUSE_MALLOC_FAILED_HOOK            0

/* ========================= 软件定时器 ========================= */

#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               (2)
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            (configMINIMAL_STACK_SIZE * 2)

/* ========================= 中断优先级配置 ========================= */

/* Cortex-M7 使用 4 位优先级 */
#ifdef __NVIC_PRIO_BITS
  #define configPRIO_BITS         __NVIC_PRIO_BITS
#else
  #define configPRIO_BITS         4
#endif

/* FreeRTOS 可管理的最低中断优先级（数值越大优先级越低）*/
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15
/* FreeRTOS API 安全调用的最高中断优先级 */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5

/* 转换为 NVIC 硬件优先级值（左移到高位）*/
#define configKERNEL_INTERRUPT_PRIORITY         (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* 阻止 FreeRTOS port 重新配置 SysTick 定时器
 * 我们在 SysTick_Handler 中同时服务 HAL 和 FreeRTOS */
#define configOVERRIDE_DEFAULT_TICK_CONFIGURATION  1

/* ========================= 包含的 API 函数 ========================= */

#define INCLUDE_vTaskPrioritySet            1
#define INCLUDE_uxTaskPriorityGet           1
#define INCLUDE_vTaskDelete                 1
#define INCLUDE_vTaskCleanUpResources       0
#define INCLUDE_vTaskSuspend                1
#define INCLUDE_vTaskDelayUntil             1
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_xTaskGetSchedulerState      1

/* ========================= Cortex-M 特定定义 ========================= */

/* 将 FreeRTOS 中断处理映射到 CMSIS 名称 */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler

/* 注意：SysTick_Handler 不在此映射，因为我们用 HAL 的 TIM 做 FreeRTOS tick */

#endif /* FREERTOS_CONFIG_H */
