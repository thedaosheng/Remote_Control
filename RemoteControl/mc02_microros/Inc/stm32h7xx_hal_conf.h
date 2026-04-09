/**
 * @file stm32h7xx_hal_conf.h
 * @brief STM32H7 HAL 库配置文件
 *
 * 本文件定义了需要启用的 HAL 模块。
 * 达妙 MC02 使用 STM32H723VGT6，外部晶振 25MHz。
 */

#ifndef __STM32H7xx_HAL_CONF_H
#define __STM32H7xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= 模块选择 ========================= */
/* 启用需要的 HAL 模块，注释掉不需要的以减小固件体积 */

#define HAL_MODULE_ENABLED          /* HAL 核心（必须）*/
#define HAL_CORTEX_MODULE_ENABLED   /* Cortex-M7 系统配置 */
#define HAL_DMA_MODULE_ENABLED      /* DMA 控制器 */
#define HAL_FLASH_MODULE_ENABLED    /* Flash 编程 */
#define HAL_GPIO_MODULE_ENABLED     /* GPIO 控制（LED 翻转需要）*/
#define HAL_PWR_MODULE_ENABLED      /* 电源管理 */
#define HAL_RCC_MODULE_ENABLED      /* 时钟配置（必须）*/
#define HAL_UART_MODULE_ENABLED     /* UART（micro-ROS transport 需要）*/
#define HAL_TIM_MODULE_ENABLED      /* 定时器（FreeRTOS 心跳需要）*/
#define HAL_EXTI_MODULE_ENABLED     /* 外部中断 */
#define HAL_FDCAN_MODULE_ENABLED    /* FDCAN（达妙电机 CAN 总线）*/
#define HAL_PCD_MODULE_ENABLED      /* USB Device (PCD) — CDC 需要 */
#define HAL_HCD_MODULE_ENABLED      /* USB Host (备用) */

/* ========================= 时钟参数 ========================= */

/* 外部高速晶振频率 - MC02 使用 25MHz */
#if !defined(HSE_VALUE)
#define HSE_VALUE    ((uint32_t)25000000U)
#endif

/* HSE 启动超时（ms）*/
#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT    ((uint32_t)100U)
#endif

/* 内部高速 RC 振荡器 64MHz */
#if !defined(HSI_VALUE)
#define HSI_VALUE    ((uint32_t)64000000U)
#endif

/* 内部低速 RC 振荡器 32kHz */
#if !defined(LSI_VALUE)
#define LSI_VALUE    ((uint32_t)32000U)
#endif

/* 外部低速晶振 32.768kHz */
#if !defined(LSE_VALUE)
#define LSE_VALUE    ((uint32_t)32768U)
#endif

#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT    ((uint32_t)5000U)
#endif

/* HSI48 RC 振荡器 48MHz（USB 时钟源）*/
#if !defined(HSI48_VALUE)
#define HSI48_VALUE  ((uint32_t)48000000U)
#endif

/* CSI RC 振荡器 4MHz */
#if !defined(CSI_VALUE)
#define CSI_VALUE    ((uint32_t)4000000U)
#endif

/* 外部时钟源（I2S 等）*/
#if !defined(EXTERNAL_CLOCK_VALUE)
#define EXTERNAL_CLOCK_VALUE    12288000U
#endif

/* ========================= 系统配置 ========================= */

/* SysTick 优先级 —— FreeRTOS 要求最低优先级 */
#define TICK_INT_PRIORITY            ((uint32_t)15U)

/* 使用 FreeRTOS 时，HAL 的 tick 来源切换为 TIM6 */
#define USE_RTOS                     0U
#define PREFETCH_ENABLE              1U
#define ART_ACCELERATOR_ENABLE       1U

/* ========================= 以太网（未使用，保留默认值）========================= */
#define ETH_TX_DESC_CNT         4U
#define ETH_RX_DESC_CNT         4U
#define ETH_MAC_ADDR0           ((uint8_t)0x02)
#define ETH_MAC_ADDR1           ((uint8_t)0x00)
#define ETH_MAC_ADDR2           ((uint8_t)0x00)
#define ETH_MAC_ADDR3           ((uint8_t)0x00)
#define ETH_MAC_ADDR4           ((uint8_t)0x00)
#define ETH_MAC_ADDR5           ((uint8_t)0x00)

/* ========================= VDD 电压 ========================= */
#define VDD_VALUE                    ((uint32_t)3300U)   /* 3.3V，单位 mV */

/* ========================= HAL 头文件包含 ========================= */

#ifdef HAL_MODULE_ENABLED
  #include "stm32h7xx_hal.h"
#endif

#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32h7xx_hal_rcc.h"
#endif

#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32h7xx_hal_gpio.h"
#endif

#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32h7xx_hal_dma.h"
#endif

#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32h7xx_hal_cortex.h"
#endif

#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32h7xx_hal_flash.h"
#endif

#ifdef HAL_PWR_MODULE_ENABLED
  #include "stm32h7xx_hal_pwr.h"
#endif

#ifdef HAL_UART_MODULE_ENABLED
  #include "stm32h7xx_hal_uart.h"
#endif

#ifdef HAL_TIM_MODULE_ENABLED
  #include "stm32h7xx_hal_tim.h"
#endif

#ifdef HAL_EXTI_MODULE_ENABLED
  #include "stm32h7xx_hal_exti.h"
#endif

#ifdef HAL_FDCAN_MODULE_ENABLED
  #include "stm32h7xx_hal_fdcan.h"
#endif

#ifdef HAL_PCD_MODULE_ENABLED
  #include "stm32h7xx_hal_pcd.h"
  #include "stm32h7xx_hal_pcd_ex.h"
  #include "stm32h7xx_ll_usb.h"
#endif

/* ========================= 断言配置 ========================= */
/* 如需调试 HAL 参数错误，取消下面注释 */
/* #define USE_FULL_ASSERT    1U */

#ifdef USE_FULL_ASSERT
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
  void assert_failed(uint8_t *file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __STM32H7xx_HAL_CONF_H */
