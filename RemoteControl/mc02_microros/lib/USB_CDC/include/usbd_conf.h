/**
 * @file usbd_conf.h
 * @brief USB Device 底层配置头文件 - MC02
 *
 * 定义 USB Device Library 所需的配置参数和内存分配宏。
 */

#ifndef __USBD_CONF_H
#define __USBD_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ========================= USB 配置参数 ========================= */

/** USB 最大端点数（EP0 + 数据 IN/OUT + 命令）*/
#define USBD_MAX_NUM_INTERFACES     1U
#define USBD_MAX_NUM_CONFIGURATION  1U
#define USBD_MAX_STR_DESC_SIZ       512U
#define USBD_MAX_CLASS_ENDPOINTS    4U
#define USBD_MAX_CLASS_INTERFACES   1U

/** 支持自供电 */
#define USBD_SELF_POWERED           1U

/** 调试输出（生产环境可关闭）*/
#define USBD_DEBUG_LEVEL            0U

/** USB 内存分配 - 使用标准 malloc/free */
#define USBD_malloc   malloc
#define USBD_free     free
#define USBD_memset   memset
#define USBD_memcpy   memcpy

/** 日志宏 */
#if (USBD_DEBUG_LEVEL > 0)
#define USBD_UsrLog(...)    printf(__VA_ARGS__); printf("\n");
#else
#define USBD_UsrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 1)
#define USBD_ErrLog(...)    printf("ERROR: "); printf(__VA_ARGS__); printf("\n");
#else
#define USBD_ErrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 2)
#define USBD_DbgLog(...)    printf("DEBUG: "); printf(__VA_ARGS__); printf("\n");
#else
#define USBD_DbgLog(...)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CONF_H */
