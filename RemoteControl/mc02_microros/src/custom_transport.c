/**
 * @file custom_transport.c
 * @brief micro-ROS 自定义 USB CDC 传输层实现
 *
 * 基于 STM32H723 USB OTG FS + CDC 虚拟串口实现 micro-ROS 传输。
 * Type-C 接口 → PA11(DM)/PA12(DP) → USB CDC → 环形缓冲区。
 *
 * 替代之前的 USART1 方案，无需额外 USB-TTL 模块，
 * 直接通过 MC02 的 Type-C 口与 PC 通信。
 *
 * 4 个必须实现的回调：open / close / write / read
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "stm32h7xx_hal.h"
#include <uxr/client/profile/transport/custom/custom_transport.h>

/* USB CDC 头文件 */
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"

/* ========================= 全局 USB 句柄 ========================= */

/** USBD 句柄 - USB Device 核心 */
USBD_HandleTypeDef hUsbDeviceFS;

/* ========================= Transport 回调实现 ========================= */

/**
 * @brief 打开 transport - 初始化 USB CDC
 *
 * 完整的 USB 设备初始化流程：
 *   1. USBD_Init — 初始化 USB Device Core + 底层硬件
 *   2. USBD_RegisterClass — 注册 CDC 类
 *   3. USBD_CDC_RegisterInterface — 注册 CDC 接口回调
 *   4. USBD_Start — 启动 USB，开始枚举
 *
 * 调用后 Host（PC）会看到一个新的虚拟串口设备。
 */
bool custom_transport_open(struct uxrCustomTransport *transport)
{
    (void)transport;

    /* USB CDC 已在 main() 中初始化并启动。
     * 这里只需确认 USB 设备已配置且 CDC 类数据可用。 */
    if (hUsbDeviceFS.pClassData == NULL) {
        /* USB 尚未就绪，等待 */
        HAL_Delay(500);
    }

    return (hUsbDeviceFS.pClassData != NULL);
}

/**
 * @brief 关闭 transport - 停止 USB
 */
bool custom_transport_close(struct uxrCustomTransport *transport)
{
    (void)transport;

    /* 不关闭 USB！USB CDC 是全局资源，在 main() 中初始化，
     * 必须始终保持运行。transport close 只做逻辑清理。 */

    return true;
}

/**
 * @brief 发送数据到 PC（micro-ROS Agent）
 *
 * 通过 USB CDC 发送。USB FS 每包最大 64 字节，
 * 所以大数据需要分包发送。
 *
 * @param buf 待发送数据
 * @param len 数据长度
 * @param err 错误码（0=成功）
 * @return 成功时返回 len，失败返回 0
 */
size_t custom_transport_write(struct uxrCustomTransport *transport,
                               const uint8_t *buf, size_t len, uint8_t *err)
{
    (void)transport;

    /* 分包发送，每包最大 64 字节 */
    size_t sent = 0;
    while (sent < len) {
        /* 计算本次发送大小 */
        uint16_t chunk = (uint16_t)((len - sent) > 64 ? 64 : (len - sent));

        /* 等待上一包发送完成（带超时保护，缩短为 50ms 避免阻塞 USB SOF）*/
        uint32_t timeout_start = HAL_GetTick();
        while (CDC_IsTxBusy()) {
            if ((HAL_GetTick() - timeout_start) > 50) {
                *err = 1;
                return sent;  /* 超时，返回已发送的 */
            }
        }

        /* 发送数据包 */
        uint8_t result = CDC_Transmit_FS((uint8_t *)(buf + sent), chunk);
        if (result == USBD_OK) {
            sent += chunk;
        } else if (result == USBD_BUSY) {
            /* 忙等，不移动 sent 指针 */
            continue;
        } else {
            *err = 1;
            return sent;
        }
    }

    *err = 0;
    return len;
}

/**
 * @brief 从 PC 接收数据（micro-ROS Agent → MCU）
 *
 * 从 CDC 环形缓冲区读取数据，带超时控制。
 * USB 接收是中断驱动的，数据会异步写入环形缓冲区。
 *
 * @param buf 接收缓冲区
 * @param len 缓冲区大小
 * @param timeout 总超时（ms）
 * @param err 错误码（0=成功）
 * @return 实际接收到的字节数
 */
size_t custom_transport_read(struct uxrCustomTransport *transport,
                              uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    (void)transport;

    uint32_t start_tick = HAL_GetTick();
    size_t received = 0;

    /* 在超时范围内尽可能多地读取数据 */
    while (received < len) {
        /* 检查环形缓冲区中是否有数据 */
        uint32_t avail = CDC_Ring_Available();
        if (avail > 0) {
            /* 读取可用数据（不超过剩余需求量）*/
            uint32_t to_read = (avail < (len - received)) ? avail : (uint32_t)(len - received);
            uint32_t got = CDC_Ring_Read(buf + received, to_read);
            received += got;
        }

        /* 如果已经有数据且超时，返回已读的 */
        uint32_t elapsed = HAL_GetTick() - start_tick;
        if (elapsed >= (uint32_t)timeout) {
            break;
        }

        /* 没数据时短暂让出 CPU（1ms，使用 FreeRTOS 延时避免忙等）*/
        if (avail == 0) {
            /* 在 FreeRTOS 任务中用 vTaskDelay 让出 CPU
             * 如果不在任务中（调度器未启动），用 HAL_Delay */
            extern uint32_t xTaskGetSchedulerState(void);
            if (xTaskGetSchedulerState() != 1 /* taskSCHEDULER_NOT_STARTED */) {
                extern void vTaskDelay(uint32_t);
                vTaskDelay(1);
            } else {
                HAL_Delay(1);
            }
        }
    }

    *err = 0;
    return received;
}
