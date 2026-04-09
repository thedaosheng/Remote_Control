/**
 * @file clock_gettime.c
 * @brief POSIX clock_gettime 实现（基于 STM32 HAL tick）
 *
 * micro-ROS 内部使用 clock_gettime(CLOCK_MONOTONIC/CLOCK_REALTIME)
 * 获取时间戳。在裸机/FreeRTOS 环境下，使用 HAL_GetTick()
 * 提供毫秒级精度的时钟源。
 *
 * 注意：这里的"时间"不是真实时钟（没有 RTC），
 * 仅提供自启动以来的单调递增时间。
 */

#include <time.h>
#include <errno.h>
#include "stm32h7xx_hal.h"

/**
 * @brief POSIX clock_gettime 实现
 *
 * @param clk_id 时钟 ID（CLOCK_REALTIME 或 CLOCK_MONOTONIC）
 * @param tp 输出的 timespec 结构体
 * @return 0 成功, -1 失败
 *
 * 使用 HAL_GetTick() 返回的毫秒数转换为 timespec（秒 + 纳秒）
 */
int clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    (void)clk_id;  /* 裸机下 REALTIME 和 MONOTONIC 行为相同 */

    if (tp == NULL) {
        errno = EINVAL;
        return -1;
    }

    /* HAL_GetTick() 返回自系统启动以来的毫秒数 */
    uint32_t ms = HAL_GetTick();

    tp->tv_sec  = (time_t)(ms / 1000U);
    tp->tv_nsec = (long)((ms % 1000U) * 1000000UL);  /* ms → ns */

    return 0;
}
