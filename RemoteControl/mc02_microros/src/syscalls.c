/**
 * @file syscalls.c
 * @brief Newlib 系统调用桩函数 - STM32H723 + FreeRTOS 优化版
 *
 * 关键修复：_sbrk 不能使用 SP 寄存器做堆上界检查！
 *
 * 原因：FreeRTOS 任务的 SP 指向 heap_4 分配的任务栈（低地址区域），
 * 而 _sbrk 的堆从 _end（BSS 末尾）向上增长。在 STM32H7 的内存布局中，
 * _end 可能比任务的 SP 地址更高，导致 _sbrk 误判为内存不足并返回 -1。
 *
 * 解决：使用 AXI SRAM 的末尾地址（0x24050000）作为固定堆上界。
 * STM32H723VGT6 的 AXI SRAM: 0x24000000 - 0x24050000 (320KB)
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <stdint.h>

/* 链接器脚本中定义的堆区域起点 */
extern uint8_t _end;

/* AXI SRAM 末尾地址（STM32H723: 320KB @ 0x24000000）*/
#define HEAP_LIMIT  ((uint8_t *)0x24050000)

/**
 * @brief _sbrk — 扩展堆内存
 *
 * 使用固定的 AXI SRAM 末尾地址作为堆上界，
 * 而不是 SP 寄存器（SP 在 FreeRTOS 任务中指向任务栈，不可靠）。
 */
void *_sbrk(ptrdiff_t incr)
{
    static uint8_t *heap_end = NULL;

    if (heap_end == NULL) {
        heap_end = &_end;
    }

    uint8_t *prev_heap_end = heap_end;

    /* 检查是否超过 AXI SRAM 末尾 */
    if (heap_end + incr > HEAP_LIMIT) {
        errno = ENOMEM;
        return (void *)-1;
    }

    heap_end += incr;
    return (void *)prev_heap_end;
}

/* ===== 其他必需的系统调用桩 ===== */

int _close(int fd) { (void)fd; return -1; }
int _fstat(int fd, struct stat *st) { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _isatty(int fd) { (void)fd; return 1; }
int _lseek(int fd, int ptr, int dir) { (void)fd; (void)ptr; (void)dir; return 0; }
int _read(int fd, char *ptr, int len) { (void)fd; (void)ptr; (void)len; return 0; }
int _write(int fd, char *ptr, int len) { (void)fd; (void)ptr; return len; }
void _exit(int status) { (void)status; for (;;) {} }
int _kill(int pid, int sig) { (void)pid; (void)sig; errno = EINVAL; return -1; }
int _getpid(void) { return 1; }
