/**
 * @file usbd_cdc_if.c
 * @brief USB CDC 虚拟串口接口实现 - MC02
 *
 * 实现 CDC 类所需的回调函数：
 *   - Init / DeInit / Control / Receive
 *
 * 内部维护一个环形接收缓冲区（2KB），供 micro-ROS transport 的
 * read 回调消费数据。发送则直接调用 CDC_Transmit_FS()。
 *
 * 数据流：
 *   PC (Agent) → USB → CDC Receive callback → ring buffer → transport read
 *   transport write → CDC_Transmit_FS → USB → PC (Agent)
 */

#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

/* ========================= 缓冲区定义 ========================= */

/** CDC 数据包大小（USB FS 最大 64 字节）*/
#define CDC_DATA_FS_MAX_PACKET_SIZE  64U

/** 环形接收缓冲区大小（必须是 2 的幂，便于取模）*/
#define CDC_RX_RING_SIZE  2048U

/** USB 接收临时缓冲区（USB 硬件 DMA 目标）*/
static uint8_t UserRxBufferFS[CDC_DATA_FS_MAX_PACKET_SIZE];
/** USB 发送缓冲区 */
static uint8_t UserTxBufferFS[CDC_DATA_FS_MAX_PACKET_SIZE];

/** 环形缓冲区 */
static volatile uint8_t  rx_ring[CDC_RX_RING_SIZE];
static volatile uint32_t rx_ring_head = 0;  /* 写入位置（中断上下文写）*/
static volatile uint32_t rx_ring_tail = 0;  /* 读取位置（任务上下文读）*/

/** 发送忙标志 */
static volatile uint8_t tx_busy = 0;

/** USBD 句柄指针（供外部 transmit 使用）*/
static USBD_HandleTypeDef *cdc_usbd_handle = NULL;

/* ========================= 环形缓冲区操作 ========================= */

/**
 * @brief 获取环形缓冲区中可读数据量
 */
static inline uint32_t ring_available(void)
{
    return (rx_ring_head - rx_ring_tail) & (CDC_RX_RING_SIZE - 1);
}

/**
 * @brief 从环形缓冲区读取数据
 * @param buf 目标缓冲区
 * @param len 期望读取长度
 * @return 实际读取字节数
 */
uint32_t CDC_Ring_Read(uint8_t *buf, uint32_t len)
{
    uint32_t avail = ring_available();
    if (len > avail) {
        len = avail;
    }
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = rx_ring[rx_ring_tail & (CDC_RX_RING_SIZE - 1)];
        rx_ring_tail++;
    }
    return len;
}

/**
 * @brief 查询环形缓冲区可读字节数
 */
uint32_t CDC_Ring_Available(void)
{
    return ring_available();
}

/* ========================= CDC 类回调实现 ========================= */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum);

/** CDC 接口操作结构体 - 注册到 CDC 类 */
USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
    CDC_Init_FS,
    CDC_DeInit_FS,
    CDC_Control_FS,
    CDC_Receive_FS,
    CDC_TransmitCplt_FS
};

/**
 * @brief CDC 初始化回调
 *
 * USB 枚举完成后由 CDC 类调用。
 * 设置接收缓冲区，准备接收第一个数据包。
 */
static int8_t CDC_Init_FS(void)
{
    /* 设置 CDC 的发送和接收缓冲区 */
    USBD_CDC_SetTxBuffer(cdc_usbd_handle, UserTxBufferFS, 0);
    USBD_CDC_SetRxBuffer(cdc_usbd_handle, UserRxBufferFS);

    /* 重置环形缓冲区 */
    rx_ring_head = 0;
    rx_ring_tail = 0;
    tx_busy = 0;

    return USBD_OK;
}

/**
 * @brief CDC 反初始化
 */
static int8_t CDC_DeInit_FS(void)
{
    return USBD_OK;
}

/**
 * @brief CDC 控制请求处理
 *
 * 处理 Host 发送的 CDC 类特定请求：
 *   - SET_LINE_CODING: Host 设置波特率等（我们忽略，USB 不需要波特率）
 *   - GET_LINE_CODING: Host 查询当前串口配置
 *   - SET_CONTROL_LINE_STATE: DTR/RTS 控制
 */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    (void)length;

    /* 默认 line coding：115200 8N1（给 Host 一个合理的回复）*/
    static uint8_t lineCoding[7] = {
        0x00, 0xC2, 0x01, 0x00,  /* 115200 baud (little-endian) */
        0x00,                     /* 1 stop bit */
        0x00,                     /* no parity */
        0x08                      /* 8 data bits */
    };

    switch (cmd) {
    case CDC_SEND_ENCAPSULATED_COMMAND:
    case CDC_GET_ENCAPSULATED_RESPONSE:
        break;

    case CDC_SET_COMM_FEATURE:
    case CDC_GET_COMM_FEATURE:
    case CDC_CLEAR_COMM_FEATURE:
        break;

    case CDC_SET_LINE_CODING:
        /* Host 设置波特率 — 保存但实际不使用 */
        if (pbuf && length >= 7) {
            memcpy(lineCoding, pbuf, 7);
        }
        break;

    case CDC_GET_LINE_CODING:
        /* 返回当前 line coding */
        if (pbuf) {
            memcpy(pbuf, lineCoding, 7);
        }
        break;

    case CDC_SET_CONTROL_LINE_STATE:
        /* DTR/RTS 状态变化 — 可用于检测终端连接 */
        break;

    case CDC_SEND_BREAK:
        break;

    default:
        break;
    }

    return USBD_OK;
}

/**
 * @brief CDC 接收回调（中断上下文！）
 *
 * 每当 Host 发送一个数据包（最大 64 字节），此函数被调用。
 * 将数据写入环形缓冲区，然后准备接收下一个包。
 *
 * @param Buf 接收到的数据
 * @param Len 数据长度
 */
static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
    /* 将接收到的数据写入环形缓冲区 */
    for (uint32_t i = 0; i < *Len; i++) {
        /* 检查缓冲区是否已满（保留 1 字节防溢出）*/
        if (ring_available() < (CDC_RX_RING_SIZE - 1)) {
            rx_ring[rx_ring_head & (CDC_RX_RING_SIZE - 1)] = Buf[i];
            rx_ring_head++;
        }
        /* 缓冲区满则丢弃数据 */
    }

    /* 准备接收下一个 USB 包 */
    USBD_CDC_SetRxBuffer(cdc_usbd_handle, UserRxBufferFS);
    USBD_CDC_ReceivePacket(cdc_usbd_handle);

    return USBD_OK;
}

/**
 * @brief CDC 发送完成回调
 *
 * 上一个发送包传输完成后调用，清除忙标志。
 */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
    (void)Buf;
    (void)Len;
    (void)epnum;
    tx_busy = 0;
    return USBD_OK;
}

/* ========================= 公共 API ========================= */

/**
 * @brief 保存 USBD 句柄（在 main 中调用 USBD_Init 后设置）
 */
void CDC_Set_USBD_Handle(USBD_HandleTypeDef *pdev)
{
    cdc_usbd_handle = pdev;
}

/**
 * @brief 通过 USB CDC 发送数据
 *
 * @param Buf 发送数据
 * @param Len 数据长度
 * @return USBD_OK 成功, USBD_BUSY 上一包还在传输, USBD_FAIL 失败
 */
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
    if (cdc_usbd_handle == NULL) {
        return USBD_FAIL;
    }

    USBD_CDC_HandleTypeDef *hcdc =
        (USBD_CDC_HandleTypeDef *)cdc_usbd_handle->pClassData;

    if (hcdc == NULL) {
        return USBD_FAIL;
    }

    if (hcdc->TxState != 0) {
        return USBD_BUSY;  /* 上一个包还在传输 */
    }

    USBD_CDC_SetTxBuffer(cdc_usbd_handle, Buf, Len);
    return (uint8_t)USBD_CDC_TransmitPacket(cdc_usbd_handle);
}

/**
 * @brief 检查发送是否忙
 */
uint8_t CDC_IsTxBusy(void)
{
    if (cdc_usbd_handle == NULL) return 1;
    USBD_CDC_HandleTypeDef *hcdc =
        (USBD_CDC_HandleTypeDef *)cdc_usbd_handle->pClassData;
    if (hcdc == NULL) return 1;
    return (hcdc->TxState != 0) ? 1 : 0;
}
