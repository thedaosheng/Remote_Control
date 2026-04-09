/**
 * @file dm_can.c
 * @brief 达妙电机 FDCAN 驱动实现
 *
 * 基于 STM32H723 FDCAN1 外设，直接驱动达妙电机。
 *
 * FDCAN1 引脚（需根据 MC02 原理图确认）：
 *   - PD0 = FDCAN1_RX（或 PB8）
 *   - PD1 = FDCAN1_TX（或 PB9）
 *
 * CAN 波特率：1Mbps（达妙默认 CAN 速率）
 *
 * 达妙协议 CAN ID 规则：
 *   MIT 模式:     CAN_ID = SlaveID
 *   POS_VEL 模式: CAN_ID = 0x100 + SlaveID
 *   VEL 模式:     CAN_ID = 0x200 + SlaveID
 *   使能/失能:    CAN_ID = SlaveID, data[0]=0xFC/0xFD
 */

#include "dm_can.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* ========================= FDCAN 句柄 ========================= */

static FDCAN_HandleTypeDef hfdcan1;
static FDCAN_TxHeaderTypeDef tx_header;

/* ========================= 内部工具函数 ========================= */

/**
 * @brief 浮点数量化为无符号整数
 *
 * 将 [x_min, x_max] 范围的浮点数映射到 [0, 2^bits-1] 的整数。
 * 达妙协议的核心编码方式。
 */
static uint32_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    /* 限幅 */
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;

    float span = x_max - x_min;
    float offset = x - x_min;
    uint32_t max_val = (1U << bits) - 1;

    return (uint32_t)(offset / span * (float)max_val);
}

/**
 * @brief 无符号整数反量化为浮点数
 */
static float uint_to_float(uint32_t x_int, float x_min, float x_max, uint8_t bits)
{
    uint32_t max_val = (1U << bits) - 1;
    float span = x_max - x_min;

    return (float)x_int / (float)max_val * span + x_min;
}

/* ========================= FDCAN 初始化 ========================= */

/**
 * @brief 初始化 FDCAN1 外设
 *
 * 配置 1Mbps 经典 CAN（非 FD），接受所有 CAN ID。
 * 时钟源：APB1 = 137.5MHz → 预分频 11 → 12.5MHz CAN 时钟
 * 位时间：12.5MHz / (1+9+2.5) = 1Mbps
 *
 * TODO: 根据 MC02 实际原理图确认 FDCAN1 引脚映射
 */
bool dm_can_init(void)
{
    /* 使能 FDCAN 和 GPIO 时钟 */
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* 配置 PD0(RX) 和 PD1(TX) 为 FDCAN1 复用 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &gpio);

    /* FDCAN1 配置 */
    hfdcan1.Instance                  = FDCAN1;
    hfdcan1.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;    /* 经典 CAN（非 FD）*/
    hfdcan1.Init.Mode                 = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission   = ENABLE;
    hfdcan1.Init.TransmitPause        = DISABLE;
    hfdcan1.Init.ProtocolException    = DISABLE;

    /* 位时间参数：1Mbps @ 12.5MHz CAN 时钟
     * APB1 = 137.5MHz, NominalPrescaler = 11 → 12.5MHz
     * Bit time = 1 + NominalTimeSeg1 + NominalTimeSeg2 = 1+9+2 = 12 tq
     * Baudrate = 12.5MHz / 12 ≈ 1.04Mbps（接近 1Mbps） */
    hfdcan1.Init.NominalPrescaler     = 11;
    hfdcan1.Init.NominalSyncJumpWidth = 1;
    hfdcan1.Init.NominalTimeSeg1      = 9;
    hfdcan1.Init.NominalTimeSeg2      = 2;

    /* 不使用 CAN FD 的数据段高速率 */
    hfdcan1.Init.DataPrescaler        = 1;
    hfdcan1.Init.DataSyncJumpWidth    = 1;
    hfdcan1.Init.DataTimeSeg1         = 1;
    hfdcan1.Init.DataTimeSeg2         = 1;

    /* 消息 RAM 配置 */
    hfdcan1.Init.MessageRAMOffset     = 0;
    hfdcan1.Init.StdFiltersNbr        = 1;
    hfdcan1.Init.ExtFiltersNbr        = 0;
    hfdcan1.Init.RxFifo0ElmtsNbr     = 16;
    hfdcan1.Init.RxFifo0ElmtSize     = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.RxFifo1ElmtsNbr     = 0;
    hfdcan1.Init.TxEventsNbr          = 0;
    hfdcan1.Init.TxBuffersNbr         = 0;
    hfdcan1.Init.TxFifoQueueElmtsNbr = 8;
    hfdcan1.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;
    hfdcan1.Init.TxElmtSize           = FDCAN_DATA_BYTES_8;

    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        return false;
    }

    /* 配置过滤器：接受所有标准 ID（用于接收电机反馈）*/
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x000;     /* 匹配所有 */
    filter.FilterID2    = 0x000;     /* 掩码 = 0 → 不过滤 */
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    /* 启动 FDCAN */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        return false;
    }

    /* 配置默认发送头 */
    tx_header.IdType      = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength  = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_OFF;
    tx_header.FDFormat            = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0;

    return true;
}

/* ========================= CAN 发送 ========================= */

/**
 * @brief 发送 8 字节 CAN 帧
 */
static void can_send(uint32_t can_id, uint8_t *data)
{
    tx_header.Identifier = can_id;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, data);
}

/* ========================= 电机控制命令 ========================= */

void dm_motor_enable(dm_motor_t *motor)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    can_send(motor->slave_id, data);
}

void dm_motor_disable(dm_motor_t *motor)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    can_send(motor->slave_id, data);
}

void dm_motor_set_zero(dm_motor_t *motor)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    can_send(motor->slave_id, data);
}

/**
 * @brief MIT 阻抗控制
 *
 * 数据编码：
 *   Byte 0-1:  位置 (16bit)
 *   Byte 2:    速度高 8bit
 *   Byte 3:    速度低 4bit | Kp 高 4bit
 *   Byte 4:    Kp 低 8bit
 *   Byte 5:    Kd 高 8bit
 *   Byte 6:    Kd 低 4bit | 力矩高 4bit
 *   Byte 7:    力矩低 8bit
 */
void dm_motor_mit(dm_motor_t *motor, float pos, float vel,
                  float kp, float kd, float torque)
{
    uint32_t q_uint   = float_to_uint(pos, -motor->type.p_max, motor->type.p_max, 16);
    uint32_t dq_uint  = float_to_uint(vel, -motor->type.v_max, motor->type.v_max, 12);
    uint32_t kp_uint  = float_to_uint(kp, 0.0f, 500.0f, 12);
    uint32_t kd_uint  = float_to_uint(kd, 0.0f, 5.0f, 12);
    uint32_t tau_uint = float_to_uint(torque, -motor->type.t_max, motor->type.t_max, 12);

    uint8_t data[8];
    data[0] = (q_uint >> 8) & 0xFF;
    data[1] = q_uint & 0xFF;
    data[2] = (dq_uint >> 4) & 0xFF;
    data[3] = ((dq_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F);
    data[4] = kp_uint & 0xFF;
    data[5] = (kd_uint >> 4) & 0xFF;
    data[6] = ((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F);
    data[7] = tau_uint & 0xFF;

    can_send(motor->slave_id, data);
}

/**
 * @brief 位置-速度模式
 * CAN_ID = 0x100 + SlaveID
 * Byte 0-3: 位置 (float32 IEEE754)
 * Byte 4-7: 速度 (float32 IEEE754)
 */
void dm_motor_pos_vel(dm_motor_t *motor, float pos, float vel)
{
    uint8_t data[8];
    memcpy(&data[0], &pos, 4);
    memcpy(&data[4], &vel, 4);

    can_send(0x100 + motor->slave_id, data);
}

/**
 * @brief 速度模式
 * CAN_ID = 0x200 + SlaveID
 * Byte 0-3: 速度 (float32 IEEE754)
 * Byte 4-7: 保留
 */
void dm_motor_vel(dm_motor_t *motor, float vel)
{
    uint8_t data[8] = {0};
    memcpy(&data[0], &vel, 4);

    can_send(0x200 + motor->slave_id, data);
}

/* ========================= CAN 接收处理 ========================= */

/* 最多支持 8 个电机的反馈映射表 */
#define DM_MAX_MOTORS  8
static dm_motor_t *motor_table[DM_MAX_MOTORS] = {NULL};
static uint8_t motor_count = 0;

/**
 * @brief 注册电机到反馈查找表
 */
void dm_motor_register(dm_motor_t *motor)
{
    if (motor_count < DM_MAX_MOTORS) {
        motor_table[motor_count++] = motor;
    }
}

/**
 * @brief 处理 FDCAN 接收 FIFO 中的反馈帧
 *
 * 达妙电机反馈帧格式：
 *   Byte 0:    MasterID (高 4bit) | 状态
 *   Byte 1-2:  位置 (16bit)
 *   Byte 3:    速度高 8bit
 *   Byte 4:    速度低 4bit | 力矩高 4bit
 *   Byte 5:    力矩低 8bit
 *   Byte 6-7:  错误码/状态
 */
void dm_can_process_rx(void)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    /* 读取所有 FIFO0 中的消息 */
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
            break;
        }

        /* 从反馈帧中提取 MasterID（Byte0 高位）*/
        /* 根据 CAN ID 查找对应电机 */
        for (uint8_t i = 0; i < motor_count; i++) {
            dm_motor_t *m = motor_table[i];
            if (m == NULL) continue;

            /* 达妙反馈帧的 CAN ID 等于 MasterID */
            if (rx_header.Identifier == m->master_id) {
                uint32_t q_uint  = ((uint32_t)rx_data[1] << 8) | rx_data[2];
                uint32_t dq_uint = ((uint32_t)rx_data[3] << 4) | (rx_data[4] >> 4);
                uint32_t tau_uint = (((uint32_t)rx_data[4] & 0x0F) << 8) | rx_data[5];

                m->fb_position = uint_to_float(q_uint, -m->type.p_max, m->type.p_max, 16);
                m->fb_velocity = uint_to_float(dq_uint, -m->type.v_max, m->type.v_max, 12);
                m->fb_torque   = uint_to_float(tau_uint, -m->type.t_max, m->type.t_max, 12);
                m->fb_tick     = HAL_GetTick();
                break;
            }
        }
    }
}

/* ========================= 紧急停止 ========================= */

void dm_motor_emergency_stop(dm_motor_t *motors, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        /* 先发速度 0 */
        dm_motor_vel(&motors[i], 0.0f);
    }
    /* 短延时后失能 */
    for (volatile uint32_t d = 0; d < 100000; d++) {}
    for (uint8_t i = 0; i < count; i++) {
        dm_motor_disable(&motors[i]);
    }
}
