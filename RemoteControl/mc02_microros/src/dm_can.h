/**
 * @file dm_can.h
 * @brief 达妙电机 FDCAN 驱动头文件
 *
 * 支持的控制模式：
 *   - MIT (阻抗控制): 位置+速度+力矩+Kp+Kd
 *   - POS_VEL (位置速度): 位置+速度限制
 *   - VEL (速度): 纯速度控制
 *
 * CAN ID 规则：
 *   MIT:     CAN_ID = SlaveID
 *   POS_VEL: CAN_ID = 0x100 + SlaveID
 *   VEL:     CAN_ID = 0x200 + SlaveID
 */

#ifndef DM_CAN_H
#define DM_CAN_H

#include <stdint.h>
#include <stdbool.h>

/* ========================= 电机类型参数 ========================= */

/** 达妙电机参数（位置/速度/力矩最大值）*/
typedef struct {
    float p_max;   /* 位置最大值 (rad) */
    float v_max;   /* 速度最大值 (rad/s) */
    float t_max;   /* 力矩最大值 (Nm) */
} dm_motor_type_t;

/* DM4310 电机参数（舵轮常用）*/
#define DM4310_PARAMS  { .p_max = 12.5f, .v_max = 30.0f, .t_max = 10.0f }

/* ========================= 电机实例 ========================= */

/** 单个达妙电机的状态和配置 */
typedef struct {
    uint8_t  slave_id;    /* CAN 从机 ID (1-127) */
    uint8_t  master_id;   /* CAN 主机 ID (反馈用) */
    dm_motor_type_t type; /* 电机类型参数 */

    /* 反馈数据（由 CAN 接收回调更新）*/
    float fb_position;    /* 实际位置 (rad) */
    float fb_velocity;    /* 实际速度 (rad/s) */
    float fb_torque;      /* 实际力矩 (Nm) */
    uint32_t fb_tick;     /* 最近一次反馈的时间戳 (ms) */
} dm_motor_t;

/* ========================= API ========================= */

/**
 * @brief 初始化 FDCAN 外设
 * @return true 成功
 */
bool dm_can_init(void);

/**
 * @brief 使能电机
 */
void dm_motor_enable(dm_motor_t *motor);

/**
 * @brief 失能电机（安全停止）
 */
void dm_motor_disable(dm_motor_t *motor);

/**
 * @brief 设置电机零位
 */
void dm_motor_set_zero(dm_motor_t *motor);

/**
 * @brief MIT 模式控制（阻抗控制）
 *
 * @param motor 电机实例
 * @param pos   目标位置 (rad)
 * @param vel   目标速度 (rad/s)
 * @param kp    位置刚度 (0-500)
 * @param kd    速度阻尼 (0-5)
 * @param torque 前馈力矩 (Nm)
 */
void dm_motor_mit(dm_motor_t *motor, float pos, float vel,
                  float kp, float kd, float torque);

/**
 * @brief 位置-速度模式控制
 *
 * @param motor 电机实例
 * @param pos   目标位置 (rad)
 * @param vel   速度限制 (rad/s)
 */
void dm_motor_pos_vel(dm_motor_t *motor, float pos, float vel);

/**
 * @brief 速度模式控制
 *
 * @param motor 电机实例
 * @param vel   目标速度 (rad/s)
 */
void dm_motor_vel(dm_motor_t *motor, float vel);

/**
 * @brief 处理 CAN 接收（在中断或轮询中调用）
 *
 * 解析反馈帧，更新对应电机的 fb_* 字段。
 */
void dm_can_process_rx(void);

/**
 * @brief 所有电机紧急停止（速度归零 + 失能）
 *
 * @param motors 电机数组
 * @param count  电机数量
 */
void dm_motor_emergency_stop(dm_motor_t *motors, uint8_t count);

#endif /* DM_CAN_H */
