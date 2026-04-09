/**
 * @file usbd_cdc_if.h
 * @brief USB CDC 接口头文件
 */

#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H

#include "usbd_cdc.h"

/** CDC 接口操作结构体 */
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/** 保存 USBD 句柄 */
void CDC_Set_USBD_Handle(USBD_HandleTypeDef *pdev);

/** 通过 USB CDC 发送数据 */
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

/** 检查发送是否忙 */
uint8_t CDC_IsTxBusy(void);

/** 从环形缓冲区读取接收数据 */
uint32_t CDC_Ring_Read(uint8_t *buf, uint32_t len);

/** 查询可读数据量 */
uint32_t CDC_Ring_Available(void);

#endif /* __USBD_CDC_IF_H */
