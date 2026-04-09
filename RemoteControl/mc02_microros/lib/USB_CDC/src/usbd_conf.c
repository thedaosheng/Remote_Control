/**
 * @file usbd_conf.c
 * @brief USB Device 底层驱动实现 - MC02 (STM32H723)
 *
 * 实现 USB Device Library 要求的底层回调：
 *   - USB OTG HS (FS mode) 外设初始化（PA11=DM, PA12=DP）
 *   - PCD（Peripheral Controller Driver）事件处理
 *   - 端点操作（Open/Close/Transmit/PrepareReceive 等）
 *
 * MC02 使用 USB_OTG_HS（Full Speed, 12Mbps）
 */

#include "stm32h7xx_hal.h"
#include "usbd_core.h"
#include "usbd_def.h"

/* ========================= 全局变量 ========================= */

/** USB OTG HS (FS mode) PCD 句柄 */
PCD_HandleTypeDef hpcd_USB_OTG;

/* ========================= PCD 中断回调 ========================= */

/**
 * @brief USB OTG HS (FS mode) 中断处理函数
 * 必须在中断向量表中注册
 */
volatile uint32_t usb_irq_count = 0;
void OTG_HS_IRQHandler(void)
{
    usb_irq_count++;
    HAL_PCD_IRQHandler(&hpcd_USB_OTG);
}

/* ========================= PCD MSP（底层硬件）初始化 ========================= */

/**
 * @brief PCD 底层初始化回调（HAL_PCD_Init 内部调用）
 *
 * 配置：
 *   - PA11 = USB_DM (AF10)
 *   - PA12 = USB_DP (AF10)
 *   - USB OTG HS (FS mode) 时钟使能
 *   - NVIC 中断配置
 */
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
    if (hpcd->Instance == USB_OTG_HS) {
        /* 使能 GPIO 和 USB 时钟 */
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USB1_OTG_HS_CLK_ENABLE();

        /* 配置 PA11(DM) 和 PA12(DP) 为 USB 复用功能 */
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_NOPULL;
        gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF10_OTG1_HS;    /* AF10 = USB OTG HS (FS mode) */
        HAL_GPIO_Init(GPIOA, &gpio);

        /* USB 中断优先级必须高于 FreeRTOS 的 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5)
         * 设为 4，这样 FreeRTOS 的临界区不会 mask 掉 USB 中断
         * 但注意：优先级 < 5 的中断不能调用 FreeRTOS API（我们不需要）*/
        HAL_NVIC_SetPriority(OTG_HS_IRQn, 4, 0);
        HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
    }
}

/**
 * @brief PCD 底层反初始化
 */
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
    if (hpcd->Instance == USB_OTG_HS) {
        __HAL_RCC_USB1_OTG_HS_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
        HAL_NVIC_DisableIRQ(OTG_HS_IRQn);
    }
}

/* ========================= PCD 事件回调 → USBD Core ========================= */

/**
 * 以下回调由 HAL_PCD_IRQHandler 触发，
 * 转发给 USB Device Library Core 层处理协议逻辑。
 */

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SetupStage((USBD_HandleTypeDef *)hpcd->pData,
                        (uint8_t *)hpcd->Setup);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataOutStage((USBD_HandleTypeDef *)hpcd->pData, epnum,
                          hpcd->OUT_ep[epnum].xfer_buff);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataInStage((USBD_HandleTypeDef *)hpcd->pData, epnum,
                         hpcd->IN_ep[epnum].xfer_buff);
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SOF((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SetSpeed((USBD_HandleTypeDef *)hpcd->pData, USBD_SPEED_FULL);
    USBD_LL_Reset((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_Suspend((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_Resume((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_DevConnected((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_DevDisconnected((USBD_HandleTypeDef *)hpcd->pData);
}

/* ========================= USBD LL（Low Level）接口实现 ========================= */

/**
 * @brief 初始化 USB OTG HS (FS mode) 外设
 *
 * 由 USBD_Init() 调用，配置 PCD 参数并启动。
 */
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
    hpcd_USB_OTG.Instance                 = USB_OTG_HS;
    hpcd_USB_OTG.Init.dev_endpoints       = 9;       /* H723 支持 9 个端点 */
    hpcd_USB_OTG.Init.speed               = PCD_SPEED_FULL;
    hpcd_USB_OTG.Init.dma_enable          = DISABLE; /* FS 模式不支持 DMA */
    hpcd_USB_OTG.Init.phy_itface          = PCD_PHY_EMBEDDED;
    hpcd_USB_OTG.Init.Sof_enable          = DISABLE;
    hpcd_USB_OTG.Init.low_power_enable    = DISABLE;
    hpcd_USB_OTG.Init.lpm_enable          = DISABLE;
    hpcd_USB_OTG.Init.battery_charging_enable = DISABLE;
    hpcd_USB_OTG.Init.vbus_sensing_enable = DISABLE; /* MC02 无 VBUS 检测 */
    hpcd_USB_OTG.Init.use_dedicated_ep1   = DISABLE;

    /* 互相绑定 PCD 和 USBD 句柄 */
    hpcd_USB_OTG.pData = pdev;
    pdev->pData = &hpcd_USB_OTG;

    /* 初始化 PCD（内部调用 HAL_PCD_MspInit）*/
    if (HAL_PCD_Init(&hpcd_USB_OTG) != HAL_OK) {
        return USBD_FAIL;
    }

    /* 配置 RX FIFO 和 TX FIFO 大小（单位：32-bit words）
     * OTG HS 在 FS 模式下使用内嵌 PHY，FIFO 总量 1.25KB（320 words）
     * 分配：RX=128, TX0=32, TX1=64, TX2=32 → 总计 256 words (1KB) */
    HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG, 256);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG, 0, 64);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG, 1, 128);

    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_DeInit((PCD_HandleTypeDef *)pdev->pData);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_Start((PCD_HandleTypeDef *)pdev->pData);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_Stop((PCD_HandleTypeDef *)pdev->pData);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                   uint8_t ep_type, uint16_t ep_mps)
{
    HAL_PCD_EP_Open((PCD_HandleTypeDef *)pdev->pData, ep_addr, ep_mps, ep_type);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_PCD_EP_Close((PCD_HandleTypeDef *)pdev->pData, ep_addr);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_PCD_EP_Flush((PCD_HandleTypeDef *)pdev->pData, ep_addr);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_PCD_EP_SetStall((PCD_HandleTypeDef *)pdev->pData, ep_addr);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_PCD_EP_ClrStall((PCD_HandleTypeDef *)pdev->pData, ep_addr);
    return USBD_OK;
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)pdev->pData;
    if ((ep_addr & 0x80U) == 0x80U) {
        return hpcd->IN_ep[ep_addr & 0x7FU].is_stall;
    } else {
        return hpcd->OUT_ep[ep_addr & 0x7FU].is_stall;
    }
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
    HAL_PCD_SetAddress((PCD_HandleTypeDef *)pdev->pData, dev_addr);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                     uint8_t *pbuf, uint32_t size)
{
    HAL_PCD_EP_Transmit((PCD_HandleTypeDef *)pdev->pData, ep_addr, pbuf, size);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                           uint8_t *pbuf, uint32_t size)
{
    HAL_PCD_EP_Receive((PCD_HandleTypeDef *)pdev->pData, ep_addr, pbuf, size);
    return USBD_OK;
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef *)pdev->pData, ep_addr);
}

/**
 * @brief 延时函数（USB 协议中某些操作需要短延时）
 */
void USBD_LL_Delay(uint32_t Delay)
{
    HAL_Delay(Delay);
}
