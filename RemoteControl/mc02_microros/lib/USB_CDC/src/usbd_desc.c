/**
 * @file usbd_desc.c
 * @brief USB 设备描述符 - MC02 CDC
 *
 * 定义 USB 枚举时 Host 读取的各种描述符：
 *   - 设备描述符（VID/PID/版本）
 *   - 配置描述符
 *   - 字符串描述符（厂商/产品/序列号）
 *   - 设备限定描述符
 *
 * VID 0x1999 / PID 0x0001 是自定义值，
 * 实际产品需要申请正式 USB VID。
 */

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

/* ========================= 描述符常量 ========================= */

#define USBD_VID                     0x1999U   /* 自定义 Vendor ID */
#define USBD_PID_FS                  0x0001U   /* MC02 micro-ROS CDC */
#define USBD_LANGID_STRING           0x0409U   /* English (US) */
#define USBD_MANUFACTURER_STRING     "DaMiao"
#define USBD_PRODUCT_STRING          "MC02 micro-ROS CDC"
#define USBD_SERIAL_STRING           "MC02-001"
#define USBD_CONFIGURATION_STRING    "CDC Config"
#define USBD_INTERFACE_STRING        "CDC Interface"

#define USB_SIZ_BOS_DESC             0x0CU

/* ========================= 描述符数据 ========================= */

/** USB 设备描述符（18 字节）*/
static uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] = {
    0x12,                       /* bLength: 描述符长度 */
    USB_DESC_TYPE_DEVICE,       /* bDescriptorType: Device */
    0x00, 0x02,                 /* bcdUSB: USB 2.0 */
    0x02,                       /* bDeviceClass: CDC (Communications) */
    0x02,                       /* bDeviceSubClass: ACM */
    0x00,                       /* bDeviceProtocol */
    USB_MAX_EP0_SIZE,           /* bMaxPacketSize0: 64 bytes */
    LOBYTE(USBD_VID), HIBYTE(USBD_VID),   /* idVendor */
    LOBYTE(USBD_PID_FS), HIBYTE(USBD_PID_FS), /* idProduct */
    0x00, 0x02,                 /* bcdDevice: rel. 2.00 */
    USBD_IDX_MFC_STR,          /* iManufacturer */
    USBD_IDX_PRODUCT_STR,      /* iProduct */
    USBD_IDX_SERIAL_STR,       /* iSerialNumber */
    USBD_MAX_NUM_CONFIGURATION  /* bNumConfigurations */
};

/** Language ID 描述符 */
static uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] = {
    USB_LEN_LANGID_STR_DESC,
    USB_DESC_TYPE_STRING,
    LOBYTE(USBD_LANGID_STRING), HIBYTE(USBD_LANGID_STRING)
};

/** 字符串描述符缓冲区 */
static uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ];

/* ========================= 描述符回调函数 ========================= */

static uint8_t *USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);

/** 描述符回调结构体 - 注册到 USBD Core */
USBD_DescriptorsTypeDef FS_Desc = {
    USBD_FS_DeviceDescriptor,
    USBD_FS_LangIDStrDescriptor,
    USBD_FS_ManufacturerStrDescriptor,
    USBD_FS_ProductStrDescriptor,
    USBD_FS_SerialStrDescriptor,
    USBD_FS_ConfigStrDescriptor,
    USBD_FS_InterfaceStrDescriptor
};

/* ========================= 回调实现 ========================= */

static uint8_t *USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(USBD_FS_DeviceDesc);
    return USBD_FS_DeviceDesc;
}

static uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(USBD_LangIDDesc);
    return USBD_LangIDDesc;
}

static uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

static uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_PRODUCT_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

static uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_SERIAL_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

static uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

static uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_INTERFACE_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}
