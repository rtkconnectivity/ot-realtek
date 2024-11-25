/**
*********************************************************************************************************
*               Copyright(c) 2024, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      usb_cfu_handle.h
* @brief     usb cfu handle
* @details
* @author    mandy
* @date      2024-10-31
* @version   v1.0
* *********************************************************************************************************
*/

#ifndef __USB_CFU_HANDLE_H
#define __USB_CFU_HANDLE_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include <stdint.h>
#include "board.h"
#include "cfu.h"

#if (FEATURE_SUPPORT_CFU && (USB_INTERFACE == CFU_INTERFACE))
/*============================================================================*
 *                         Macros
 *============================================================================*/
#if (USB_TYPE == USE_USB_CDC)
#define CFU_CMD_SIZE                (CFU_DATA_REPORT_LENGTH + 1)
#endif

/*============================================================================*
 *                              Variables
 *============================================================================*/
#if (USB_TYPE == USE_USB_CDC)
typedef struct t_cdc_packet_def
{
    uint8_t     cdc_buf[CFU_CMD_SIZE];  /* command buffer */
    uint32_t    payload_len;     /* length of decoder payload */
} T_CDC_PACKET_DEF;
#endif

/*============================================================================*
 *                              Functions Declaration
 *============================================================================*/
#if (USB_TYPE == USE_USB_CDC)
void usb_cfu_handle_cdc_packet(T_CDC_PACKET_DEF *p_packet);
#endif

#endif

#ifdef __cplusplus
}
#endif

#endif /*__USB_CFU_HANDLE_H*/

/******************* (C) COPYRIGHT 2024 Realtek Semiconductor Corporation *****END OF FILE****/

