/**
*********************************************************************************************************
*               Copyright(c) 2017, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      uart_transport.h
* @brief
* @details
* @author    bian
* @date      2023-03-25
* @version   v1.0
* *********************************************************************************************************
*/

#ifndef __UART_TRANSPORT_H
#define __UART_TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                        Header Files
 *============================================================================*/
#include "board.h"
#include <stdint.h>
#include <stdbool.h>
#include "trace.h"
#include "app_section.h"

#if (FEATURE_SUPPORT_CFU && (UART_INTERFACE == CFU_INTERFACE))

/*============================================================================*
 *                              Macros
 *============================================================================*/
/* Enable print log or not */
#define UART_PRINT_LOG

/* Configure UART packet buffer length */
#define CMD_SIZE                        61

#ifdef UART_PRINT_LOG
#define UART_DBG_BUFFER(MODULE, LEVEL, fmt, para_num,...) DBG_BUFFER_##LEVEL(TYPE_BEE2, SUBTYPE_FORMAT, MODULE, fmt, para_num, ##__VA_ARGS__)
#else
#define UART_DBG_BUFFER(MODULE, LEVEL, fmt, para_num,...) ((void)0)
#endif

/*============================================================================*
 *                         Types
 *============================================================================*/
/* UART packet data structure */
typedef struct t_uart_packet_def
{
    uint8_t  uart_buf[CMD_SIZE];  /* command buffer */
    uint16_t buf_index;       /* index of buffer */
    uint16_t payload_len;     /* length of decoder payload */
} T_UART_PACKET_DEF;

/*============================================================================*
*                        Export Global Variables
*============================================================================*/

/*============================================================================*
 *                         Functions
 *============================================================================*/
void data_uart_packet_struct_init(T_UART_PACKET_DEF *p_packet);
void data_uart_transport_init(void);
void data_uart_nvic_config(void);
bool data_uart_cmd_response(uint8_t usb_report_id, uint8_t *p_payload, uint16_t payload_length);
#endif

#ifdef __cplusplus
}
#endif

#endif /*__UART_TRANSPORT_H*/
/******************* (C) COPYRIGHT 2017 Realtek Semiconductor Corporation *****END OF FILE****/

