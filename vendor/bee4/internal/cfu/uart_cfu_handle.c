/**
**********************************************************************************************************
*               Copyright(c) 2024, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     uart_cfu_handle.c
* @brief    uart cfu handle
* @details
* @author   mandy
* @date     2024-10-31
* @version  v1.0
*********************************************************************************************************
*/

/*============================================================================*
 *                              Includes
 *============================================================================*/
#include "trace.h"
#include "string.h"
#include "app_section.h"
#include "cfu.h"
#include "uart_cfu_handle.h"

#if (FEATURE_SUPPORT_CFU && (UART_INTERFACE == CFU_INTERFACE))
/*============================================================================*
 *                              Macros
 *============================================================================*/

/*============================================================================*
 *                              Local Variables
 *============================================================================*/

/*============================================================================*
 *                              Global Variables
 *============================================================================*/

/*============================================================================*
 *                              Functions Declaration
 *============================================================================*/

/*============================================================================*
 *                              Private Funcitons
 *============================================================================*/

/*============================================================================*
 *                              Public Funcitons
 *============================================================================*//**
/******************************************************************
 * @brief   uart_cfu_handle_rx_packet
 * @param   p_packet - point to UART packet struct
 * @return  none
 * @retval  void
 */
APP_FLASH_TEXT_SECTION
void uart_cfu_handle_rx_packet(T_UART_PACKET_DEF *p_packet)
{
    uint8_t request_report_id = p_packet->uart_buf[0];
    uint8_t *p_request_payload = &p_packet->uart_buf[1];

    APP_PRINT_INFO2("[uart_cfu_handle_rx_packet] request report id = 0x%x, payload length = %d",
                    request_report_id, p_packet->payload_len);
    APP_PRINT_INFO1("[uart_cfu_handle_rx_packet] p_packet->uart_buf = 0x %b",
                    TRACE_BINARY(p_packet->payload_len, p_packet->uart_buf));

    if ((request_report_id == REPORT_ID_CFU_FW_VERSION_OR_DATA_REQUEST) &&
        (p_packet->payload_len == (CFU_FW_VERSION_REQUEST_LENGTH + 1)))
    {
        uint8_t p_response_data[255];
        uint16_t p_response_len = 0;
        cfu_handle_get_report_packet(request_report_id, p_response_data, &p_response_len);

        if (p_response_len > 0)
        {
            data_uart_cmd_response(REPORT_ID_CFU_FW_VERSION_OR_DATA_REQUEST, p_response_data, p_response_len);
        }
    }
    else
    {
        cfu_handle_set_report_packet(request_report_id, p_request_payload, (p_packet->payload_len - 1));
    }
}

#endif

/******************* (C) COPYRIGHT 2024 Realtek Semiconductor Corporation *****END OF FILE****/
