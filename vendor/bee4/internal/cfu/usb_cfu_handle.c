/**
**********************************************************************************************************
*               Copyright(c) 2024, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     usb_cfu_handle.c
* @brief    usb cfu handle
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
#include "usb_cfu_handle.h"

#if (FEATURE_SUPPORT_CFU && (USB_INTERFACE == CFU_INTERFACE))
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
 *============================================================================*/
#if (USB_TYPE == USE_USB_CDC)
/**
 * @brief   Cfu handler for data from usb cdc
 * @param   p_packet - cdc rx packet
 * @return  None
 */
void usb_cfu_handle_cdc_packet(T_CDC_PACKET_DEF *p_packet)
{
    uint8_t request_report_id = p_packet->cdc_buf[0];
    uint8_t *p_request_payload = &p_packet->cdc_buf[1];

    if ((request_report_id == REPORT_ID_CFU_FW_VERSION_OR_DATA_REQUEST) &&
        (p_packet->payload_len == (CFU_FW_VERSION_REQUEST_LENGTH + 1)))
    {
        uint8_t p_response_data[255];
        uint16_t p_response_len = 0;
        cfu_handle_get_report_packet(request_report_id, p_response_data, &p_response_len);

        if (p_response_len > 0)
        {
            extern bool app_usb_send_dfu_data(uint8_t report_id, uint8_t *data, uint16_t len);
            app_usb_send_dfu_data(REPORT_ID_CFU_FW_VERSION_OR_DATA_REQUEST, (uint8_t *)&p_response_data,
                                  p_response_len);
        }
    }
    else
    {
        cfu_handle_set_report_packet(request_report_id, p_request_payload, (p_packet->payload_len - 1));
    }
}
#endif

#endif

/******************* (C) COPYRIGHT 2023 Realtek Semiconductor Corporation *****END OF FILE****/
