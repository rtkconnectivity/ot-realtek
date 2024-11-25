/**
**********************************************************************************************************
*               Copyright(c) 2024, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     cfu_callback.c
* @brief    Define cfu callback functions
* @details
* @author   mandy
* @date     2024-03-16
* @version  v1.0
*********************************************************************************************************
*/

/*============================================================================*
 *                              Includes
 *============================================================================*/
#include "trace.h"
#include <app_section.h>
#include "os_timer.h"
#include "cfu_callback.h"
#include "cfu_application.h"
#include "wdt.h"

#if FEATURE_SUPPORT_CFU
/*============================================================================*
 *                              Macros
 *============================================================================*/

/*============================================================================*
 *                              Variables
 *============================================================================*/

/*============================================================================*
 *                              Functions Declaration
 *============================================================================*/

/*============================================================================*
 *                              Private Funcitons
 *============================================================================*/
/**
 * @brief   Handle cfu status timer callback
 * @param   status - timer handler
 * @return  None
 */
void cfu_status_timer_handle(T_CFU_STATUS status)
{
    APP_PRINT_INFO1("[cfu_status_timer_handle] status = %d", status);

    if (CFU_UPDATE_COMPLETE == status)
    {
#if (CFU_MODE == SILENT_MODE)
        dfu_fw_reboot(RESET_ALL, DFU_ACTIVE_RESET);
#else
        if (!is_ota_support_bank_switch())
        {
            save_enter_cfu_mode_flag(ALLOW_TO_ENTER_CFU_MODE_FLAG);
            WDG_SystemReset(RESET_ALL_EXCEPT_AON, DFU_SWITCH_TO_OTA_MODE);
        }
        else
        {
            WDG_SystemReset(RESET_ALL_EXCEPT_AON, DFU_ACTIVE_RESET);
        }
#endif
    }
    else
    {
        cfu_init();

#if (CFU_MODE == NORMAL_MODE)
        WDG_SystemReset(RESET_ALL_EXCEPT_AON, DFU_FAIL_RESET);
#endif
    }
}

/**
 * @brief   Handle cfu user ignore version callback
 * @param   image_id - the id of image
 * @return  bool - user ignore version is true or false
 */
bool cfu_user_ignore_version_handle(uint16_t image_id)
{
    APP_PRINT_INFO0("[cfu_user_ignore_version_handle]");

    if (IMG_MCUAPPDATA1 == image_id)
    {
        return true;
    }

    return false;
}

#if (CFU_MODE == NORMAL_MODE)
/**
 * @brief   Handle cfu start offer callback
 * @param   None
 * @return  None
 */
void cfu_start_offer_handle(void)
{
    APP_PRINT_INFO0("[cfu_start_offer_handle]");

    os_timer_stop(&exit_cfu_mode_timer);
}

/**
 * @brief   Handle cfu get version callback
 * @param   None
 * @return  None
 */
void cfu_get_version_handle(T_CFU_STATUS status)
{
    APP_PRINT_INFO1("[cfu_get_version_handle] status = %d", status);

    os_timer_stop(&exit_cfu_mode_timer);

    if (CFU_IDLE == status)
    {
        restart_cfu_status_timer(CFU_STATUS_CEHCK_TIMEOUT);
    }
}
#endif

#if (WATCH_DOG_ENABLE == 1)
/**
 * @brief   Handle cfu init callback
 * @param   None
 * @return  None
 */
void cfu_init_handle(void)
{
    APP_PRINT_INFO0("[cfu_init_handle]");

    app_watchdog_close();
    app_watchdog_open(WATCH_DOG_TIMEOUT_MS, RESET_ALL_EXCEPT_AON);
}

/**
 * @brief   Handle cfu accept offer callback
 * @param   None
 * @return  None
 */
void cfu_offer_accept_handle(void)
{
    APP_PRINT_INFO0("[cfu_offer_accept_handle]");

    app_watchdog_close();
    app_watchdog_open(WATCH_DOG_TIMEOUT_MS_FOR_DFU, RESET_ALL_EXCEPT_AON);
}
#endif

/*============================================================================*
*                              Public Funcitons
*============================================================================*/
/**
 * @brief   Init cfu callback struct
 * @param   None
 * @return  None
 */
void cfu_cb_struct_init(void)
{
    APP_PRINT_INFO0("[cfu_cb_struct_init]");
#if (CFU_PROTOCOL == CFU_NEW_PROTOCOL)
    T_CFU_CB cfu_cb = {NULL};
    cfu_cb.cfu_status_timer_cb = cfu_status_timer_handle;
    cfu_cb.cfu_user_ignore_version_cb = cfu_user_ignore_version_handle;
#if (CFU_MODE == NORMAL_MODE)
    cfu_cb.cfu_start_offer_cb = cfu_start_offer_handle;
    cfu_cb.cfu_get_version_cb = cfu_get_version_handle;
#endif
#if (WATCH_DOG_ENABLE == 1)
    cfu_cb.cfu_offer_accept_cb = cfu_offer_accept_handle;
    cfu_cb.cfu_init_cb = cfu_init_handle;
#endif
#if (USB_INTERFACE == CFU_INTERFACE)
    extern bool app_usb_send_dfu_data(uint8_t report_id, uint8_t *data, uint16_t len);
    cfu_cb.cfu_send_data_cb = app_usb_send_dfu_data;
#elif (UART_INTERFACE == CFU_INTERFACE)
    extern bool data_uart_cmd_response(uint8_t usb_report_id, uint8_t *p_payload,
                                       uint16_t payload_length);
    cfu_cb.cfu_send_data_cb = data_uart_cmd_response;
#endif

    cfu_cb_init(cfu_cb);
#endif
}

#endif

/******************* (C) COPYRIGHT 2024 Realtek Semiconductor Corporation *****END OF FILE****/
