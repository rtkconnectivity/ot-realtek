/**
**********************************************************************************************************
*               Copyright(c) 2024, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     cfu_application.c
* @brief
* @details
* @author   mandy
* @date     2024-03-16
* @version  v1.0
*********************************************************************************************************
*/

/*============================================================================*
 *                              Includes
 *============================================================================*/
#include <string.h>
#include <os_msg.h>
#include <os_task.h>
#include "board.h"
#include "trace.h"
#include "cfu_application.h"
#include <app_section.h>
#include "os_timer.h"
#include "ftl.h"
#include "cfu_task.h"
#include "cfu_callback.h"
#if (USB_INTERFACE == CFU_INTERFACE)
#include "usb_cfu_handle.h"
#elif (UART_INTERFACE == CFU_INTERFACE)
#include "uart_cfu_handle.h"
#include "uart_transport.h"
#endif
#if FEATURE_SUPPORT_CFU

/*============================================================================*
 *                              Macros
 *============================================================================*/

/*============================================================================*
 *                              Variables
 *============================================================================*/
#if (CFU_MODE == NORMAL_MODE)
static uint32_t enter_to_cfu_mode_flag = 0;
TimerHandle_t exit_cfu_mode_timer = NULL;
bool is_app_in_cfu_mode = false;
#endif
#if (WATCH_DOG_ENABLE == 1)
TimerHandle_t watch_dog_reset_dlps_timer;
bool is_aon_wdg_enable = false;/* to indicate whether enable aon watchdog or not */
#endif

/*============================================================================*
 *                              Functions Declaration
 *============================================================================*/
#if (WATCH_DOG_ENABLE == 1)
static void watch_dog_reset_dlps_timer_callback(TimerHandle_t p_timer) RAM_FUNCTION;
#endif

/*============================================================================*
 *                              Private Funcitons
 *============================================================================*/
#if (WATCH_DOG_ENABLE == 1)
/**
 * @brief  Watch dog reset timer callback
 * @param  p_timer - timer handler
 * @return None
 */
static void watch_dog_reset_dlps_timer_callback(TimerHandle_t p_timer)
{
    T_IO_MSG bee_io_msg = {0};
    bee_io_msg.type = IO_MSG_TYPE_RESET_WDG_TIMER;
    if (false == app_send_msg_to_apptask(&bee_io_msg))
    {
        APP_PRINT_ERROR0("[WDG] send IO_MSG_TYPE_RESET_WDG_TIMER message failed!");
    }
}
#endif

#if (CFU_MODE == NORMAL_MODE)
/**
 * @brief   exit_cfu_mode_timer callback
 * @param   p_timer - timer handler
 * @return  none
 * @retval  void
 */
static void exit_cfu_mode_timer_callback(TimerHandle_t p_timer)
{
    APP_PRINT_INFO0("exit_cfu_mode_timer_callback timeout!");

    if (CFU_IDLE == cfu_get_cfu_status())
    {
        WDG_SystemReset(RESET_ALL_EXCEPT_AON, DFU_FAIL_RESET);
    }
}
#endif

#if (UART_INTERFACE == CFU_INTERFACE)
/******************************************************************
 * @brief   handle UART RX message.
 * @param   io_driver_msg_recv - recieved io message.
 * @return  none
 * @retval  void
 */
static void uart_cfu_handle_rx_msg(T_IO_MSG io_driver_msg_recv)
{
    T_UART_PACKET_DEF *p_uart_packet = (T_UART_PACKET_DEF *)(io_driver_msg_recv.u.buf);

    uart_cfu_handle_rx_packet(p_uart_packet);
    data_uart_packet_struct_init(p_uart_packet);
}
#endif

/*============================================================================*
*                              Public Funcitons
*============================================================================*/
/**
 * @brief   Init sw timer
 * @param   None
 * @return  None
 */
void sw_timer_init(void)
{
    APP_PRINT_INFO0("[sw_timer_init] initialize sw timer");

#if (CFU_MODE == NORMAL_MODE)
    if (true == is_app_in_cfu_mode)
    {
        if (exit_cfu_mode_timer == NULL)
        {
            if (false == os_timer_create(&exit_cfu_mode_timer, "exit_cfu_mode_timer",  1, \
                                         EXIT_CFU_MODE_TIMEOUT, false, exit_cfu_mode_timer_callback))
            {
                APP_PRINT_ERROR0("[sw_timer_init] init exit_cfu_mode_timer failed");
            }
            else
            {
                os_timer_restart(&exit_cfu_mode_timer, EXIT_CFU_MODE_TIMEOUT);
            }
        }
    }
#endif

#if (WATCH_DOG_ENABLE == 1)
    if (false == os_timer_create(&watch_dog_reset_dlps_timer, "watch_dog_reset_dlps_timer",
                                 1, \
                                 WATCH_DOG_TIMEOUT_MS - 1000, true, watch_dog_reset_dlps_timer_callback))
    {
        APP_PRINT_INFO0("[sw_timer_init] init watch_dog_reset_dlps_timer failed");
    }
    else
    {
        os_timer_start(&watch_dog_reset_dlps_timer);
        APP_PRINT_INFO0("Start watch_dog_reset_dlps_timer!");
    }
#endif
}

#if (WATCH_DOG_ENABLE == 1)
/**
 * @brief  Open active and aon watch dog
 * @param  ms - timeout for watch dog reset
 * @param  wdt_mode - watch dog reset mode
 * @return None
 */
void app_watchdog_open(uint32_t ms, WDTMode_TypeDef wdt_mode)
{
    if (!is_aon_wdg_enable)
    {
        WDT_Start(ms, wdt_mode);
        is_aon_wdg_enable = true;
        os_timer_restart(&watch_dog_reset_dlps_timer, WATCH_DOG_TIMEOUT_MS - 1000);
    }
}

/**
 * @brief  Close active and aon watch dog
 * @param  None
 * @return None
 */
void app_watchdog_close(void)
{
    if (is_aon_wdg_enable)
    {
        WDT_Disable();
        is_aon_wdg_enable = false;
        os_timer_stop(&watch_dog_reset_dlps_timer);
    }
}
#endif

#if (CFU_MODE == NORMAL_MODE)
/**
 * @brief   save enter_to_cfu_mode_flag in ftl
 * @param   flag - the value of enter_to_cfu_mode_flag to be saved
 * @return  the result of ftl_save
 * @retval  bool
 */
bool save_enter_cfu_mode_flag(uint32_t flag)
{
    int32_t result = 0;
    enter_to_cfu_mode_flag = flag;

    result = ftl_save_to_module("app", &enter_to_cfu_mode_flag, FTL_ENTER_CFU_MODE_FLAG_BASE_ADDR,
                                FTL_ENTER_CFU_MODE_FLAG_LEN);
    APP_PRINT_INFO2("[save_enter_cfu_mode_flag] flag = %d, result = %d", flag, result);

    return (result == 0);
}

/**
 * @brief   load enter_to_cfu_mode_flag in ftl
 * @param   none
 * @return  the value of enter_to_cfu_mode_flag in ftl
 * @retval  uint32_t
 */
void load_enter_cfu_mode_flag(void)
{
    int32_t ftl_res = 0;

    ftl_res = ftl_load_from_module("app", &enter_to_cfu_mode_flag, FTL_ENTER_CFU_MODE_FLAG_BASE_ADDR,
                                   FTL_ENTER_CFU_MODE_FLAG_LEN);
    if (0 != ftl_res)
    {
        APP_PRINT_WARN0("[load_enter_cfu_mode_flag] enter cfu mode flag is invalid, reset to disable!");
        save_enter_cfu_mode_flag(REJECT_TO_ENTER_CFU_MODE_FLAG);
        ftl_res = ftl_load_from_module("app", &enter_to_cfu_mode_flag, FTL_ENTER_CFU_MODE_FLAG_BASE_ADDR,
                                       FTL_ENTER_CFU_MODE_FLAG_LEN);
    }

    APP_PRINT_INFO2("[load_enter_cfu_mode_flag] ftl_res is %d, value is 0x%08X", ftl_res,
                    enter_to_cfu_mode_flag);
}

/**
 * @brief   get enter_to_cfu_mode_flag
 * @param   none
 * @return  the value of enter_to_cfu_mode_flag
 * @retval  uint32_t
 */
uint32_t get_enter_cfu_mode_flag(void)
{
    APP_PRINT_INFO0("[get_enter_cfu_mode_flag]");

    return enter_to_cfu_mode_flag;
}

/**
 * @brief   switch to cfu mode
 * @param   data - the received datas
 * @param   data_len - the length of received datas
 * @return  none
 * @retval  void
 */
void enter_to_cfu_mode(const uint8_t *data, uint8_t data_len)
{
    APP_PRINT_INFO1("[enter_to_cfu_mode] data_len = %d", data_len);
    if (ENTER_CFU_MODE_DATA_LEN == data_len)
    {
        if ((ENTER_CFU_MODE_DATA_0 == data[0]) && (ENTER_CFU_MODE_DATA_1 == data[1]) &&
            (ENTER_CFU_MODE_DATA_2 == data[2]))
        {
            APP_PRINT_INFO0("[enter_to_cfu_mode] enter success");
            save_enter_cfu_mode_flag(ALLOW_TO_ENTER_CFU_MODE_FLAG);
            WDG_SystemReset(RESET_ALL_EXCEPT_AON, DFU_SWITCH_TO_OTA_MODE);
        }
        else
        {
            APP_PRINT_INFO0("[enter_to_cfu_mode] enter fail");
        }
    }
}

/**
 * @brief   init cfu driver
 * @param   none
 * @return  none
 * @retval  void
 */
void cfu_driver_init(void)
{
#if (WATCH_DOG_ENABLE == 1)
    app_watchdog_open(WATCH_DOG_TIMEOUT_MS, RESET_ALL_EXCEPT_AON);
#endif

    extern void mac_LoadConfigParam(void);
    mac_LoadConfigParam();

    sw_timer_init();

    extern void cfu_cb_struct_init(void);
    cfu_cb_struct_init();
    cfu_init();

#if (USB_INTERFACE == CFU_INTERFACE)
    extern void uart_init_internal(void);
    uart_init_internal();
    extern void usb_start(void);
    usb_start();
#elif (UART_INTERFACE == CFU_INTERFACE)
    data_uart_transport_init();
    data_uart_nvic_config();
#endif
}

/**
 * @brief   init cfu mode
 * @param   none
 * @return  none
 * @retval  void
 */
void init_cfu_mode(void)
{
    is_app_in_cfu_mode = true;

    save_enter_cfu_mode_flag(REJECT_TO_ENTER_CFU_MODE_FLAG);

    task_init();
}
#endif

/**
 * @brief  All the application messages are pre-handled in this function
 * @note   All the IO MSGs are sent to this function, then the event handling
 *         function shall be called according to the MSG type.
 * @param  io_msg - IO message data
 * @return None
 */
void app_handle_io_msg(T_IO_MSG io_msg)
{
    uint16_t msg_type = io_msg.type;
    uint16_t subtype = io_msg.subtype;

    switch (msg_type)
    {
#if ((UART_INTERFACE == CFU_INTERFACE) || (USB_TYPE == USE_USB_CDC))
    case IO_MSG_TYPE_UART:
        {
#if(CFU_MODE == NORMAL_MODE)
            if (false == is_app_in_cfu_mode)
            {
#if (USB_INTERFACE == CFU_INTERFACE)
                T_CDC_PACKET_DEF *p_packet = io_msg.u.buf;
                APP_PRINT_INFO1("[app_handle_io_msg] data_len = %d", p_packet->payload_len);
                if (p_packet->payload_len == ENTER_CFU_MODE_DATA_LEN)
                {
                    enter_to_cfu_mode(p_packet->cdc_buf, p_packet->payload_len);
                }
#elif (UART_INTERFACE == CFU_INTERFACE)
                T_UART_PACKET_DEF *p_packet = io_msg.u.buf;
                APP_PRINT_INFO1("[app_handle_io_msg] data_len = %d", p_packet->payload_len);
                if (p_packet->payload_len == ENTER_CFU_MODE_DATA_LEN)
                {
                    enter_to_cfu_mode(p_packet->uart_buf, p_packet->payload_len);
                }
#endif
            }
            else
#endif
            {
#if (USB_INTERFACE == CFU_INTERFACE)
                usb_cfu_handle_cdc_packet(io_msg.u.buf);
#elif (UART_INTERFACE == CFU_INTERFACE)
                uart_cfu_handle_rx_msg(io_msg);
#endif
            }
        }
        break;
#endif

#if (WATCH_DOG_ENABLE == 1)
    case IO_MSG_TYPE_RESET_WDG_TIMER:
        {
            APP_PRINT_INFO0("[app_handle_ble_io_msg] Watch Dog Rset Timer");
            WDT_Kick();
        }
        break;
#endif

    default:
        break;
    }
}

#endif

/******************* (C) COPYRIGHT 2024 Realtek Semiconductor Corporation *****END OF FILE****/
