/**
**********************************************************************************************************
*               Copyright(c) 2024, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     cfu_task.c
* @brief    Routines to create cfu task and handle events & messages
* @details
* @author   barry
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
#include "cfu_task.h"

#if FEATURE_SUPPORT_CFU

/*============================================================================*
 *                              Macros
 *============================================================================*/
#define configMINIMAL_SECURE_STACK_SIZE  1024

#define APP_TASK_PRIORITY               1   /* Task priorities. */
#define APP_TASK_STACK_SIZE             (512 * 4)

#define MAX_NUMBER_OF_GAP_MESSAGE       0x20
#define MAX_NUMBER_OF_IO_MESSAGE        0x20
#define MAX_NUMBER_OF_EVENT_MESSAGE     (MAX_NUMBER_OF_GAP_MESSAGE + MAX_NUMBER_OF_IO_MESSAGE)

/*============================================================================*
 *                              Variables
 *============================================================================*/
void *app_task_handle;
void *evt_queue_handle;
void *io_queue_handle;

/*============================================================================*
 *                              Functions Declaration
 *============================================================================*/

/*============================================================================*
 *                              Private Funcitons
 *============================================================================*/
/**
 * @brief  app main task
 * @param  p_param
 * @return None
 */
void app_main_task(void *p_param)
{
    uint8_t event;

    /* This task calls secure side functions. So allocate a secure context for
     * it. */
    //must locate at the first line
    os_alloc_secure_ctx(configMINIMAL_SECURE_STACK_SIZE);

    APP_PRINT_INFO0("app task");

    os_msg_queue_create(&io_queue_handle, "ioQ", MAX_NUMBER_OF_IO_MESSAGE,  sizeof(T_IO_MSG));
    os_msg_queue_create(&evt_queue_handle, "evtQ", MAX_NUMBER_OF_EVENT_MESSAGE, sizeof(uint8_t));

    extern void cfu_driver_init(void);
    cfu_driver_init();

    while (true)
    {
        // Task code goes here.
        if (os_msg_recv(evt_queue_handle, &event, 0xFFFFFFFF) == true)
        {
            if (event == EVENT_IO_TO_APP)
            {
                T_IO_MSG io_msg;
                if (os_msg_recv(io_queue_handle, &io_msg, 0) == true)
                {
                    extern void app_handle_io_msg(T_IO_MSG io_msg);
                    app_handle_io_msg(io_msg);
                }
            }
            else
            {
                APP_PRINT_INFO1("[app_main_task] event1 = %d", event);
            }
        }
    }
}

/*============================================================================*
*                              Public Funcitons
*============================================================================*/
/**
 * @brief  Initialize App task
 * @param  None
 * @return None
 */
void task_init(void)
{
    os_task_create(&app_task_handle, "app", app_main_task, 0, APP_TASK_STACK_SIZE,
                   APP_TASK_PRIORITY);
}

/**
 * @brief  Send message to app task
 * @param  p_msg - point to message
 * @return None
 */
bool app_send_msg_to_apptask(T_IO_MSG *p_msg)
{
    uint8_t event = EVENT_IO_TO_APP;

    if (os_msg_send(io_queue_handle, p_msg, 0) == false)
    {
        APP_PRINT_ERROR0("send_io_msg_to_app fail");
        return false;
    }
    if (os_msg_send(evt_queue_handle, &event, 0) == false)
    {
        APP_PRINT_ERROR0("send_evt_msg_to_app fail");
        return false;
    }
    return true;
}

#endif

/******************* (C) COPYRIGHT 2024 Realtek Semiconductor Corporation *****END OF FILE****/
