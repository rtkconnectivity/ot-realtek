/**
*****************************************************************************************
*     Copyright(c) 2025, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file      app_task.c
   * @brief     Routines to create App task and handle events & messages
   * @author    jane
   * @date      2017-06-02
   * @version   v1.0
   **************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2025 Realtek Semiconductor Corporation</center></h2>
   **************************************************************************************
  */

/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include <os_msg.h>
#include <os_task.h>
#include <os_sched.h>
#include <os_sync.h>
#include <trace.h>

#if (AON_WDG_ENABLE == 1)
#include "rtl876x_aon_wdg.h"
#endif
#include "rtl876x.h"
#include "rtl_pinmux.h"
#include "rtl_rcc.h"
#include "rtl_gpio.h"
#include "rtl_uart.h"
#include "rtl_tim.h"
#include "rtl_nvic.h"
#include "zb_tst_cfg.h"
#include "dbg_printf.h"

/** @addtogroup  MAC_TASK_DEMO
    * @{
    */

/** @defgroup  MAC_TASK IEEE802.15.4 MAC Task
    * @brief This file handles the implementation of application task related functions.
    *
    * Create App task and handle events & messages
    * @{
    */

/*============================================================================*
 *                              Macros
 *============================================================================*/
#ifdef BOARD_RTL8777G
#define OTA_SEL_PIN P9_0 // RTL8777G dongle SW2
#else
#define OTA_SEL_PIN P3_5 // EVB Key4
#endif

/*============================================================================*
 *                              Variables
 *============================================================================*/
void *thread_task_handle;   //!< MAC Task handle
void *zigbee_task_handle;   //!< MAC Task handle
void *matter_task_handle;   //!< MAC Task handle

/*============================================================================*
 *                              Functions
 *============================================================================*/
#include "mbedtls/threading.h"
#include "threading_alt.h"

void bee_mutex_init(mbedtls_threading_mutex_t *mutex)
{
    os_mutex_create(&mutex->mutex);
    if (mutex->mutex) { mutex->flag = 1; }
    else { mutex->flag = 0; }
}

void bee_mutex_free(mbedtls_threading_mutex_t *mutex)
{
    if (mutex != NULL && mutex->mutex != NULL && mutex->flag)
    {
        os_mutex_delete(mutex->mutex);
        mutex->flag = 0;
    }
}

int bee_mutex_lock(mbedtls_threading_mutex_t *mutex)
{
    if (mutex->flag)
    {
        os_mutex_take(mutex->mutex, 0xffffffff);
        return 0;
    }
    else
    {
        return MBEDTLS_ERR_THREADING_MUTEX_ERROR;
    }
}

int bee_mutex_unlock(mbedtls_threading_mutex_t *mutex)
{
    if (mutex->flag)
    {
        os_mutex_give(mutex->mutex);
        return 0;
    }
    else
    {
        return MBEDTLS_ERR_THREADING_MUTEX_ERROR;
    }
}

extern void uart_init_internal(void);
extern int main(int argc, char *argv[]);

void thread_test_task(void *p_param)
{
    DBG_DIRECT("%s", __func__);
    mbedtls_threading_set_alt(bee_mutex_init, bee_mutex_free, bee_mutex_lock, bee_mutex_unlock);
    uart_init_internal();
    main(0, NULL);
    while (1);
}

__attribute__((weak)) void thread_task_init(void)
{
    os_task_create(&thread_task_handle, "THREAD", thread_test_task, 0, THREAD_TASK_STACK_SIZE,
                   THREAD_TASK_PRIORITY);
}


#ifdef BUILD_MATTER
extern void InitGPIO(void);
extern void ChipTest(void);

void matter_test_task(void *p_param)
{
    DBG_DIRECT("%s", __func__);
    mbedtls_threading_set_alt(bee_mutex_init, bee_mutex_free, bee_mutex_lock, bee_mutex_unlock);
    uart_init_internal();
    InitGPIO();
    ChipTest();
    DBG_DIRECT("matter task done!");
    os_task_delete(NULL);
}
#endif

void startup_task_init(void)
{
#ifdef BUILD_MATTER
    extern uint32_t random_seed_value;
    srand(random_seed_value);
    os_task_create(&matter_task_handle, "matter_test", matter_test_task, 0, MATTER_TASK_STACK_SIZE,
                   MATTER_TASK_PRIORITY);
#else
    thread_task_init();
#endif
}

/** @} */ /* End of group MAC_TASK */
/** @} */ /* End of group MAC_TASK_DEMO */

