/*
 *  Copyright (c) 2025, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include <stdlib.h>
#include <os_sched.h>
#include <trace.h>
#include <gap.h>
#include <gap_adv.h>
#include <gap_bond_le.h>
#include <profile_server.h>
#include <gap_msg.h>
//#include <simple_ble_service.h>
//#include <bas.h>
//#include <app_task.h>
//#include <peripheral_app.h>
#if F_BT_ANCS_CLIENT_SUPPORT
#include <profile_client.h>
#include <ancs.h>
#endif
#include "board.h"
#include "mem_config.h"
#include "pm.h"
#include "rtl_nvic.h"
#include "io_dlps.h"
#include "app_section.h"

POWER_CheckResult dlps_allow = POWER_CHECK_PASS;

/** @defgroup  PERIPH_DEMO_MAIN Peripheral Main
    * @brief Main file to initialize hardware and BT stack and start task scheduling
    * @{
    */

/*============================================================================*
 *                              Constants
 *============================================================================*/

/*============================================================================*
 *                              Variables
 *============================================================================*/

/*============================================================================*
 *                              Functions
 *============================================================================*/
/**
 * @brief    Contains the initialization of pinmux settings and pad settings
 * @note     All the pinmux settings and pad settings shall be initiated in this function,
 *           but if legacy driver is used, the initialization of pinmux setting and pad setting
 *           should be peformed with the IO initializing.
 * @return   void
 */
void board_init(void)
{

}

/**
 * @brief    Contains the initialization of peripherals
 * @note     Both new architecture driver and legacy driver initialization method can be used
 * @return   void
 */
void driver_init(void)
{

}

/**
 * @brief    System_Handler
 * @note     system handle to judge which pin is wake source
 * @return   void
 */
RAM_FUNCTION
void System_Handler(void)
{
//    DBG_DIRECT("SYSTEM_HANDLER 0x%x", HAL_READ32(SOC_VENDOR2_REG_BASE, 0x0058));

//    if (System_WakeUpInterruptValue(P2_7) == SET)
//    {
//        DBG_DIRECT("P2_7 Wake up");
//        Pad_ClearWakeupINTPendingBit(P2_7);
//        System_WakeUpPinDisable(P2_7);
//    }

//    if (WakeUpDebounceInterruptValue(P2_7) == SET)
//    {
//        DBG_DIRECT("P2_7 debounce Wake up");
//        System_WakeUpPinDisable(P2_7);
//    }

//    HAL_WRITE32(SOC_VENDOR2_REG_BASE, 0x0058, 0x000001FF);
}

/**
 * @brief this function will be called before enter DLPS
 *
 *  set PAD and wakeup pin config for enterring DLPS
 *
 * @param none
 * @return none
 * @retval void
*/
extern void io_uart_dlps_enter(void);

/**
 * @brief this function will be called after exit DLPS
 *
 *  set PAD and wakeup pin config for enterring DLPS
 *
 * @param none
 * @return none
 * @retval void
*/
extern void io_uart_dlps_exit(void);

/**
 * @brief DLPS CallBack function
 * @param none
* @return true : allow enter dlps
 * @retval void
*/
RAM_FUNCTION
POWER_CheckResult app_dlps_check_cb(void)
{
    return dlps_allow;
}

/**
 * @brief    Contains the power mode settings
 * @return   void
 */
void pwr_mgr_init(void)
{
#if DLPS_EN
    power_check_cb_register(app_dlps_check_cb);
    DLPS_IORegUserDlpsEnterCb(io_uart_dlps_enter);
    DLPS_IORegUserDlpsExitCb(io_uart_dlps_exit);
    DLPS_IORegister();
    //bt_power_mode_set(BTPOWER_DEEP_SLEEP);
    power_mode_set(POWER_DLPS_MODE);
#endif
}

extern void zb_task_init(void);
/**
 * @brief    Entry of APP code
 * @return   int (To avoid compile warning)
 */
int rtk_main(void)
{
    if (FEATURE_TRUSTZONE_ENABLE)
    {
        DBG_DIRECT("Non-Secure World: main");
    }
    else
    {
        DBG_DIRECT("Secure World: main");
    }
    extern uint32_t random_seed_value;
    srand(random_seed_value);

    board_init();
    pwr_mgr_init();
    zb_task_init();
    os_sched_start();

    return 0;
}
/** @} */ /* End of group PERIPH_DEMO_MAIN */


