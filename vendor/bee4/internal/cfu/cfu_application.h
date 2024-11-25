/**
*****************************************************************************************
*     Copyright(c) 2024, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file      cfu_application.h
   * @brief
   * @author    mandy
   * @date      2024-03-16
   * @version   v1.0
   **************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2017 Realtek Semiconductor Corporation</center></h2>
   **************************************************************************************
  */

#ifndef _CFU_APPLICATION__
#define _CFU_APPLICATION__

#ifdef __cplusplus
extern "C" {
#endif
/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include <stdint.h>
#include <stdbool.h>
#include "board.h"
#include "app_msg.h"
#if (CFU_PROTOCOL == CFU_NEW_PROTOCOL)
#include "cfu.h"
#elif (CFU_PROTOCOL == CFU_OLD_PROTOCOL)
#include "usb_old_cfu.h"
#endif
#if (WATCH_DOG_ENABLE == 1)
#include "wdt.h"
#endif

/*============================================================================*
 *                              Macros
 *============================================================================*/
#if (FEATURE_SUPPORT_CFU && (CFU_MODE == NORMAL_MODE))
#define EXIT_CFU_MODE_TIMEOUT 10000 //10s

#define ALLOW_TO_ENTER_CFU_MODE_FLAG 0x5aa55aa5
#define REJECT_TO_ENTER_CFU_MODE_FLAG 0x0

#define ENTER_CFU_MODE_DATA_LEN 3
#define ENTER_CFU_MODE_DATA_0 0x5d
#define ENTER_CFU_MODE_DATA_1 0x00
#define ENTER_CFU_MODE_DATA_2 0x01
#endif

/*============================================================================*
 *                         Types
 *============================================================================*/

/*============================================================================*
*                        Export Global Variables
*============================================================================*/
#if (FEATURE_SUPPORT_CFU && (CFU_MODE == NORMAL_MODE))
extern TimerHandle_t exit_cfu_mode_timer;
extern bool is_app_in_cfu_mode;
#endif
#if (WATCH_DOG_ENABLE == 1)
extern bool is_aon_wdg_enable;/* to indicate whether enable aon watchdog or not */
#endif

/*============================================================================*
 *                         Functions
 *============================================================================*/
void sw_timer_init(void);
#if (WATCH_DOG_ENABLE == 1)
void app_watchdog_open(uint32_t ms, WDTMode_TypeDef wdt_mode);
void app_watchdog_close(void);
#endif
#if (FEATURE_SUPPORT_CFU && (CFU_MODE == NORMAL_MODE))
bool save_enter_cfu_mode_flag(uint32_t flag);
void load_enter_cfu_mode_flag(void);
uint32_t get_enter_cfu_mode_flag(void);
void enter_to_cfu_mode(const uint8_t *data, uint8_t data_len);
void init_cfu_mode(void);
#endif
void app_handle_io_msg(T_IO_MSG io_msg);

#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2024 Realtek Semiconductor Corporation *****END OF FILE****/
