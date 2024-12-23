/**
*****************************************************************************************
*     Copyright(c) 2024, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file      cfu_task.h
   * @brief     Routines to create App task and handle events & messages
   * @author    jane
   * @date      2017-06-02
   * @version   v1.0
   **************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2024 Realtek Semiconductor Corporation</center></h2>
   **************************************************************************************
  */
#ifndef _CFU_TASK__
#define _CFU_TASK__

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

/*============================================================================*
 *                              Macros
 *============================================================================*/

/*============================================================================*
 *                         Types
 *============================================================================*/

/*============================================================================*
*                        Export Global Variables
*============================================================================*/

/*============================================================================*
 *                         Functions
 *============================================================================*/
void task_init(void);
bool app_send_msg_to_apptask(T_IO_MSG *p_msg);

#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2024 Realtek Semiconductor Corporation *****END OF FILE****/


