/**
*****************************************************************************************
*     Copyright(c) 2025, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file    board.h
  * @brief   Board Configuration
  * @date    2017.6.6
  * @version v1.0
  * *************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2025 Realtek Semiconductor Corporation</center></h2>
   * *************************************************************************************
  */

/*============================================================================*
 *               Define to prevent recursive inclusion
 *============================================================================*/
#ifndef __BOARD_H__
#define __BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

#if(1 == BUILD_RCP)
#ifdef BOARD_RTL8771GUV
#include "openthread-core-rtl8771guv-rcp-config.h"
#endif
#ifdef BOARD_RTL8771GTV
#include "openthread-core-rtl8771gtv-rcp-config.h"
#endif
#ifdef BOARD_RTL8777G
#include "openthread-core-rtl8777g-rcp-config.h"
#endif
#include "zb_tst_cfg.h"
#endif

#if(1 == BUILD_RCP)
/*******************************************************
*                 CFU Config
*******************************************************/
#if FEATURE_SUPPORT_CFU

#define SILENT_MODE 1
#define NORMAL_MODE 2
#define CFU_MODE NORMAL_MODE

#define CFU_OLD_PROTOCOL 0
#define CFU_NEW_PROTOCOL 1
#define CFU_PROTOCOL CFU_NEW_PROTOCOL

#define USB_INTERFACE 1
#define UART_INTERFACE 2

#ifdef BOARD_RTL8777G
#define CFU_INTERFACE USB_INTERFACE
#endif
#ifdef BOARD_RTL8771GUV
#define CFU_INTERFACE USB_INTERFACE
#endif
#ifdef BOARD_RTL8771GTV
#define CFU_INTERFACE UART_INTERFACE
#endif

#if (USB_INTERFACE == CFU_INTERFACE)
#define USE_USB_HID 1
#define USE_USB_CDC 2
#define USB_TYPE USE_USB_CDC
#elif (UART_INTERFACE == CFU_INTERFACE)
#define CFU_UART           ZB_CLI_UART
#define CFU_UART_APB       ZB_CLI_UART_APB
#define CFU_UART_APBCLK    ZB_CLI_UART_APBCLK
#define CFU_UART_IRQN      ZB_CLI_UART_IRQn
#define CFU_UART_VECTORn   ZB_CLI_UART_VECTORn
#define CFU_UARTIntHandler ZB_CLI_UARTIntHandler

#define CFU_UART_TX        ZB_CLI_UART_TX
#define CFU_UART_RX        ZB_CLI_UART_RX
#define CFU_UART_RTS       ZB_CLI_UART_RTS
#define CFU_UART_CTS       ZB_CLI_UART_CTS
#endif

#endif

/*******************************************************
*                 WDG Config
*******************************************************/
#define WATCH_DOG_ENABLE        1    /* set 1 to enable active and dlps watch dog */
#define WATCH_DOG_TIMEOUT_MS    5000 /* unit: ms */

/*******************************************************
*                 FTL Address Config
*******************************************************/
#define FTL_ENTER_CFU_MODE_FLAG_BASE_ADDR 0
#define FTL_ENTER_CFU_MODE_FLAG_LEN 4

#define FTL_CONFIG_PARAM_BASE_ADDR 8
#define FTL_CONFIG_PARAM_LEN 28 //the max value of FTL_CONFIG_PARAM_LEN is 256

//Can start using FTL from logic address 264

#endif

/*******************************************************
*                 DLPS Module Config
*******************************************************/

/** @defgroup IO Driver Config
  * @note user must config it firstly!! Do not change macro names!!
  * @{
  */
/* if use user define dlps enter/dlps exit callback function */
#define USE_USER_DEFINE_DLPS_EXIT_CB    1
#define USE_USER_DEFINE_DLPS_ENTER_CB   1

/* if use any peripherals below, #define it 1 */
#define USE_ADC_DLPS         0
#define USE_ENHTIM_DLPS      0
#define USE_GPIOA_DLPS       0
#define USE_GPIOB_DLPS       0
#define USE_I2C0_DLPS        0
#define USE_I2C1_DLPS        0
#define USE_I2C2_DLPS        0
#define USE_I2C3_DLPS        0
#define USE_IR_DLPS          0
#define USE_KEYSCAN_DLPS     0
#define USE_SPI0_DLPS        0
#define USE_SPI1_DLPS        0
#define USE_SPI0_SLAVE_DLPS  0
#define USE_TIM_DLPS         0
#define USE_UART0_DLPS       0
#define USE_UART1_DLPS       0
#define USE_UART2_DLPS       0
#ifdef BUILD_USB
#define USE_UART3_DLPS       0
#else
#define USE_UART3_DLPS       1
#endif
#define USE_UART4_DLPS       0
#define USE_UART5_DLPS       0

/* do not modify USE_IO_DRIVER_DLPS macro */
#define USE_IO_DRIVER_DLPS  (USE_ADC_DLPS | USE_ENHTIM_DLPS | USE_GPIOA_DLPS | USE_GPIOB_DLPS \
                             | USE_I2C0_DLPS | USE_I2C1_DLPS | USE_I2C2_DLPS | USE_I2C3_DLPS \
                             | USE_IR_DLPS | USE_KEYSCAN_DLPS | USE_TIM_DLPS | USE_SPI0_DLPS \
                             | USE_SPI1_DLPS | USE_SPI0_SLAVE_DLPS | USE_UART0_DLPS | USE_UART1_DLPS \
                             | USE_UART2_DLPS | USE_UART3_DLPS | USE_UART4_DLPS | USE_UART5_DLPS \
                             | USE_USER_DEFINE_DLPS_ENTER_CB \
                             | USE_USER_DEFINE_DLPS_EXIT_CB)
/*******************************************************/

/*0: use FreeRTOS API, 1: USE OS Interface*/
#define  USE_OSIF                  1
#define  TEST_FTL                  0

#define  TEST_UART         1
#define  TEST_HW_TIMER     2
#define  TEST_LPC          3
#define  TEST_GPIO         4

#define  TEST_INTERRUPT             0 //TEST_UART //TEST_HW_TIMER // TEST_LPC // TEST_GPIO

#ifdef __cplusplus
}
#endif


/** @} */ /* End of group MEM_CONFIG */

#endif

