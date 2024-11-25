/**
*********************************************************************************************************
*               Copyright(c) 2020, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     uart_transport.c
* @brief    This file provides uart transport layer driver for rcu uart test.
* @details
* @author   barry_bian
* @date     2023-03-25
* @version  v1.0
*********************************************************************************************************
*/

/*============================================================================*
 *                                  Header Files
 *============================================================================*/
#include <uart_transport.h>
#include <rtl_uart.h>
#include <rtl_pinmux.h>
#include <rtl_rcc.h>
#include <rtl_nvic.h>
#include <loop_queue.h>
#include <app_section.h>
#include "platform-bee.h"
#include "os_msg.h"
#include "app_msg.h"
#include "zb_tst_cfg.h"
#include "board.h"
#include "rtl_uart.h"

#if (FEATURE_SUPPORT_CFU && (UART_INTERFACE == CFU_INTERFACE))

/*============================================================================*
 *                                  Macros
 *============================================================================*/
/* Configure data uart receive trigger level */
#define UART_RX_TRIGGER_LEVEL           16
#define UART_RX_TRIGGER_VALUE           32

#define  FAR
typedef unsigned char    UCHAR, BYTE, * PUCHAR, * PBYTE;
typedef unsigned short   WORD, USHORT, * PUSHORT, * PWORD;
typedef unsigned char    FAR *LPBYTE;

/*============================================================================*
 *                              Local Variables
 *============================================================================*/
/* UART packet data structure */
static T_UART_PACKET_DEF uart_packet = {0};

/*============================================================================*
 *                              Functions Declaration
 *============================================================================*/
static void data_uart_write_data_to_cmd_fifo(T_UART_PACKET_DEF *p_uart_packet,
                                             const uint8_t *source_buf, uint8_t length) DATA_RAM_FUNCTION;
static void data_uart_handle_interrupt_handler(void) DATA_RAM_FUNCTION;

/*============================================================================*
 *                              Local Functions
 *============================================================================*/
/******************************************************************
 * @brief   write data to cmd fifo.
 * @param   none
 * @return  none
 * @retval  void
 */
static void data_uart_write_data_to_cmd_fifo(T_UART_PACKET_DEF *p_uart_packet,
                                             const uint8_t *source_buf, uint8_t length)
{
    uint8_t write_len = length;
    if (length > CMD_SIZE - p_uart_packet->buf_index)
    {
        write_len = CMD_SIZE - p_uart_packet->buf_index;
    }
    memcpy(&p_uart_packet->uart_buf[p_uart_packet->buf_index], source_buf, write_len);
    p_uart_packet->buf_index += write_len;
    p_uart_packet->payload_len = p_uart_packet->buf_index;
}

/******************************************************************
 * @brief   Data UART interrupt handle.
 * @param   none
 * @return  none
 * @retval  void
 */
void data_uart_handle_interrupt_handler(void)
{
    /* read interrupt id */
    uint32_t int_status = UART_GetIID(CFU_UART);

    UART_DBG_BUFFER(MODULE_UART, LEVEL_INFO, "[data_uart_handle_interrupt_handler] int_status = 0x%x",
                    1,
                    int_status);

    /* disable interrupt */
    UART_INTConfig(CFU_UART, UART_INT_RD_AVA | UART_INT_RX_LINE_STS, DISABLE);

    if (UART_GetFlagStatus(CFU_UART, UART_FLAG_RX_IDLE) == SET)
    {
        UART_INTConfig(CFU_UART, UART_INT_RX_IDLE, DISABLE);
        /* Send Msg to App task */
        T_IO_MSG uart_handle_msg;
        uart_handle_msg.type  = IO_MSG_TYPE_UART;
        uart_handle_msg.subtype = IO_MSG_UART_RX;
        uart_handle_msg.u.buf    = (void *)(&uart_packet);
        extern bool app_send_msg_to_apptask(T_IO_MSG * p_msg);
        if (false == app_send_msg_to_apptask(&uart_handle_msg))
        {
            APP_PRINT_INFO0("[data_uart_handle_interrupt_handler] Send IO_MSG_TYPE_UART message failed!");
        }
        UART_ClearRxFIFO(CFU_UART);
        UART_INTConfig(CFU_UART, UART_INT_RX_IDLE, ENABLE);
    }

    switch (int_status)
    {
    /* tx fifo empty, not enable */
    case UART_INT_ID_TX_EMPTY:
        {
            /* do nothing */
            break;
        }

    /* rx data valiable */
    case UART_INT_ID_RX_LEVEL_REACH:
    /* rx time out */
    case UART_INT_ID_RX_DATA_TIMEOUT:
        {
            uint8_t temp_buf[UART_RX_TRIGGER_VALUE];
            uint8_t length = UART_GetRxFIFODataLen(CFU_UART);
            UART_ReceiveData(CFU_UART, temp_buf, length);
            UART_DBG_BUFFER(MODULE_UART, LEVEL_INFO, "[data_uart_handle_interrupt_handler] length = %d", 1,
                            length);
            // UART_DBG_BUFFER(MODULE_UART, LEVEL_INFO, "[data_uart_handle_interrupt_handler] rx data = 0x %b", 1,
            //                 TRACE_BINARY(length, temp_buf));
            data_uart_write_data_to_cmd_fifo(&uart_packet, temp_buf, length);
            break;
        }

    /* receive line status interrupt */
    case UART_INT_ID_LINE_STATUS:
        {
            UART_GetFlagStatus(CFU_UART, UART_FLAG_RX_OVERRUN);
            UART_DBG_BUFFER(MODULE_APP, LEVEL_INFO, "[data_uart_handle_interrupt_handler] line status error!",
                            0);
            break;
        }

    default:
        break;
    }
    /* enable interrupt again */
    UART_INTConfig(CFU_UART, UART_INT_RD_AVA | UART_INT_RX_LINE_STS, ENABLE);
}

/*============================================================================*
 *                              Global Functions
 *============================================================================*/
/******************************************************************
 * @brief   Initializes UART packet data structure.
 * @param   p_packet - point to UART packet structure.
 * @return  none
 * @retval  void
 */
APP_FLASH_TEXT_SECTION
void data_uart_packet_struct_init(T_UART_PACKET_DEF *p_packet)
{
    /* Initialize UART packet data structure */
    p_packet->buf_index   = 0;
    p_packet->payload_len = 0;
}

/******************************************************************
 * @brief   Initializes UART peripheral.
 * @param   buadrate_opt - buadrate value.
 * @return  none
 * @retval  void
 */
APP_FLASH_TEXT_SECTION
void data_uart_init(uint32_t buadrate_opt)
{
    /* pinmux configuration */
    UART_DeInit(CFU_UART);
    Pinmux_Deinit(config_param.tx_pin);
    Pinmux_Deinit(config_param.rx_pin);
    Pinmux_Config(config_param.tx_pin, CFU_UART_TX);
    Pinmux_Config(config_param.rx_pin, CFU_UART_RX);

    /* pad configuration */
    Pad_Config(config_param.tx_pin, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
               PAD_OUT_LOW);
    Pad_Config(config_param.rx_pin, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
               PAD_OUT_LOW);

    /* turn on UART clock */
    RCC_PeriphClockCmd(CFU_UART_APB, CFU_UART_APBCLK, ENABLE);

    /* uart init */
    UART_InitTypeDef uartInitStruct;
    UART_StructInit(&uartInitStruct);
    uartInitStruct.UART_RxThdLevel = UART_RX_TRIGGER_LEVEL;

    switch (buadrate_opt)
    {
    case 115200:
        uartInitStruct.UART_Div = 20;
        uartInitStruct.UART_Ovsr = 12;
        uartInitStruct.UART_OvsrAdj = 0x252;
        break;

    case 230400:
        uartInitStruct.UART_Div = 11;
        uartInitStruct.UART_Ovsr = 10;
        uartInitStruct.UART_OvsrAdj = 0x3BB;
        break;

    case 460800:
        uartInitStruct.UART_Div = 6;
        uartInitStruct.UART_Ovsr = 9;
        uartInitStruct.UART_OvsrAdj = 0xAA;
        break;

    case 921600:
        uartInitStruct.UART_Div = 7;
        uartInitStruct.UART_Ovsr = 1;
        uartInitStruct.UART_OvsrAdj = 3;
        break;

    case 1000000:
        uartInitStruct.UART_Div = 4;
        uartInitStruct.UART_Ovsr = 5;
        uartInitStruct.UART_OvsrAdj = 0;
        break;

    case 2000000:
        uartInitStruct.UART_Div = 2;
        uartInitStruct.UART_Ovsr = 5;
        uartInitStruct.UART_OvsrAdj = 0;
        break;

    default:
        UART_DBG_BUFFER(MODULE_APP, LEVEL_INFO, "[data_uart_init] baurdate %d not support, set to 921600",
                        1, buadrate_opt);
        uartInitStruct.UART_Div = 7;
        uartInitStruct.UART_Ovsr = 1;
        uartInitStruct.UART_OvsrAdj = 3;
        break;
    }
    uartInitStruct.UART_Parity = UART_PARITY_NO_PARTY;

    UART_Init(CFU_UART, &uartInitStruct);
}

/******************************************************************
 * @brief   data_uart_nvic_config
 * @param   none
 * @return  none
 * @retval  void
 */
APP_FLASH_TEXT_SECTION
void data_uart_nvic_config(void)
{
    /* Enable UART IRQ */
    NVIC_InitTypeDef nvic_init_struct;
    nvic_init_struct.NVIC_IRQChannel         = CFU_UART_IRQN;
    nvic_init_struct.NVIC_IRQChannelCmd      = ENABLE;
    nvic_init_struct.NVIC_IRQChannelPriority = 4;
    NVIC_Init(&nvic_init_struct);

    UART_INTConfig(CFU_UART, UART_INT_RD_AVA | UART_INT_RX_LINE_STS | UART_INT_RX_IDLE, ENABLE);
    NVIC_ClearPendingIRQ(CFU_UART_IRQN);
}

/******************************************************************
 * @brief   Initializes loop queue and UART packet data structure to their default reset values.
 * @param   none
 * @return  none
 * @retval  void
 */
APP_FLASH_TEXT_SECTION
void data_uart_transport_init(void)
{
    /* Initialize UART packet data structure */
    data_uart_packet_struct_init(&uart_packet);

    /* Update Data UART interrupt handle */
    RamVectorTableUpdate(CFU_UART_VECTORn, data_uart_handle_interrupt_handler);

    /* Initialize Data UART peripheral */
    data_uart_init(config_param.baud_rate);
}

/******************************************************************
 * @brief   Response of uart command.
 * @param   p_payload - point to payload of uart response.
 * @param   payload_length - length of payload.
 * @return  none
 * @retval  void
 */
APP_FLASH_TEXT_SECTION
bool data_uart_cmd_response(uint8_t usb_report_id, uint8_t *p_payload, uint16_t payload_length)
{
    bool ret = true;
    T_UART_PACKET_DEF response_packet;
    T_UART_PACKET_DEF *p_resp_packet = &response_packet;

    p_resp_packet->buf_index = 0;
    p_resp_packet->payload_len = payload_length;

    p_resp_packet->uart_buf[p_resp_packet->buf_index++] = usb_report_id;

    while (p_resp_packet->payload_len--)
    {
        p_resp_packet->uart_buf[p_resp_packet->buf_index++] = *p_payload++;
    }

    UART_DBG_BUFFER(MODULE_UART, LEVEL_INFO, "[data_uart_cmd_response] tx data(%d bytes) = 0x %b", 2,
                    payload_length, TRACE_BINARY(payload_length + 1, p_resp_packet->uart_buf));

    /* send block bytes(16 bytes) */
    uint32_t i = 0;
    for (i = 0; i < (p_resp_packet->buf_index / UART_TX_FIFO_SIZE); i++)
    {
        UART_SendData(CFU_UART, &(p_resp_packet->uart_buf[UART_TX_FIFO_SIZE * i]), UART_TX_FIFO_SIZE);
        /* wait tx fifo empty */
        while (UART_GetFlagStatus(CFU_UART, UART_FLAG_TX_EMPTY) != SET);
    }

    /* send left bytes */
    UART_SendData(CFU_UART, &(p_resp_packet->uart_buf[UART_TX_FIFO_SIZE * i]),
                  p_resp_packet->buf_index % UART_TX_FIFO_SIZE);
    /* wait tx fifo empty */
    while (UART_GetFlagStatus(CFU_UART, UART_FLAG_TX_EMPTY) != SET);

    return true;
}
#endif

/******************* (C) COPYRIGHT 2020 Realtek Semiconductor Corporation *****END OF FILE****/

