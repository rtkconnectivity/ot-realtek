/**
*****************************************************************************************
*     Copyright(c) 2017, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file      main.c
   * @brief     Source file for BLE peripheral project, mainly used for initialize modules
   * @author    jane
   * @date      2017-06-12
   * @version   v1.0
   **************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2017 Realtek Semiconductor Corporation</center></h2>
   **************************************************************************************
  */

/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include <os_sched.h>
#include <string.h>
#include <stdlib.h>
#include <trace.h>
#include <os_task.h>
#include <os_sync.h>
#include "rtl_pinmux.h"
#include "rtl_rcc.h"
#include "rtl_gpio.h"
#include "rtl_uart.h"
#include "rtl_tim.h"
#include "rtl_nvic.h"

#include "zb_tst_cfg.h"
#include "vector_table_ext.h"

#include "app_section.h"
#include "mac_driver.h"
#include "mac_driver_mpan.h"

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
void *zb_task_handle;   //!< ZB MAC Task handle
extern void shell_cmd_init(void);
extern void otPlatFlashErase(uint32_t *, uint8_t);
extern int32_t matter_kvs_clean(void);

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
void zb_pin_mux_init(void)
{
#if (1 == BUILD_RCP)
#ifdef BOARD_RTL8771GTV
    // UART for CLI
    Pad_Config(config_param.tx_pin, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
               PAD_OUT_HIGH);
    Pad_Config(config_param.rx_pin, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
               PAD_OUT_HIGH);
    if (((config_param.func_msk & (1 << HW_FLOWCTL_OFFSET)) == 0x00) ||
        ((config_param.func_msk & (1 << FORCE_HW_FLOWCTL_OFFSET)) == 0x00))
    {
        Pad_Config(config_param.rts_pin, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
                   PAD_OUT_LOW);
        Pad_Config(config_param.cts_pin, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
                   PAD_OUT_LOW);
    }

    Pinmux_Config(config_param.tx_pin, ZB_CLI_UART_TX);
    Pinmux_Config(config_param.rx_pin, ZB_CLI_UART_RX);
    if (((config_param.func_msk & (1 << HW_FLOWCTL_OFFSET)) == 0x00) ||
        ((config_param.func_msk & (1 << FORCE_HW_FLOWCTL_OFFSET)) == 0x00))
    {
        Pinmux_Config(config_param.rts_pin, ZB_CLI_UART_RTS);
        Pinmux_Config(config_param.cts_pin, ZB_CLI_UART_CTS);
    }
#endif

    // External PA
    if (config_param.ext_pa != 0xff)
    {
        if (config_param.ext_pa_lna_ploatiry & (1 << 0))
        {
            Pad_Config(config_param.ext_pa, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
                       PAD_OUT_HIGH);
        }
        else
        {
            Pad_Config(config_param.ext_pa, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
                       PAD_OUT_LOW);
        }
    }

    // External LNA
    if (config_param.ext_lna != 0xff)
    {
        if (config_param.ext_pa_lna_ploatiry & (1 << 1))
        {
            Pad_Config(config_param.ext_lna, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
                       PAD_OUT_HIGH);
        }
        else
        {
            Pad_Config(config_param.ext_lna, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
                       PAD_OUT_LOW);
        }
    }
#else
    // UART for CLI
    Pad_Config(ZB_CLI_UART_TX_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
               PAD_OUT_HIGH);
    Pad_Config(ZB_CLI_UART_RX_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
               PAD_OUT_HIGH);
    Pinmux_Config(ZB_CLI_UART_TX_PIN, ZB_CLI_UART_TX);
    Pinmux_Config(ZB_CLI_UART_RX_PIN, ZB_CLI_UART_RX);
#endif
}

/**
 * @brief    Contains the initialization of peripherals
 * @note     Both new architecture driver and legacy driver initialization method can be used
 * @return   void
 */
void zb_periheral_drv_init(void)
{
#if (1 == BUILD_RCP)
    DBG_DIRECT("uart_tx_pin %d", config_param.tx_pin);
    DBG_DIRECT("uart_rx_pin %d", config_param.rx_pin);
    DBG_DIRECT("uart_rts_pin %d", config_param.rts_pin);
    DBG_DIRECT("uart_cts_pin %d", config_param.cts_pin);
    DBG_DIRECT("uart_func_msk %x", config_param.func_msk);
    DBG_DIRECT("uart_baudrate %d", config_param.baud_rate);;
    DBG_DIRECT("pta_dis %x", config_param.pta_dis);
    DBG_DIRECT("pta_wl_act %d", config_param.pta_wl_act);
    DBG_DIRECT("pta_bt_act %d", config_param.pta_bt_act);
    DBG_DIRECT("pta_bt_stat %d", config_param.pta_bt_stat);
    DBG_DIRECT("pta_bt_clk %d", config_param.pta_bt_clk);
    DBG_DIRECT("ext_pa %d", config_param.ext_pa);
    DBG_DIRECT("ext_lna %d", config_param.ext_lna);
    DBG_DIRECT("ext_pa_lna_polarity %x", config_param.ext_pa_lna_ploatiry);
#endif
}

extern void Zigbee_Handler_Patch(void);

void zb_mac_interrupt_enable(void)
{
    //NVIC_InitTypeDef NVIC_InitStruct;
    // TODO: enable MAC interrupt
    /* share the same IRQ number with BT_MAC on FPGA temporary, so the interrupt
       shall be initialed in BT lower stack initialization */
    NVIC_SetPriority(Zigbee_IRQn, 2);
    NVIC_EnableIRQ(Zigbee_IRQn);
    RamVectorTableUpdate_ext(Zigbee_VECTORn, Zigbee_Handler_Patch);
}

int32_t edscan_level2dbm(int32_t level)
{
    return (level * 2) - 90;
}

extern uint32_t (*lowerstack_SystemCall)(uint32_t opcode, uint32_t param, uint32_t param1,
                                         uint32_t param2);
extern void set_zigbee_priority(uint16_t priority, uint16_t priority_min);
extern uint32_t get_zigbee_window_slot_imp(int16_t *prio, int16_t *prio_min);
extern void (*modem_set_zb_cca_combination)(uint8_t comb);

mac_attribute_t attr;
mac_driver_t drv;
pan_mac_comm_t pan_mac_comm;

extern void mac_SetTxNCsma(bool enable);
void zb_mac_drv_init(void)
{
    mac_InitAttribute(&attr);
    attr.mac_cfg.rf_early_term = 0;
    //attr.mac_cfg.frm06_rx_early = 0;
#if (BUILD_RCP == 1)
    attr.phy_arbitration_en = 0;
#else
    lowerstack_SystemCall(10, 1, 512, -1);
#endif
    mac_Enable();
    mac_Initialize(&drv, &attr);
    mac_Initialize_Additional();
    mac_SetCcaMode(MAC_CCA_ED);
}

typedef void (*bt_hci_reset_handler_t)(void);
extern void mac_RegisterBtHciResetHanlder(bt_hci_reset_handler_t handler);
void zb_mac_drv_enable(void)
{
    mpan_CommonInit(&pan_mac_comm);
    zb_mac_drv_init();
    mac_RegisterBtHciResetHanlder(zb_mac_drv_init);
    mac_RegisterCallback(NULL, edscan_level2dbm, set_zigbee_priority,
                         *modem_set_zb_cca_combination);
}

extern void mac_Initialize_Patch(void);
extern void startup_task_init(void);

void zb_task_init(void)
{
#if (1 == BUILD_RCP)
    extern void mac_LoadConfigParam(void);
    mac_LoadConfigParam();
#endif

    mac_Initialize_Patch();

    zb_pin_mux_init();
    zb_periheral_drv_init();
    zb_mac_interrupt_enable();
    zb_mac_drv_enable();

    startup_task_init();
}

/** @} */ /* End of group PERIPH_DEMO_MAIN */

#include <string.h>
#include "os_mem.h"

APP_FLASH_TEXT_SECTION void *__wrap__malloc_r(struct _reent *ptr, size_t size)
{
    void *mem;
    mem = os_mem_alloc(RAM_TYPE_DATA_ON, size);
    return mem;
}

APP_FLASH_TEXT_SECTION void __wrap__free_r(struct _reent *ptr, void *addr)
{
    os_mem_free(addr);
}

APP_FLASH_TEXT_SECTION void *__wrap__realloc_r(struct _reent *ptr, void *mem, size_t newsize)
{
    void *p;
    if (mem)
    {
        os_mem_free(mem);
    }
    p = os_mem_alloc(RAM_TYPE_DATA_ON, newsize);
    return p;
}

APP_FLASH_TEXT_SECTION void *__wrap__calloc_r(struct _reent *ptr, size_t size, size_t len)
{
    void *mem;
    mem = os_mem_zalloc(RAM_TYPE_DATA_ON, (size * len));
    return mem;
}

#define CHECK_STR_UNALIGNED(X, Y) \
    (((uint32_t)(X) & (sizeof (uint32_t) - 1)) | \
     ((uint32_t)(Y) & (sizeof (uint32_t) - 1)))

#define STR_OPT_BIGBLOCKSIZE     (sizeof(uint32_t) << 2)

#define STR_OPT_LITTLEBLOCKSIZE (sizeof (uint32_t))

APP_FLASH_TEXT_SECTION void *__wrap_memcpy(void *s1, const void *s2, size_t n)
{
    char *dst = (char *) s1;
    const char *src = (const char *) s2;

    uint32_t *aligned_dst;
    const uint32_t *aligned_src;

    /* If the size is small, or either SRC or DST is unaligned,
     * then punt into the byte copy loop.  This should be rare.
     */
    if (n < sizeof(uint32_t) || CHECK_STR_UNALIGNED(src, dst))
    {
        while (n--)
        {
            *dst++ = *src++;
        }

        return s1;
    } /* if */

    aligned_dst = (uint32_t *)dst;
    aligned_src = (const uint32_t *)src;

    /* Copy 4X long words at a time if possible.  */
    while (n >= STR_OPT_BIGBLOCKSIZE)
    {
        *aligned_dst++ = *aligned_src++;
        *aligned_dst++ = *aligned_src++;
        *aligned_dst++ = *aligned_src++;
        *aligned_dst++ = *aligned_src++;
        n -= STR_OPT_BIGBLOCKSIZE;
    } /* while */

    /* Copy one long word at a time if possible.  */
    while (n >= STR_OPT_LITTLEBLOCKSIZE)
    {
        *aligned_dst++ = *aligned_src++;
        n -= STR_OPT_LITTLEBLOCKSIZE;
    } /* while */

    /* Pick up any residual with a byte copier.  */
    dst = (char *)aligned_dst;
    src = (const char *)aligned_src;
    while (n--)
    {
        *dst++ = *src++;
    }

    return s1;
} /* _memcpy() */

#define wsize   sizeof(uint32_t)
#define wmask   (wsize - 1)

void *__wrap_memset(void *dst0, int Val, size_t length)
{
    size_t t;
    uint32_t Wideval;
    uint8_t *dst;

    dst = dst0;
    /*
     * If not enough words, just fill bytes.  A length >= 2 words
     * guarantees that at least one of them is `complete' after
     * any necessary alignment.  For instance:
     *
     *  |-----------|-----------|-----------|
     *  |00|01|02|03|04|05|06|07|08|09|0A|00|
     *            ^---------------------^
     *       dst         dst+length-1
     *
     * but we use a minimum of 3 here since the overhead of the code
     * to do word writes is substantial.
     */
    if (length < 3 * wsize)
    {
        while (length != 0)
        {
            *dst++ = Val;
            --length;
        }
        return (dst0);
    }

    if ((Wideval = (uint32_t)Val) != 0)     /* Fill the word. */
    {
        Wideval = ((Wideval << 24) | (Wideval << 16) | (Wideval << 8) | Wideval); /* u_int is 32 bits. */
    }

    /* Align destination by filling in bytes. */
    if ((t = (uint32_t)dst & wmask) != 0)
    {
        t = wsize - t;
        length -= t;
        do
        {
            *dst++ = Val;
        }
        while (--t != 0);
    }

    /* Fill words.  Length was >= 2*words so we know t >= 1 here. */
    t = length / wsize;
    do
    {
        *(uint32_t *)dst = Wideval;
        dst += wsize;
    }
    while (--t != 0);

    /* Mop up trailing bytes, if any. */
    t = length & wmask;
    if (t != 0)
        do
        {
            *dst++ = Val;
        }
        while (--t != 0);

    return (dst0);
}