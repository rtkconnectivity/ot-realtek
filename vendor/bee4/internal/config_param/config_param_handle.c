/**
*********************************************************************************************************
*               Copyright(c) 2024, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file      config_param_handle.c
* @brief     This file handle config parameters flow.
* @details
* @author    mandy
* @date      2024-11-13
* @version   v1.0
*********************************************************************************************************
*/

/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include <trace.h>
#include "app_section.h"
#include "rtl_pinmux.h"
#include "zb_tst_cfg.h"
#include "board.h"
#include "ftl.h"
#include "wdt.h"
#include "os_timer.h"

#if (1 == BUILD_RCP)
/*============================================================================*
 *                                  Macros
 *============================================================================*/
typedef void *TimerHandle_t;
#define ENABLE_FLOW_CONTROL_TIMEOUT 100 //100ms
#define WRITE_CONFIG_PARAM_REBOOT_TIMEOUT 100 //100ms

/*============================================================================*
 *                              Local Variables
 *============================================================================*/
TimerHandle_t enable_flow_control_timer = NULL;
TimerHandle_t write_config_param_reboot_timer = NULL;

/*============================================================================*
 *                              Global Variables
 *============================================================================*/
rtk_config_param config_param;

/*============================================================================*
 *                              Functions Declaration
 *============================================================================*/
static void enable_flow_control_timer_callback(TimerHandle_t p_timer) DATA_RAM_FUNCTION;
static void write_config_param_reboot_timer_callback(TimerHandle_t p_timer) DATA_RAM_FUNCTION;

/*============================================================================*
 *                              local Functions
 *============================================================================*/
void set_default_config_param(void)
{
    APP_PRINT_INFO0("[set_default_config_param]");

    config_param.sign = 0x5A5AA5A5;
    config_param.vid = 0xFFFF;
    config_param.pid = 0xFFFF;
    config_param.tx_pin = P3_0;
    config_param.rx_pin = P3_1;
    config_param.rts_pin = P1_1;
    config_param.cts_pin = P1_0;
    config_param.func_msk = 0xFF;
    config_param.baud_rate = RCP_BAUDRATE_921600;
    config_param.pta_dis = 0xFF;
    config_param.pta_wl_act = P2_7;
    config_param.pta_bt_act = P2_6;
    config_param.pta_bt_stat = P2_5;
    config_param.pta_bt_clk = P2_4;
    config_param.ext_pa = 0xFF;//XI32K
    config_param.ext_lna = 0xFF;//XO32K
    config_param.ext_pa_lna_ploatiry = 0xFF;
    config_param.ic_type = RTK_RCP_IC_TYPE;
    config_param.reserved = 0xffff;
}

void print_config_param(rtk_config_param *config_param)
{
    APP_PRINT_INFO6("[print_config_param] sign=0x%x,vid=0x%x,pid=0x%x,tx_pin=0x%x,rx_pin=0x%x,rts_pin=0x%x",
                    \
                    config_param->sign, config_param->vid, config_param->pid, config_param->tx_pin,
                    config_param->rx_pin, config_param->rts_pin);

    APP_PRINT_INFO6("[print_config_param] cts_pin=0x%x,func_msk=0x%x,baud_rate=%d,pta_dis=0x%x,pta_wl_act=0x%x,pta_bt_act=0x%x",
                    \
                    config_param->cts_pin, config_param->func_msk, config_param->baud_rate, config_param->pta_dis,
                    config_param->pta_wl_act, config_param->pta_bt_act);

    APP_PRINT_INFO6("[print_config_param] pta_bt_stat=0x%x,pta_bt_clk=0x%x,ext_pa=0x%x,ext_lna=0x%x,ext_pa_lna_ploatiry=0x%x,ic_type=0x%x",
                    \
                    config_param->pta_bt_stat, config_param->pta_bt_clk, config_param->ext_pa, config_param->ext_lna,
                    config_param->ext_pa_lna_ploatiry, config_param->ic_type);
}

bool save_config_param(rtk_config_param *config_param)
{
    uint32_t result = false;

    //config parameters can not be modified
    config_param->sign = 0x5A5AA5A5;
    config_param->tx_pin = P3_0;
    config_param->rx_pin = P3_1;
    config_param->rts_pin = P1_1;
    config_param->cts_pin = P1_0;
    config_param->func_msk |= (1 << HW_FLOWCTL_OFFSET);
#ifdef BOARD_RTL8771GUV
    config_param->baud_rate = RCP_BAUDRATE_921600;
#endif
    config_param->pta_wl_act = P2_7;
    config_param->pta_bt_act = P2_6;
    config_param->pta_bt_stat = P2_5;
    config_param->pta_bt_clk = P2_4;
    config_param->reserved = 0xffff;

    result = ftl_save_to_module("app", config_param, FTL_CONFIG_PARAM_BASE_ADDR, FTL_CONFIG_PARAM_LEN);

    APP_PRINT_INFO1("[save_config_parama] result is %d", result);

    return (result == 0);
}

uint32_t check_config_param(rtk_config_param *config_data_to_check, bool pidvid_check_enable)
{
    uint32_t check_result = CONFIG_DATA_NO_ERROR;

    //check ic type
    if (RTK_RCP_IC_TYPE != config_data_to_check->ic_type)
    {
        check_result |= CONFIG_DATA_IC_TYPE_ERROR;
    }

#ifdef BOARD_RTL8771GTV
    //check baudrate
    switch (config_data_to_check->baud_rate)
    {
    case RCP_BAUDRATE_115200:
    case RCP_BAUDRATE_230400:
    case RCP_BAUDRATE_460800:
    case RCP_BAUDRATE_921600:
    case RCP_BAUDRATE_1000000:
    case RCP_BAUDRATE_2000000:
        break;

    default:
        check_result |= CONFIG_DATA_BAUDRATE_ERROR;
        break;
    }
#endif

    //check PID/VID
    if (true == pidvid_check_enable)
    {
        if (((config_param.vid != 0xffff) && (config_data_to_check->vid != config_param.vid)) ||
            ((config_param.pid != 0xffff) && (config_data_to_check->pid != config_param.pid)))
        {
            check_result |= CONFIG_DATA_PID_VID_ERROR;
        }
    }

    //check ext_pa PIN
    if ((0xFF != config_data_to_check->ext_pa) && (XI32K != config_data_to_check->ext_pa))
    {
        check_result |= CONFIG_DATA_PA_PIN_ERROR;
    }

    //check ext_lna PIN
    if ((0xFF != config_data_to_check->ext_lna) && (XO32K != config_data_to_check->ext_lna))
    {
        check_result |= CONFIG_DATA_LNA_PIN_ERROR;
    }

    APP_PRINT_INFO1("[check_config_param] check_result=0x%x", check_result);

    return check_result;
}

void load_config_param(void)
{
    APP_PRINT_INFO0("[load_config_param]");

    uint32_t ftl_res = 0;
    uint32_t check_result = CONFIG_DATA_NO_ERROR;

    ftl_res = ftl_load_from_module("app", &config_param, FTL_CONFIG_PARAM_BASE_ADDR,
                                   FTL_CONFIG_PARAM_LEN);

    if (0 != ftl_res)
    {
        APP_PRINT_WARN0("[load_config_param] config param are invalid, reset to default!");
        set_default_config_param();
        save_config_param(&config_param);
        ftl_res = ftl_load_from_module("app", &config_param, FTL_CONFIG_PARAM_BASE_ADDR,
                                       FTL_CONFIG_PARAM_LEN);
    }

    check_result = check_config_param(&config_param, false);
    if (CONFIG_DATA_NO_ERROR != check_result)
    {
        if ((check_result & CONFIG_DATA_IC_TYPE_ERROR))
        {
            APP_PRINT_WARN0("[load_config_param] ic type is invalid, reset to default!");
            config_param.ic_type = RTK_RCP_IC_TYPE;
        }
#ifdef BOARD_RTL8771GTV
        if ((check_result & CONFIG_DATA_BAUDRATE_ERROR))
        {
            APP_PRINT_WARN0("[load_config_param] baudrate is invalid, reset to default!");
            config_param.baud_rate = RCP_BAUDRATE_921600;
        }
#endif
        if ((check_result & CONFIG_DATA_PID_VID_ERROR))
        {
            APP_PRINT_WARN0("[load_config_param] pid/vid is invalid, reset to default!");
            config_param.vid = 0xffff;
            config_param.pid = 0xffff;
        }
        if ((check_result & CONFIG_DATA_PA_PIN_ERROR))
        {
            APP_PRINT_WARN0("[load_config_param] ext_pa pin is invalid, reset to default!");
            config_param.ext_pa = 0xff;
        }
        if ((check_result & CONFIG_DATA_LNA_PIN_ERROR))
        {
            APP_PRINT_WARN0("[load_config_param] ext_lna pin is invalid, reset to default!");
            config_param.ext_lna = 0xff;
        }
        save_config_param(&config_param);
    }

    print_config_param(&config_param);
}

void write_config_param_reboot_timer_callback(TimerHandle_t p_timer)
{
    APP_PRINT_INFO0("[write_config_param_reboot_timer_callback]");

    WDG_SystemReset(RESET_ALL_EXCEPT_AON, SWITCH_TO_TEST_MODE_BY_UART);
}

bool restart_write_config_param_reboot_timer(void)
{
    APP_PRINT_INFO0("[restart_write_config_param_reboot_timer]");

    bool result = false;

    if (write_config_param_reboot_timer == NULL)
    {
        if (false == os_timer_create(&write_config_param_reboot_timer, "write_config_param_reboot_timer",
                                     1, \
                                     WRITE_CONFIG_PARAM_REBOOT_TIMEOUT, false, write_config_param_reboot_timer_callback))
        {
            APP_PRINT_ERROR0("[restart_write_config_param_reboot_timer] init write_config_param_reboot_timer failed");
        }
        else
        {
            os_timer_restart(&write_config_param_reboot_timer, WRITE_CONFIG_PARAM_REBOOT_TIMEOUT);
            result = true;
        }
    }
    else
    {
        os_timer_restart(&write_config_param_reboot_timer, WRITE_CONFIG_PARAM_REBOOT_TIMEOUT);
        result = true;
    }

    return result;
}

void enable_flow_control_timer_callback(TimerHandle_t p_timer)
{
    APP_PRINT_INFO0("[enable_flow_control_timer_callback]");

#ifdef BOARD_RTL8771GTV
    config_param.func_msk &= ~(1 << HW_FLOWCTL_OFFSET);

    Pad_Config(config_param.rts_pin, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
               PAD_OUT_LOW);
    Pad_Config(config_param.cts_pin, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
               PAD_OUT_LOW);
    Pinmux_Config(config_param.rts_pin, ZB_CLI_UART_RTS);
    Pinmux_Config(config_param.cts_pin, ZB_CLI_UART_CTS);

    extern void uart_init_internal(void);
    uart_init_internal();
#endif
}

/*============================================================================*
 *                              Global Functions
 *============================================================================*/
bool rtk_write_config_param(const uint8_t *data, uint8_t data_len)
{
    APP_PRINT_INFO1("[rtk_write_config_param] data_len=%d", data_len);

    rtk_config_param *tmp_config_param = data;
    bool result = false;

    if (data_len != sizeof(rtk_config_param))
    {
        APP_PRINT_ERROR0("[rtk_enable_flow_control] data_len is wrong");
        return result;
    }

    print_config_param(tmp_config_param);

    if (CONFIG_DATA_NO_ERROR == check_config_param(tmp_config_param, true))
    {
        result = save_config_param(tmp_config_param);

        if (true == result)
        {
            result = restart_write_config_param_reboot_timer();
        }
    }

    return result;
}

bool rtk_enable_flow_control(const uint8_t *data, uint8_t data_len)
{
    APP_PRINT_INFO0("[rtk_enable_flow_control]");

    bool result = false;

#ifdef BOARD_RTL8771GUV
    return true;
#endif

    if ((1 != data_len) || (1 != data[0]))
    {
        APP_PRINT_ERROR2("[rtk_enable_flow_control] data_len=%d, data[0]=%d are wrong", data_len, data[0]);
        return result;
    }

    if (enable_flow_control_timer == NULL)
    {
        if (false == os_timer_create(&enable_flow_control_timer, "enable_flow_control_timer",  1, \
                                     ENABLE_FLOW_CONTROL_TIMEOUT, false, enable_flow_control_timer_callback))
        {
            APP_PRINT_ERROR0("[rtk_enable_flow_control] init enable_flow_control_timer failed");
        }
        else
        {
            os_timer_restart(&enable_flow_control_timer, ENABLE_FLOW_CONTROL_TIMEOUT);
            result = true;
        }
    }
    else
    {
        os_timer_restart(&enable_flow_control_timer, ENABLE_FLOW_CONTROL_TIMEOUT);
        result = true;
    }

    return result;
}

void mac_LoadConfigParam(void)
{
    load_config_param();

    // if (config_param.pta_dis >= ZB_PTA_MODE_INVALID)
    // {
    //     memset((void *)&zb_pta_cfg, 0, sizeof(zb_pta_cfg));
    //     zb_pta_cfg.zb_pta_cfg_b.en = 0;
    // }
    // else
    // {
    //     zb_pta_cfg.zb_pta_cfg_b.en = 1;
    //     zb_pta_cfg.zb_pta_cfg_b.mode = config_param.pta_dis;
    //     // load PTA pins configuration
    //     zb_pta_cfg.pta_pins.bt_act = config_param.pta_bt_act;
    //     zb_pta_cfg.pta_pins.bt_state = config_param.pta_bt_stat;
    //     zb_pta_cfg.pta_pins.bt_clk = config_param.pta_bt_clk;
    //     zb_pta_cfg.pta_pins.wl_act = config_param.pta_wl_act;
    // }
}

#endif

/******************* (C) COPYRIGHT 2024 Realtek Semiconductor Corporation *****END OF FILE****/
