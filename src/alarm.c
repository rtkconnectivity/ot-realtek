/*
 *  Copyright (c) 2016, The OpenThread Authors.
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

/**
 * @file
 *   This file implements the OpenThread platform abstraction for the alarm.
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform-bee.h"
#include "mac_driver.h"
#include "mac_driver_mpan.h"
#include "common/logging.hpp"
#include <openthread/config.h>
#include <openthread/platform/alarm-micro.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/diag.h>
#include <openthread/platform/time.h>

void BEE_AlarmInit(void)
{
}

volatile uint64_t micro_alarm_us[MAX_PAN_NUM];

void micro_handler(uint8_t pan_idx)
{
    if (pan_idx == 0) otSysEventSignalPending();
    else zbSysEventSignalPending();
}

uint32_t otPlatAlarmMicroGetNow(void)
{
    return (uint32_t)otPlatTimeGet();
}

void otPlatAlarmMicroStartAt(otInstance *aInstance, uint32_t t0, uint32_t dt)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    uint64_t now = otPlatTimeGet();
    uint64_t target_us = mac_ConvertT0AndDtTo64BitTime(t0, dt, &now);
    micro_alarm_us[pan_idx] = target_us;
    if (pan_idx == 0)
    {
        if (target_us > now)
        {
            target_us = target_us % MAX_BT_CLOCK_COUNTER;
            mac_SetBTClkUSInt(MAC_BT_TIMER1, target_us);
        }
        else
        {
            otTaskletsSignalPending(NULL);
        }
    }
#ifdef RT_PLATFORM_BB2ULTRA
    else
    {
        if (target_us > now)
        {
            target_us = target_us % MAX_BT_CLOCK_COUNTER;
            mac_SetBTClkUSInt(MAC_BT_TIMER5, target_us);
        }
        else
        {
            otTaskletsSignalPending(NULL);
        }
    }
#endif
}

void otPlatAlarmMicroStop(otInstance *aInstance)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);
    micro_alarm_us[pan_idx] = 0;
}

volatile uint64_t milli_alarm_us[MAX_PAN_NUM];

void milli_handler(uint8_t pan_idx)
{
    if (pan_idx == 0) otSysEventSignalPending();
    else zbSysEventSignalPending();
}

uint32_t otPlatAlarmMilliGetNow(void)
{
    return (uint32_t)(otPlatTimeGet() / 1000);
}

void otPlatAlarmMilliStartAt(otInstance *aInstance, uint32_t t0, uint32_t dt)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    uint64_t now = otPlatTimeGet();
    uint64_t target_us;
    uint64_t now_ms = now/1000;
    uint64_t target_ms = mac_ConvertT0AndDtTo64BitTime(t0, dt, &now_ms);

    if (target_ms > now_ms) target_us = now + (target_ms - now_ms)*1000;
    else target_us = target_ms*1000;

    milli_alarm_us[pan_idx] = target_us;
    if (pan_idx == 0)
    {
        if (target_ms > now_ms)
        {
            target_us = target_us % MAX_BT_CLOCK_COUNTER;
            mac_SetBTClkUSInt(MAC_BT_TIMER0, target_us);
        }
        else
        {
            otTaskletsSignalPending(NULL);
        }
    }
#ifdef RT_PLATFORM_BB2ULTRA
    else
    {
        if (target_ms > now_ms)
        {
            target_us = target_us % MAX_BT_CLOCK_COUNTER;
            mac_SetBTClkUSInt(MAC_BT_TIMER4, target_us);
        }
        else
        {
            otTaskletsSignalPending(NULL);
        }
    }
#endif
}

void otPlatAlarmMilliStop(otInstance *aInstance)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);
    milli_alarm_us[pan_idx] = 0;
}

void BEE_AlarmProcess(otInstance *aInstance, uint8_t pan_idx)
{
	uint64_t now = otPlatTimeGet();
	if (micro_alarm_us[pan_idx] > 0)
	{
		if (now >= micro_alarm_us[pan_idx])
		{
			micro_alarm_us[pan_idx] = 0;
			otPlatAlarmMicroFired(aInstance);
		}
	}

	if (milli_alarm_us[pan_idx] > 0)
	{
		if (now >= milli_alarm_us[pan_idx])
		{
			milli_alarm_us[pan_idx] = 0;
			otPlatAlarmMilliFired(aInstance);
		}
	}
}

#define XTAL_ACCURACY       40
uint16_t otPlatTimeGetXtalAccuracy(void)
{
    return XTAL_ACCURACY;
}

uint64_t otPlatTimeGet(void)
{
    return mac_GetCurrentMACTime();
}
