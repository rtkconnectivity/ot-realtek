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

#include <openthread/platform/misc.h>

#include "platform-bee.h"

#ifdef BUILD_MATTER

void otPlatReset(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    WDG_SystemReset(RESET_ALL, SW_RESET_APP_END);
}

#else

extern int gPlatformPseudoResetLevel;
extern int gPlatformPseudoResetLevel_zb;
extern uint8_t mpan_GetCurrentPANIdx(void);

void otPlatReset(otInstance *aInstance)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);
    if (pan_idx == 0)
    {
        gPlatformPseudoResetLevel++;
    }
    else
    {
        gPlatformPseudoResetLevel_zb++;
    }
    otTaskletsSignalPending(NULL);
}

#endif

otPlatResetReason otPlatGetResetReason(otInstance *aInstance)
{
    uint32_t rst_rsn;
    otPlatResetReason reason;
    OT_UNUSED_VARIABLE(aInstance);
    // TODO: Write me!
    rst_rsn = SW_RESET_APP_END;

    switch (rst_rsn)
    {
    case RESET_REASON_HW:
        reason = OT_PLAT_RESET_REASON_POWER_ON;
        break;
#ifdef RT_PLATFORM_BEE4
    case RESET_REASON_WDT_TIMEOUT:
#else
    case RESET_REASON_WDG_TIMEOUT:
#endif
        reason = OT_PLAT_RESET_REASON_WATCHDOG;
        break;

    case SW_RESET_APP_END:
        reason = OT_PLAT_RESET_REASON_SOFTWARE;
        break;

    default:
        reason = OT_PLAT_RESET_REASON_UNKNOWN;
        break;
    }

    return reason;
}

void otPlatWakeHost(void)
{
    // TODO: implement an operation to wake the host from sleep state.
}

