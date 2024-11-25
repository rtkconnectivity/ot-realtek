/*
 *  Copyright (c) 2019, The OpenThread Authors.
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
 *   This file implements the OpenThread platform abstraction for radio communication.
 *
 */
#include "platform-bee.h"
#include "mac_driver.h"
#ifdef RT_PLATFORM_BB2ULTRA
#include "mac_reg_mask.h"
#else
#include "mac_driver_mpan.h"
#endif

#include <openthread-core-config.h>
#include <openthread/config.h>
#include <openthread/tasklet.h>

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <openthread/link.h>

#include "common/code_utils.hpp"
#include "common/logging.hpp"
#include "utils/code_utils.h"
#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
#include "utils/link_metrics.h"
#endif
#include "utils/mac_frame.h"
#include "utils/uart.h"
//#include <platform-config.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/alarm-micro.h>
#include <openthread/platform/diag.h>
#include <openthread/platform/radio.h>
#include <openthread/platform/time.h>

#include "openthread-system.h"

#include <openthread/random_noncrypto.h>
#include "dbg_printf.h"

// clang-format off
#define FCS_LEN               2

#define SHORT_ADDRESS_SIZE    2            ///< Size of MAC short address.
#define US_PER_MS             1000ULL      ///< Microseconds in millisecond.

#define ACK_REQUEST_OFFSET       1         ///< Byte containing Ack request bit (+1 for frame length byte).
#define ACK_REQUEST_BIT          (1 << 5)  ///< Ack request bit.
#define FRAME_PENDING_OFFSET     1         ///< Byte containing pending bit (+1 for frame length byte).
#define FRAME_PENDING_BIT        (1 << 4)  ///< Frame Pending bit.
#define SECURITY_ENABLED_OFFSET  1         ///< Byte containing security enabled bit (+1 for frame length byte).
#define SECURITY_ENABLED_BIT     (1 << 3)  ///< Security enabled bit.

#define RSSI_SETTLE_TIME_US   40           ///< RSSI settle time in microseconds.
#define SAFE_DELTA            1000         ///< A safe value for the `dt` parameter of delayed operations.

#if defined(__ICCARM__)
_Pragma("diag_suppress=Pe167")
#endif

enum
{
    SBEE2_RECEIVE_SENSITIVITY  = -100, // dBm
    SBEE2_MIN_CCA_ED_THRESHOLD = -94,  // dBm
};

// clang-format on
#define IEEE802154_MAX_LENGTH    (127)
#define MAC_FRAME_TX_HDR_LEN     (2)     // extra length of frame header for ASIC depend TX frame description
#define MAC_FRAME_RX_HDR_LEN     (1)     // extra length of rx frame header: frame length
#define MAC_FRAME_RX_TAIL_LEN    (7)     // extra length of frame trailer: LQI + RSSI
#define MAC_FRAME_IMMACK_LEN     (3)

#define RX_BUF_SIZE     16
#define EV_BUF_SIZE     16

#define NONE 0
#define RX_OK 1
#define TX_WAIT_ACK 2
#define TX_OK 3
#define TX_CCA_FAIL 4
#define TX_NO_ACK 5
#define ED_SCAN 6
#define ENTER_SLEEP 7

#define MAX_TRANSMIT_RETRY 16
#define MAX_TRANSMIT_AT_RETRY 4
#ifdef BUILD_MATTER
#define MAX_TRANSMIT_BC_RETRY 1 // 1 times retry for broadcast frame
#else
#define MAX_TRANSMIT_BC_RETRY 0
#endif

typedef struct
{
    otRadioFrame sReceivedFrames;
    uint8_t sReceivedPsdu[MAC_FRAME_RX_HDR_LEN + IEEE802154_MAX_LENGTH + MAC_FRAME_RX_TAIL_LEN];
} rx_item_t;

typedef struct
{
void *edscan_done;
bool sDisabled;
otRadioState sState;
uint8_t curr_channel;
uint8_t saved_channel;
uint16_t sPanid;
int8_t sPower;

uint16_t rx_tail;
uint16_t rx_head;
rx_item_t *rx_queue;

uint16_t ev_tail;
uint16_t ev_head;
uint8_t ev_queue[EV_BUF_SIZE];

uint32_t     sTransmitRetry;
uint32_t     sTransmitAtRetry;
uint32_t     sTransmitBcRetry;

uint32_t     sTransmitCcaFailCnt;
otRadioFrame sTransmitFrame;
uint8_t      sTransmitPsdu[MAC_FRAME_TX_HDR_LEN + IEEE802154_MAX_LENGTH];

#if OPENTHREAD_CONFIG_MAC_HEADER_IE_SUPPORT
otExtAddress  sExtAddress;
otRadioIeInfo sTransmitIeInfo;
#endif

// tx ack frame
bool sAckedWithFramePending;

// tx enhack frame
uint8_t sEnhAckPsdu[MAC_FRAME_TX_HDR_LEN + IEEE802154_MAX_LENGTH];
uint8_t enhack_frm_len;
bool    enhack_frm_sec_en;

int8_t   sLnaGain;
uint16_t sRegionCode;

bool sPendingSleep;
#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
uint32_t      sCslPeriod;
uint32_t      sCslSampleTime;
uint8_t       sCslIeIndex;
#endif // OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
uint8_t       enhAckProbingDataLen;
uint8_t       sVendorIeIndex;
#endif

uint32_t sEnergyDetectionTime;
uint8_t sEnergyDetectionChannel;

#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
uint32_t        sMacFrameCounter;
uint8_t         sKeyId;
otMacKeyMaterial sPrevKey;
otMacKeyMaterial sCurrKey;
otMacKeyMaterial sNextKey;
bool            sAckedWithSecEnhAck;
uint32_t        sAckFrameCounter;
uint8_t         sAckKeyId;
#endif
} radio_inst_t;
static radio_inst_t radio_inst[MAX_PAN_NUM];

void BEE_EventSend(uint8_t event, uint32_t pan_idx)
{
    radio_inst[pan_idx].ev_queue[radio_inst[pan_idx].ev_tail] = event;
    radio_inst[pan_idx].ev_tail = (radio_inst[pan_idx].ev_tail + 1) % EV_BUF_SIZE;
}

typedef struct
{
    uint16_t len: 7;
    uint16_t id: 8;
    uint16_t type: 1;
    uint16_t phase;
    uint16_t period;
} __attribute__((packed)) csl_ie_t;

typedef struct
{
    uint16_t len: 7;
    uint16_t id: 8;
    uint16_t type: 1;
    uint8_t oui[3];
    uint8_t subtype;
    uint8_t content[2];
} __attribute__((packed)) vendor_ie_t;

#ifdef RT_PLATFORM_BEE3PLUS
extern bool (*hw_sha256)(uint8_t *input, uint32_t byte_len, uint32_t *result, int mode);
#else
extern bool hw_sha256(uint8_t *input, uint32_t byte_len, uint32_t *result, int mode);
#endif
static uint8_t sha256_output[32];

static void dataInit(uint8_t pan_idx)
{
    os_sem_create(&radio_inst[pan_idx].edscan_done, "edscan", 0, 1);
    radio_inst[pan_idx].sDisabled = true;
    radio_inst[pan_idx].rx_queue = malloc(sizeof(rx_item_t) * RX_BUF_SIZE);

    radio_inst[pan_idx].sTransmitFrame.mPsdu = &radio_inst[pan_idx].sTransmitPsdu[MAC_FRAME_TX_HDR_LEN];
#if OPENTHREAD_CONFIG_MAC_HEADER_IE_SUPPORT
    radio_inst[pan_idx].sTransmitFrame.mInfo.mTxInfo.mIeInfo = &radio_inst[pan_idx].sTransmitIeInfo;
#endif

    hw_sha256(get_ic_euid(), 14, sha256_output, 0);
}

#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
static void txAckProcessSecurity(uint8_t *aAckFrame, uint8_t pan_idx)
{
    //otRadioFrame     ackFrame;
    otMacKeyMaterial *key = NULL;
    uint8_t          keyId;

    struct
    {
        uint8_t sec_level;
        uint32_t frame_counter;
        uint64_t src_ext_addr;
    } __attribute__((packed)) nonce;

    radio_inst[pan_idx].sAckedWithSecEnhAck = false;
    //otEXPECT(aAckFrame[SECURITY_ENABLED_OFFSET] & SECURITY_ENABLED_BIT);
    otEXPECT(mac_GetRxFrmSecEn());

    //memset(&ackFrame, 0, sizeof(ackFrame));
    //ackFrame.mPsdu   = &aAckFrame[1];
    //ackFrame.mLength = aAckFrame[0];

    //keyId = otMacFrameGetKeyId(&ackFrame);
    keyId = mac_GetRxFrmSecKeyId();

    //otEXPECT(otMacFrameIsKeyIdMode1(&ackFrame) && keyId != 0);
    otEXPECT(0x01 == mac_GetRxFrmSecKeyIdMode() && keyId != 0);

    if (keyId == radio_inst[pan_idx].sKeyId)
    {
        key = &radio_inst[pan_idx].sCurrKey;
    }
    else if (keyId == radio_inst[pan_idx].sKeyId - 1)
    {
        key = &radio_inst[pan_idx].sPrevKey;
    }
    else if (keyId == radio_inst[pan_idx].sKeyId + 1)
    {
        key = &radio_inst[pan_idx].sNextKey;
    }
    else
    {
        otEXPECT(false);
    }

    radio_inst[pan_idx].sAckFrameCounter    = radio_inst[pan_idx].sMacFrameCounter;
    radio_inst[pan_idx].sAckKeyId           = keyId;
    radio_inst[pan_idx].sAckedWithSecEnhAck = true;

    //ackFrame.mInfo.mTxInfo.mAesKey = key;

    //otMacFrameSetKeyId(&ackFrame, keyId);
    //otMacFrameSetFrameCounter(&ackFrame, sMacFrameCounter++);

    //otMacFrameProcessTransmitAesCcm(&ackFrame, &sExtAddress);
    nonce.sec_level = mac_GetRxFrmSecLevel();
    nonce.frame_counter = radio_inst[pan_idx].sMacFrameCounter;
    mac_memcpy(&nonce.src_ext_addr, mpan_GetLongAddress(pan_idx), sizeof(nonce.src_ext_addr));

    // set nonce
    mac_LoadNonce((uint8_t *)&nonce);
    // set key
    mac_LoadTxEnhAckKey(key->mKeyMaterial.mKey.m8);
    // set security level
    mac_SetTxEnhAckChiper(nonce.sec_level);
exit:
    return;
}
#endif // OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2

#if !OPENTHREAD_CONFIG_ENABLE_PLATFORM_EUI64_CUSTOM_SOURCE
void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
    memcpy(aIeeeEui64, sha256_output, sizeof(uint64_t));
}
#endif // OPENTHREAD_CONFIG_ENABLE_PLATFORM_EUI64_CUSTOM_SOURCE

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t aPanid)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);
    otLogInfoPlat("PANID=%X", aPanid);
    radio_inst[pan_idx].sPanid = aPanid;
    mpan_SetPANId(aPanid, pan_idx);
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, const otExtAddress *aExtAddress)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);
    otLogInfoPlat("ExtAddr=%X%X%X%X%X%X%X%X", aExtAddress->m8[7], aExtAddress->m8[6],
                  aExtAddress->m8[5], aExtAddress->m8[4],
                  aExtAddress->m8[3], aExtAddress->m8[2], aExtAddress->m8[1], aExtAddress->m8[0]);
#if OPENTHREAD_CONFIG_MAC_HEADER_IE_SUPPORT
    for (size_t i = 0; i < sizeof(*aExtAddress); i++)
    {
        radio_inst[pan_idx].sExtAddress.m8[i] = aExtAddress->m8[sizeof(*aExtAddress) - 1 - i];
    }
#endif
    mpan_SetLongAddress((u8_t *)aExtAddress->m8, pan_idx);
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t aShortAddress)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);
    otLogInfoPlat("ShortAddr=%X", aShortAddress);
    mpan_SetShortAddress(aShortAddress, pan_idx);
}

extern void (*modem_set_zb_cca_energy_detect_threshold)(int8_t thres, uint8_t res);
extern mac_attribute_t attr;
extern mac_driver_t drv;

void txn_handler(uint8_t pan_idx, uint32_t arg);
void rxely_handler(uint8_t pan_idx, uint32_t arg);
void rxdone_handler(uint8_t pan_idx, uint32_t arg);
void edscan_handler(uint8_t pan_idx, uint32_t arg);
void btcmp0_handler(uint8_t pan_idx, uint32_t arg);
void btcmp1_handler(uint8_t pan_idx, uint32_t arg);
#ifdef RT_PLATFORM_BB2ULTRA
void btcmp4_handler(uint8_t pan_idx, uint32_t arg);
void btcmp5_handler(uint8_t pan_idx, uint32_t arg);
#endif

void BEE_RadioInit(uint8_t pan_idx)
{
    mac_SetTxNCsma(false);
    mac_SetTxNMaxRetry(0);

    if (pan_idx == 0)
    {
        mpan_RegisterISR(0, txn_handler, rxely_handler, rxdone_handler, edscan_handler);
        mpan_RegisterTimer(0, MAC_BT_TIMER0, btcmp0_handler, 0);
        mpan_RegisterTimer(0, MAC_BT_TIMER1, btcmp1_handler, 0);
        mpan_EnableCtl(0, 1);
        dataInit(0);
    }
#ifdef RT_PLATFORM_BB2ULTRA
    else
    {
        mpan_RegisterISR(1, txn_handler, rxely_handler, rxdone_handler, edscan_handler);
        mpan_RegisterTimer(1, MAC_BT_TIMER4, btcmp4_handler, 0);
        mpan_RegisterTimer(1, MAC_BT_TIMER5, btcmp5_handler, 0);
        mpan_EnableCtl(1, 1);
        dataInit(1);
    }
#endif
}

otRadioState otPlatRadioGetState(otInstance *aInstance)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);

    if (radio_inst[pan_idx].sDisabled)
    {
        return OT_RADIO_STATE_DISABLED;
    }

    return radio_inst[pan_idx].sState; // It is the default state. Return it in case of unknown.
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);
    return (radio_inst[pan_idx].sState != OT_RADIO_STATE_DISABLED) ? true : false;
}

otError otPlatRadioEnable(otInstance *aInstance)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    if (!otPlatRadioIsEnabled(aInstance))
    {
        // TODO: enable radio and enter sleep mode
        otLogInfoPlat("State=OT_RADIO_STATE_SLEEP");
        radio_inst[pan_idx].sState = OT_RADIO_STATE_SLEEP;
    }

    return OT_ERROR_NONE;
}

otError otPlatRadioDisable(otInstance *aInstance)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    if (otPlatRadioIsEnabled(aInstance))
    {
        otLogInfoPlat("State=OT_RADIO_STATE_DISABLED");
        radio_inst[pan_idx].sState = OT_RADIO_STATE_DISABLED;
        // TODO: radio disable
    }

    return OT_ERROR_NONE;
}

otError otPlatRadioSleep(otInstance *aInstance)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);
    otError error = OT_ERROR_INVALID_STATE;

    if (radio_inst[pan_idx].sState != OT_RADIO_STATE_DISABLED)
    {
        error  = OT_ERROR_NONE;
        radio_inst[pan_idx].sState = OT_RADIO_STATE_SLEEP;
        // TODO: radio enter sleep
        mac_SetRadioState(MAC_RADIO_STATE_RX);
        radio_inst[pan_idx].sPendingSleep = true;
        BEE_SleepProcess(aInstance, pan_idx);
    }

    return error;
}

void setChannel(uint8_t aChannel, uint8_t pan_idx)
{
    if (radio_inst[pan_idx].curr_channel != aChannel)
    {
        radio_inst[pan_idx].saved_channel = radio_inst[pan_idx].curr_channel;
        radio_inst[pan_idx].curr_channel = aChannel;
        mac_SetChannel(aChannel);
    }
}

otError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);
    otError error = OT_ERROR_INVALID_STATE;

    if (radio_inst[pan_idx].sState != OT_RADIO_STATE_DISABLED)
    {
        error  = OT_ERROR_NONE;
        radio_inst[pan_idx].sState = OT_RADIO_STATE_RECEIVE;
        setChannel(aChannel, pan_idx);
        // TODO: radio enter RX ready
    }
    return error;
}

#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
otError otPlatRadioReceiveAt(otInstance *aInstance, uint8_t aChannel, uint32_t aStart,
                             uint32_t aDuration)
{
    return OT_ERROR_NONE;
}
#endif

void BEE_tx_started(uint8_t *p_data, uint8_t pan_idx);
void txProcessSecurity(uint8_t *aFrame, uint8_t pan_idx);
extern uint8_t otMacFrameGetHeaderLength(otRadioFrame *aFrame);
extern uint16_t otMacFrameGetPayloadLength(otRadioFrame *aFrame);
extern uint8_t otMacFrameGetSecurityLevel(otRadioFrame *aFrame);
otError otPlatRadioTransmit(otInstance *aInstance, otRadioFrame *aFrame)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    otError error  = OT_ERROR_NONE;
    uint8_t ret;
    uint64_t now;
    uint64_t target_us;
    uint64_t diff;

    if (radio_inst[pan_idx].sState == OT_RADIO_STATE_DISABLED)
    {
        return OT_ERROR_INVALID_STATE;
    }

    mpan_tx_lock(pan_idx);
    radio_inst[pan_idx].sState = OT_RADIO_STATE_TRANSMIT;

    if (aFrame->mInfo.mTxInfo.mTxDelay > 0)
    {
        now = otPlatTimeGet();
        target_us = mac_ConvertT0AndDtTo64BitTime(aFrame->mInfo.mTxInfo.mTxDelayBaseTime,
                                                  aFrame->mInfo.mTxInfo.mTxDelay - 128,
                                                  &now);
        if (target_us > now) diff = target_us - now;
        else diff = 0;
    }

#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
    if (otMacFrameIsSecurityEnabled(aFrame) && otMacFrameIsKeyIdMode1(aFrame) &&
        !aFrame->mInfo.mTxInfo.mIsARetx)
    {
        otMacFrameSetKeyId(aFrame, radio_inst[pan_idx].sKeyId);
        otMacFrameSetFrameCounter(aFrame, radio_inst[pan_idx].sMacFrameCounter++);
    }

    BEE_tx_started(NULL, pan_idx);
#endif
    if (otMacFrameIsSecurityEnabled(aFrame) && otMacFrameIsKeyIdMode1(aFrame))
    {
        if (!aFrame->mInfo.mTxInfo.mIsSecurityProcessed)
        {
            mac_LoadTxNPayload(otMacFrameGetHeaderLength(aFrame),
                               otMacFrameGetHeaderLength(aFrame) + otMacFrameGetPayloadLength(aFrame), &aFrame->mPsdu[0]);
            txProcessSecurity(NULL, pan_idx);
            mac_TrigUpperEnc();
            aFrame->mInfo.mTxInfo.mIsSecurityProcessed = true;
        }
    }
    else
    {
        mac_LoadTxNPayload(0, aFrame->mLength - FCS_LEN, &aFrame->mPsdu[0]);
    }

    setChannel(aFrame->mChannel, pan_idx);
    if (aFrame->mInfo.mTxInfo.mTxDelay > 0)
    {
        radio_inst[pan_idx].sTransmitCcaFailCnt = 0;
        radio_inst[pan_idx].sTransmitAtRetry = 0;
        radio_inst[pan_idx].sTransmitBcRetry = 0;
        if (diff)
        {
            now = otPlatTimeGet();
            target_us = now + diff;
            do
            {
                now = otPlatTimeGet();
            }
            while (now < target_us);
            mac_TrigTxN(otMacFrameIsAckRequested(aFrame), false);
        }
        else
        {
            mac_TrigTxN(otMacFrameIsAckRequested(aFrame), false);
            dbg_printf("TXAT %u expired %llu ", otMacFrameGetSequence(aFrame), now - target_us);
        }
    }
    else
    {
        radio_inst[pan_idx].sTransmitCcaFailCnt = 0;
        radio_inst[pan_idx].sTransmitRetry = 0;
        radio_inst[pan_idx].sTransmitBcRetry = 0;
        now = otPlatTimeGet();
        target_us = now + mac_BackoffDelay();
        do
        {
            now = otPlatTimeGet();
        }
        while (now < target_us);
        mac_TrigTxN(otMacFrameIsAckRequested(aFrame), false);
    }

    otPlatRadioTxStarted(aInstance, aFrame);
    return error;
}

otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance *aInstance)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);

    return &radio_inst[pan_idx].sTransmitFrame;
}

int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    uint8_t rssi_raw;
    int8_t rssi;

    mpan_tx_lock(pan_idx);
    setChannel(radio_inst[pan_idx].curr_channel, pan_idx);

    mac_EDScan_begin(1);
    os_sem_take(radio_inst[pan_idx].edscan_done, 0xffffffff);
    mac_EDScan_end(&rssi_raw);
    rssi = drv.mac_edscan_lv2dbm_cb(rssi_raw);

    setChannel(radio_inst[pan_idx].saved_channel, pan_idx);
    mpan_tx_unlock();

    return rssi;
}

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    otRadioCaps caps = (OT_RADIO_CAPS_CSMA_BACKOFF | OT_RADIO_CAPS_ACK_TIMEOUT | OT_RADIO_CAPS_SLEEP_TO_TX |
                        OT_RADIO_CAPS_TRANSMIT_RETRIES |
#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
                        OT_RADIO_CAPS_TRANSMIT_SEC | OT_RADIO_CAPS_TRANSMIT_TIMING |
#endif
                        OT_RADIO_CAPS_ENERGY_SCAN);

    return caps;
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    if (mac_GetPromiscuous()) { return true; }
    else { return false; }
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{
    OT_UNUSED_VARIABLE(aInstance);
    mac_SetPromiscuous(aEnable);
}

void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);
    otLogDebgPlat("EnableSrcMatch=%d", aEnable ? 1 : 0);
    if (aEnable) { mpan_SetAddrMatchMode(AUTO_ACK_PENDING_MODE_THREAD, pan_idx); }
    else { mpan_SetAddrMatchMode(AUTO_ACK_PENDING_MODE_ZIGBEE, pan_idx); }
}

otError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance, uint16_t aShortAddress)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);

    if (MAC_STS_HW_LIMIT == mpan_AddSrcShortAddrMatch(aShortAddress, radio_inst[pan_idx].sPanid, pan_idx))
    {
        otLogInfoPlat("ShortEntry full!");
        return OT_ERROR_NO_BUFS;
    }
    else
    {
        otLogDebgPlat("add %x", aShortAddress);
        return OT_ERROR_NONE;
    }
}

otError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);

    if (MAC_STS_HW_LIMIT == mpan_AddSrcExtAddrMatch((uint8_t *)aExtAddress->m8, pan_idx))
    {
        otLogInfoPlat("ExtEntry full!");
        return OT_ERROR_NO_BUFS;
    }
    else
    {
        otLogDebgPlat("add %02x%02x%02x%02x%02x%02x%02x%02x",
                      aExtAddress->m8[7], aExtAddress->m8[6], aExtAddress->m8[5], aExtAddress->m8[4], aExtAddress->m8[3],
                      aExtAddress->m8[2], aExtAddress->m8[1], aExtAddress->m8[0]);
        return OT_ERROR_NONE;
    }
}

otError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance, uint16_t aShortAddress)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);

    if (MAC_STS_FAILURE == mpan_DelSrcShortAddrMatch(aShortAddress, radio_inst[pan_idx].sPanid, pan_idx))
    {
        otLogInfoPlat("%x not found!", aShortAddress);
        return OT_ERROR_NO_ADDRESS;
    }
    else
    {
        otLogDebgPlat("clear %x", aShortAddress);
        return OT_ERROR_NONE;
    }
}

otError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);

    if (MAC_STS_FAILURE == mpan_DelSrcExtAddrMatch((uint8_t *)aExtAddress->m8, pan_idx))
    {
        otLogInfoPlat("%02x%02x%02x%02x%02x%02x%02x%02x not found!",
                      aExtAddress->m8[7], aExtAddress->m8[6], aExtAddress->m8[5], aExtAddress->m8[4],
                      aExtAddress->m8[3], aExtAddress->m8[2], aExtAddress->m8[1], aExtAddress->m8[0]);
        return OT_ERROR_NO_ADDRESS;
    }
    else
    {
        otLogDebgPlat("clear %02x%02x%02x%02x%02x%02x%02x%02x",
                      aExtAddress->m8[7], aExtAddress->m8[6], aExtAddress->m8[5], aExtAddress->m8[4],
                      aExtAddress->m8[3], aExtAddress->m8[2], aExtAddress->m8[1], aExtAddress->m8[0]);
        return OT_ERROR_NONE;
    }
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);

    mpan_DelAllSrcAddrMatch(pan_idx);
}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);

    mpan_DelAllSrcAddrMatch(pan_idx);
}

otError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);

    radio_inst[pan_idx].sEnergyDetectionTime = (uint32_t)aScanDuration * 1000UL;
    radio_inst[pan_idx].sEnergyDetectionChannel = aScanChannel;

    BEE_EventSend(ED_SCAN, pan_idx);
    if (pan_idx == 0) otTaskletsSignalPending(NULL);
    else zbTaskletsSignalPending(NULL);
    return OT_ERROR_NONE;
}

otError otPlatRadioGetTransmitPower(otInstance *aInstance, int8_t *aPower)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);

    otError error = OT_ERROR_NONE;

    if (aPower == NULL)
    {
        error = OT_ERROR_INVALID_ARGS;
    }
    else
    {
        *aPower = radio_inst[pan_idx].sPower;
    }

    return error;
}

otError otPlatRadioSetTransmitPower(otInstance *aInstance, int8_t aPower)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aPower);
    int8_t actual_power;

    otError error   = OT_ERROR_NONE;
    radio_inst[pan_idx].sPower = aPower;
    actual_power = mac_SetTXPower_patch(aPower);
    otLogInfoPlat("tx_power %d actual_power %d", aPower, actual_power);
    return error;
}

otError otPlatRadioGetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t *aThreshold)
{
    OT_UNUSED_VARIABLE(aInstance);

    otError              error = OT_ERROR_NONE;

    if (aThreshold == NULL)
    {
        error = OT_ERROR_INVALID_ARGS;
    }
    else
    {
        *aThreshold = mac_GetCcaEDThreshold();
    }

    return error;
}

otError otPlatRadioSetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t aThreshold)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);

    otError              error = OT_ERROR_NONE;

    aThreshold += radio_inst[pan_idx].sLnaGain;

    // The minimum value of ED threshold for radio driver is -94 dBm
    if (aThreshold < SBEE2_MIN_CCA_ED_THRESHOLD)
    {
        error = OT_ERROR_INVALID_ARGS;
    }
    else
    {
        modem_set_zb_cca_energy_detect_threshold(aThreshold, 1 /* ED_RES_0P50 */);
    }

    return error;
}

otError otPlatRadioGetFemLnaGain(otInstance *aInstance, int8_t *aGain)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);

    otError error = OT_ERROR_NONE;

    if (aGain == NULL)
    {
        error = OT_ERROR_INVALID_ARGS;
    }
    else
    {
        *aGain = radio_inst[pan_idx].sLnaGain;
    }

    return error;
}

otError otPlatRadioSetFemLnaGain(otInstance *aInstance, int8_t aGain)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);

    int8_t  threshold;
    int8_t  oldLnaGain = radio_inst[pan_idx].sLnaGain;
    otError error      = OT_ERROR_NONE;

    error = otPlatRadioGetCcaEnergyDetectThreshold(aInstance, &threshold);
    otEXPECT(error == OT_ERROR_NONE);

    radio_inst[pan_idx].sLnaGain = aGain;
    error    = otPlatRadioSetCcaEnergyDetectThreshold(aInstance, threshold);
    otEXPECT_ACTION(error == OT_ERROR_NONE, radio_inst[pan_idx].sLnaGain = oldLnaGain);

exit:
    return error;
}

extern void (*zbmac_pm_init)(zbpm_adapter_t *padapter);

static bool zbpm_inited = false;
static zbpm_adapter_t zbpm_adap;

static void zbpm_enter(void)
{
    otLogInfoPlat("%s", __func__);
}

static void zbpm_exit(void)
{
    otLogInfoPlat("%u %u", otPlatAlarmMicroGetNow(), zbpm_adap.wakeup_time_us);
}

extern volatile uint64_t micro_alarm_us[MAX_PAN_NUM];
extern volatile uint64_t milli_alarm_us[MAX_PAN_NUM];

void BEE_SleepProcess(otInstance *aInstance, uint8_t pan_idx)
{
    zbpm_adapter_t *padapter = &zbpm_adap;
    if (radio_inst[pan_idx].sPendingSleep)
    {
        radio_inst[pan_idx].sPendingSleep = false;
#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
        if (radio_inst[pan_idx].sCslPeriod > 0)
        {
            //otLogInfoPlat("sPendingSleep %u", micro_alarm_us[pan_idx]);
            padapter->power_mode = ZBMAC_DEEP_SLEEP;
            padapter->wakeup_reason = ZBMAC_PM_WAKEUP_UNKNOWN;
            padapter->error_code = ZBMAC_PM_ERROR_UNKNOWN;

            padapter->stage_time[ZBMAC_PM_CHECK] = 20;
            padapter->stage_time[ZBMAC_PM_STORE] = 15;
            padapter->stage_time[ZBMAC_PM_ENTER] = 5;
            padapter->stage_time[ZBMAC_PM_EXIT] = 5;
            padapter->stage_time[ZBMAC_PM_RESTORE] = 20;
            padapter->minimum_sleep_time = 20;
            padapter->learning_guard_time = 7; // 3 (learning guard time) + 4 (two 16k po_intr drift)

            padapter->cfg.wake_interval_en = 0;
            padapter->cfg.stage_time_learned = 0;
            padapter->wakeup_time_us = micro_alarm_us[pan_idx];
            padapter->wakeup_interval_us = 0;

            //padapter->enter_callback = zbpm_enter;
            //padapter->exit_callback = zbpm_exit;

            if (!zbpm_inited)
            {
                zbpm_inited = true;
                zbmac_pm_init(padapter);
            }
        }
#endif
    }
}

void BEE_EventProcess(otInstance *aInstance, uint8_t pan_idx)
{
    uint8_t ed_value_raw;
    int8_t ed_value;
    uint16_t curr_ev_tail;
    uint8_t event;
    rx_item_t *rx_item;
    uint64_t now;
    uint64_t target_us;

    curr_ev_tail = radio_inst[pan_idx].ev_tail;
    while (radio_inst[pan_idx].ev_head != curr_ev_tail)
    {
        event = radio_inst[pan_idx].ev_queue[radio_inst[pan_idx].ev_head];
        radio_inst[pan_idx].ev_head = (radio_inst[pan_idx].ev_head + 1) % EV_BUF_SIZE;

        switch (event)
        {
            case RX_OK:
            {
                rx_item = &radio_inst[pan_idx].rx_queue[radio_inst[pan_idx].rx_head];
                if (otMacFrameIsAck(&rx_item->sReceivedFrames))
                {
                    otPlatRadioTxDone(aInstance, &radio_inst[pan_idx].sTransmitFrame, &rx_item->sReceivedFrames, OT_ERROR_NONE);
                    radio_inst[pan_idx].rx_head = (radio_inst[pan_idx].rx_head + 1) % RX_BUF_SIZE;
                    mpan_tx_unlock();
                }
                else
                {
                    otPlatRadioReceiveDone(aInstance, &rx_item->sReceivedFrames, OT_ERROR_NONE);
                    radio_inst[pan_idx].rx_head = (radio_inst[pan_idx].rx_head + 1) % RX_BUF_SIZE;
                }
            }
            break;

            case TX_OK:
            {
                if (radio_inst[pan_idx].sTransmitBcRetry == MAX_TRANSMIT_BC_RETRY)
                {
                    otPlatRadioTxDone(aInstance, &radio_inst[pan_idx].sTransmitFrame, NULL, OT_ERROR_NONE);
                    mpan_tx_unlock();
                }
                else
                {
                    radio_inst[pan_idx].sTransmitBcRetry++;
                    if (radio_inst[pan_idx].sTransmitFrame.mInfo.mTxInfo.mTxDelay > 0)
                    {
                        mac_TrigTxN(otMacFrameIsAckRequested(&radio_inst[pan_idx].sTransmitFrame), false);
                    }
                    else
                    {
                        now = otPlatTimeGet();
                        target_us = now + mac_BackoffDelay();
                        do
                        {
                            now = otPlatTimeGet();
                        }
                        while (now < target_us);
                        mac_TrigTxN(otMacFrameIsAckRequested(&radio_inst[pan_idx].sTransmitFrame), false);
                    }
                }
            }
            break;

            case TX_CCA_FAIL:
            {
                mac_PTA_Wrokaround();
                radio_inst[pan_idx].sTransmitCcaFailCnt++;
                if (radio_inst[pan_idx].sTransmitFrame.mInfo.mTxInfo.mTxDelay > 0)
                {
                    mac_TrigTxN(otMacFrameIsAckRequested(&radio_inst[pan_idx].sTransmitFrame), false);
                }
                else
                {
                    now = otPlatTimeGet();
                    target_us = now + mac_BackoffDelay();
                    do
                    {
                        now = otPlatTimeGet();
                    }
                    while (now < target_us);
                    mac_TrigTxN(otMacFrameIsAckRequested(&radio_inst[pan_idx].sTransmitFrame), false);
                }
            }
            break;

            case TX_NO_ACK:
            {
                mac_PTA_Wrokaround();
                if (radio_inst[pan_idx].sTransmitFrame.mInfo.mTxInfo.mTxDelay > 0)
                {
                    otLogDebgPlat("TXAT_NO_ACK %u %u cca_fail %u retry %u", otMacFrameGetSequence(&radio_inst[pan_idx].sTransmitFrame), radio_inst[pan_idx].sTransmitFrame.mLength - FCS_LEN, radio_inst[pan_idx].sTransmitCcaFailCnt, radio_inst[pan_idx].sTransmitAtRetry);
                    otPlatRadioTxDone(aInstance, &radio_inst[pan_idx].sTransmitFrame, NULL, OT_ERROR_NO_ACK);
                    mpan_tx_unlock();
                }
                else
                {
                    radio_inst[pan_idx].sTransmitRetry++;
                    if (radio_inst[pan_idx].sTransmitRetry == MAX_TRANSMIT_RETRY)
                    {
                        otLogDebgPlat("TX_NO_ACK %u %u cca_fail %u retry %u", otMacFrameGetSequence(&radio_inst[pan_idx].sTransmitFrame), radio_inst[pan_idx].sTransmitFrame.mLength - FCS_LEN, radio_inst[pan_idx].sTransmitCcaFailCnt, radio_inst[pan_idx].sTransmitRetry);
                        otPlatRadioTxDone(aInstance, &radio_inst[pan_idx].sTransmitFrame, NULL, OT_ERROR_NO_ACK);
                        mpan_tx_unlock();
                    }
                    else
                    {
                        now = otPlatTimeGet();
                        target_us = now + mac_BackoffDelay();
                        do
                        {
                            now = otPlatTimeGet();
                        }
                        while (now < target_us);
                        mac_TrigTxN(otMacFrameIsAckRequested(&radio_inst[pan_idx].sTransmitFrame), false);
                    }
                }
            }
            break;

            case ED_SCAN:
            {
                mpan_tx_lock(pan_idx);
                setChannel(radio_inst[pan_idx].sEnergyDetectionChannel, pan_idx);

                mac_EDScan_begin(radio_inst[pan_idx].sEnergyDetectionTime / 128);
                os_sem_take(radio_inst[pan_idx].edscan_done, 0xffffffff);
                mac_EDScan_end(&ed_value_raw);
                ed_value = drv.mac_edscan_lv2dbm_cb(ed_value_raw);

                setChannel(radio_inst[pan_idx].saved_channel, pan_idx);
                otPlatRadioEnergyScanDone(aInstance, ed_value);
                mpan_tx_unlock();
            }
            break;

            case ENTER_SLEEP:
            {
                //BEE_SleepProcess(aInstance, pan_idx);
            }
            break;

            default:
            break;
        }
    }
}

typedef struct _fc_t
{
    uint16_t type: 3;
    uint16_t sec_en: 1;
    uint16_t pending: 1;
    uint16_t ack_req: 1;
    uint16_t panid_compress: 1;
    uint16_t rsv: 1;
    uint16_t seq_num_suppress: 1;
    uint16_t ie_present: 1;
    uint16_t dst_addr_mode: 2;
    uint16_t ver: 2;
    uint16_t src_addr_mode: 2;
} fc_t;

#define FRAME_TYPE_BEACON   0
#define FRAME_TYPE_DATA     1
#define FRAME_TYPE_ACK      2
#define FRAME_TYPE_COMMAND  3

#define ADDR_MODE_NOT_PRESENT   0
#define ADDR_MODE_RSV           1
#define ADDR_MODE_SHORT         2
#define ADDR_MODE_EXTEND        3

#define FRAME_VER_2003 0
#define FRAME_VER_2006 1
#define FRAME_VER_2015 2

#define SEC_NONE        0
#define SEC_MIC_32      1
#define SEC_MIC_64      2
#define SEC_MIC_128     3
#define SEC_ENC         4
#define SEC_ENC_MIC_32  5
#define SEC_ENC_MIC_64  6
#define SEC_ENC_MIC_128 7

#define KEY_ID_MODE_0  0
#define KEY_ID_MODE_1  1
#define KEY_ID_MODE_2  2
#define KEY_ID_MODE_3  3

typedef struct _aux_sec_ctl_t
{
    uint8_t sec_level: 3;
    uint8_t key_id_mode: 2;
    uint8_t frame_counter_supp: 1;
    uint8_t asn_in_nonce: 1;
    uint8_t rsv: 1;
} aux_sec_ctl_t;

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
static uint16_t getCslPhase(uint8_t pan_idx)
{
    uint32_t curTime       = otPlatAlarmMicroGetNow();
    uint32_t cslPeriodInUs = radio_inst[pan_idx].sCslPeriod * OT_US_PER_TEN_SYMBOLS;
    uint32_t diff = (cslPeriodInUs - (curTime % cslPeriodInUs) + (radio_inst[pan_idx].sCslSampleTime % cslPeriodInUs)) %
                    cslPeriodInUs;
    return (uint16_t)(diff / OT_US_PER_TEN_SYMBOLS + 1);
}
#endif

void BEE_tx_ack_started(uint8_t *p_data, int8_t power, uint8_t lqi, uint8_t pan_idx)
{
    otRadioFrame ackFrame;
    csl_ie_t *p_csl_ie;
    vendor_ie_t *p_vendor_ie;
#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
    uint8_t      linkMetricsDataLen = 0;
    uint8_t      linkMetricsData[OT_ENH_PROBING_IE_DATA_MAX_SIZE];
    otMacAddress macAddress;
#else
    OT_UNUSED_VARIABLE(power);
    OT_UNUSED_VARIABLE(lqi);
#endif

    ackFrame.mPsdu   = &p_data[1];
    ackFrame.mLength = p_data[0];

    // Check if the frame pending bit is set in ACK frame.
    //sEnhAckdWithFramePending = p_data[FRAME_PENDING_OFFSET] & FRAME_PENDING_BIT;

#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
    // Update IE and secure Enh-ACK.
#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
    if (radio_inst[pan_idx].sCslPeriod > 0)
    {
        p_csl_ie = (csl_ie_t *)&radio_inst[pan_idx].sEnhAckPsdu[MAC_FRAME_TX_HDR_LEN + radio_inst[pan_idx].sCslIeIndex];
        p_csl_ie->phase = getCslPhase(pan_idx);
        p_csl_ie->period = radio_inst[pan_idx].sCslPeriod;
    }
#endif

#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
    if (radio_inst[pan_idx].enhAckProbingDataLen > 0)
    {
        otMacFrameGetDstAddr(&ackFrame, &macAddress);
        if ((linkMetricsDataLen = otLinkMetricsEnhAckGenData(&macAddress, lqi, power, linkMetricsData)) > 0)
        {
            p_vendor_ie = (vendor_ie_t *)&radio_inst[pan_idx].sEnhAckPsdu[MAC_FRAME_TX_HDR_LEN + radio_inst[pan_idx].sVendorIeIndex];
            p_vendor_ie->oui[0] = 0x9b;
            p_vendor_ie->oui[1] = 0xb8;
            p_vendor_ie->oui[2] = 0xea;
            p_vendor_ie->subtype = 0;
            mac_memcpy(&p_vendor_ie->content, linkMetricsData, linkMetricsDataLen);
        }
    }
#endif
#endif
}

extern volatile uint64_t bt_clk_offset;

static uint8_t prev_rx_seq = 0;
static uint8_t prev_rx_src_addrmode = 0;
static uint64_t prev_rx_src_addr = 0;

bool readFrame(rx_item_t *item, uint8_t pan_idx)
{
    otRadioFrame *receivedFrame = &item->sReceivedFrames;
    pmac_rxfifo_t prx_fifo = (pmac_rxfifo_t)&item->sReceivedPsdu[0];
    mac_rxfifo_tail_t *prx_fifo_tail;
    uint8_t channel;
    int8_t rssi;
    uint8_t lqi;
    uint8_t *p_data;

#if 0
    if (mac_GetRxFrmSecEn() && mac_GetRxFrmAckReq() && 0x2 == mac_GetRxFrmSecKeyIdMode())
    {
        return false;
    }
#endif

    //mac_Rx((uint8_t *)prx_fifo);
    prx_fifo_tail = (mac_rxfifo_tail_t *)&item->sReceivedPsdu[1 + prx_fifo->frm_len];
    channel = mac_GetChannel();
    rssi = mac_GetRSSIFromRaw(prx_fifo_tail->rssi, channel);
    lqi = prx_fifo_tail->lqi;

    if (radio_inst[pan_idx].enhack_frm_len > 0 && mac_GetTxEnhAckPending())
    {
        BEE_tx_ack_started(&radio_inst[pan_idx].sEnhAckPsdu[1], rssi, lqi, pan_idx);
        if (radio_inst[pan_idx].enhack_frm_sec_en)
        {
            mac_LoadTxEnhAckPayload(radio_inst[pan_idx].enhack_frm_len, radio_inst[pan_idx].enhack_frm_len, &radio_inst[pan_idx].sEnhAckPsdu[2]);
            txAckProcessSecurity(&radio_inst[pan_idx].sEnhAckPsdu[1], pan_idx);
        }
        else
        {
            mac_LoadTxEnhAckPayload(0, radio_inst[pan_idx].enhack_frm_len, &radio_inst[pan_idx].sEnhAckPsdu[2]);
        }
        if (MAC_STS_SUCCESS == mac_TrigTxEnhAck(false, radio_inst[pan_idx].enhack_frm_sec_en))
        {
            radio_inst[pan_idx].enhack_frm_len = 0;
            if (radio_inst[pan_idx].enhack_frm_sec_en)
            {
                radio_inst[pan_idx].enhack_frm_sec_en = false;
                radio_inst[pan_idx].sMacFrameCounter++;
            }
        }
        else
        {
            otLogInfoPlat("enhack tx timeout");
        }
    }

    p_data = &item->sReceivedPsdu[0];
    receivedFrame->mPsdu               = &p_data[1];
    receivedFrame->mLength             = p_data[0];
    receivedFrame->mChannel            = channel;
    receivedFrame->mInfo.mRxInfo.mRssi = rssi;
    receivedFrame->mInfo.mRxInfo.mLqi  = lqi;

    // Inform if this frame was acknowledged with frame pending set.
    receivedFrame->mInfo.mRxInfo.mAckedWithFramePending = radio_inst[pan_idx].sAckedWithFramePending;

    // 0x7E header cmdid propid STREAM_RAW crc16 0x7E
    // Get the timestamp when the SFD was received
    receivedFrame->mInfo.mRxInfo.mTimestamp = otPlatTimeGet() -
                                              (receivedFrame->mLength * 32) -
                                              PHY_HDR_SYMBOL_TIME_US;

#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
    // Inform if this frame was acknowledged with secured Enh-ACK.
    if (p_data[ACK_REQUEST_OFFSET] & ACK_REQUEST_BIT && otMacFrameIsVersion2015(receivedFrame))
    {
        receivedFrame->mInfo.mRxInfo.mAckedWithSecEnhAck = radio_inst[pan_idx].sAckedWithSecEnhAck;
        receivedFrame->mInfo.mRxInfo.mAckFrameCounter    = radio_inst[pan_idx].sAckFrameCounter;
        receivedFrame->mInfo.mRxInfo.mAckKeyId           = radio_inst[pan_idx].sAckKeyId;
    }

    radio_inst[pan_idx].sAckedWithSecEnhAck = false;
#endif

    return true;
}

int8_t otPlatRadioGetReceiveSensitivity(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return SBEE2_RECEIVE_SENSITIVITY;
}

#if OPENTHREAD_CONFIG_MAC_HEADER_IE_SUPPORT
void BEE_tx_started(uint8_t *p_data, uint8_t pan_idx)
{
    bool processSecurity = false;

    OT_UNUSED_VARIABLE(p_data);

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
    if (radio_inst[pan_idx].sCslPeriod > 0)
    {
        otMacFrameSetCslIe(&radio_inst[pan_idx].sTransmitFrame, (uint16_t)radio_inst[pan_idx].sCslPeriod, getCslPhase(pan_idx));
    }
#endif

    // Update IE and secure transmit frame
#if OPENTHREAD_CONFIG_TIME_SYNC_ENABLE
    if (sTransmitFrame.mInfo.mTxInfo.mIeInfo->mTimeIeOffset != 0)
    {
        uint8_t *timeIe = sTransmitFrame.mPsdu + sTransmitFrame.mInfo.mTxInfo.mIeInfo->mTimeIeOffset;
        uint64_t time   = otPlatTimeGet() + sTransmitFrame.mInfo.mTxInfo.mIeInfo->mNetworkTimeOffset;

        *timeIe = sTransmitFrame.mInfo.mTxInfo.mIeInfo->mTimeSyncSeq;

        *(++timeIe) = (uint8_t)(time & 0xff);
        for (uint8_t i = 1; i < sizeof(uint64_t); i++)
        {
            time        = time >> 8;
            *(++timeIe) = (uint8_t)(time & 0xff);
        }

        processSecurity = true;
    }
#endif // OPENTHREAD_CONFIG_TIME_SYNC_ENABLE

exit:
    return;
}
#endif

void txProcessSecurity(uint8_t *aFrame, uint8_t pan_idx)
{
    OT_UNUSED_VARIABLE(aFrame);
#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
    radio_inst[pan_idx].sTransmitFrame.mInfo.mTxInfo.mAesKey = &radio_inst[pan_idx].sCurrKey;

    struct
    {
        uint8_t sec_level;
        uint32_t frame_counter;
        uint64_t src_ext_addr;
    } __attribute__((packed)) nonce;
    nonce.sec_level = otMacFrameGetSecurityLevel(&radio_inst[pan_idx].sTransmitFrame);
    nonce.frame_counter = otMacFrameGetFrameCounter(&radio_inst[pan_idx].sTransmitFrame);
    mac_memcpy((uint8_t *)&nonce.src_ext_addr, mpan_GetLongAddress(pan_idx), sizeof(nonce.src_ext_addr));

    // set nonce
    mac_LoadNonce((uint8_t *)&nonce);
    // set key
    mac_LoadTxNKey(radio_inst[pan_idx].sCurrKey.mKeyMaterial.mKey.m8);
    // set security level
    mac_SetTxNChiper(nonce.sec_level);
#endif
}

uint64_t otPlatRadioGetNow(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return otPlatTimeGet();
}

#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
void otPlatRadioSetMacKey(otInstance             *aInstance,
                          uint8_t                 aKeyIdMode,
                          uint8_t                 aKeyId,
                          const otMacKeyMaterial *aPrevKey,
                          const otMacKeyMaterial *aCurrKey,
                          const otMacKeyMaterial *aNextKey,
                          otRadioKeyType          aKeyType)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aKeyIdMode);

    assert(aKeyType == OT_KEY_TYPE_LITERAL_KEY);
    assert(aPrevKey != NULL && aCurrKey != NULL && aNextKey != NULL);

    uint32_t s = os_lock();

    radio_inst[pan_idx].sKeyId   = aKeyId;
    radio_inst[pan_idx].sPrevKey = *aPrevKey;
    radio_inst[pan_idx].sCurrKey = *aCurrKey;
    radio_inst[pan_idx].sNextKey = *aNextKey;

    os_unlock(s);
}

void otPlatRadioSetMacFrameCounter(otInstance *aInstance, uint32_t aMacFrameCounter)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);

    uint32_t s = os_lock();

    radio_inst[pan_idx].sMacFrameCounter = aMacFrameCounter;

    os_unlock(s);
}

void otPlatRadioSetMacFrameCounterIfLarget(otInstance *aInstance, uint32_t aMacFrameCounter)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);

    uint32_t s = os_lock();

    if (aMacFrameCounter > radio_inst[pan_idx].sMacFrameCounter)
    {
        radio_inst[pan_idx].sMacFrameCounter = aMacFrameCounter;
    }

    os_unlock(s);
}
#endif // OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2

#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
static void updateIeData(otInstance *aInstance, otShortAddress aShortAddr,
                         const otExtAddress *aExtAddr, uint8_t pan_idx)
{
    OT_UNUSED_VARIABLE(aInstance);

    otMacAddress macAddress;
    macAddress.mType                  = OT_MAC_ADDRESS_TYPE_SHORT;
    macAddress.mAddress.mShortAddress = aShortAddr;

    radio_inst[pan_idx].enhAckProbingDataLen = otLinkMetricsEnhAckGetDataLen(&macAddress);
}
#endif // OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
otError otPlatRadioEnableCsl(otInstance         *aInstance,
                             uint32_t            aCslPeriod,
                             otShortAddress      aShortAddr,
                             const otExtAddress *aExtAddr)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    radio_inst[pan_idx].sCslPeriod = aCslPeriod;
    return OT_ERROR_NONE;
}

void otPlatRadioUpdateCslSampleTime(otInstance *aInstance, uint32_t aCslSampleTime)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);

    radio_inst[pan_idx].sCslSampleTime = aCslSampleTime;
}

uint8_t otPlatRadioGetCslAccuracy(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return otPlatTimeGetXtalAccuracy() / 2;
}
#endif // OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE

#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
otError otPlatRadioConfigureEnhAckProbing(otInstance          *aInstance,
                                          otLinkMetrics        aLinkMetrics,
                                          const otShortAddress aShortAddress,
                                          const otExtAddress *aExtAddress)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aLinkMetrics);
    OT_UNUSED_VARIABLE(aShortAddress);
    OT_UNUSED_VARIABLE(aExtAddress);

    otError error = OT_ERROR_NONE;

    SuccessOrExit(error = otLinkMetricsConfigureEnhAckProbing(aShortAddress, aExtAddress,
                                                              aLinkMetrics));
    updateIeData(aInstance, aShortAddress, aExtAddress, pan_idx);

exit:
    return error;
}
#endif

otError otPlatRadioSetChannelMaxTransmitPower(otInstance *aInstance, uint8_t aChannel,
                                              int8_t aMaxPower)
{
    OT_UNUSED_VARIABLE(aInstance);
    otError error = OT_ERROR_NONE;

    return error;
}

otError otPlatRadioSetRegion(otInstance *aInstance, uint16_t aRegionCode)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);

    radio_inst[pan_idx].sRegionCode = aRegionCode;
    return OT_ERROR_NONE;
}

otError otPlatRadioGetRegion(otInstance *aInstance, uint16_t *aRegionCode)
{
    uint8_t pan_idx = mpan_GetCurrentPANIdx();
    OT_UNUSED_VARIABLE(aInstance);
    otError error = OT_ERROR_NONE;

    VerifyOrExit(aRegionCode != NULL, error = OT_ERROR_INVALID_ARGS);

    *aRegionCode = radio_inst[pan_idx].sRegionCode;
exit:
    return error;
}

static uint8_t generate_ieee_enhack_frame(uint8_t *txbuf, uint8_t buf_len, fc_t *fc, uint8_t seq,
                                          uint16_t dpid, uint8_t *dadr, uint8_t aux_len, uint8_t *aux, uint8_t pan_idx)
{
    uint8_t len = 0;
    csl_ie_t *p_csl_ie;
    vendor_ie_t *p_vendor_ie;

    //frame control
    mac_memcpy((void *)(txbuf + len), (void *)fc, 2);
    len += 2;

    if (!fc->seq_num_suppress)
    {
        //sequence number
        txbuf[len] = seq;
        len += 1;
    }

    {
        if (!fc->panid_compress)
        {
            mac_memcpy(&txbuf[len], &dpid, 2);
            len += 2;
        }
        if (fc->dst_addr_mode > 0)
        {
            if (fc->dst_addr_mode == ADDR_MODE_SHORT)
            {
                mac_memcpy(&txbuf[len], dadr, 2);
                len += 2;
            }
            else
            {
                mac_memcpy(&txbuf[len], dadr, 8);
                len += 8;
            }
        }
    }

    if (fc->sec_en) //security check
    {
        mac_memcpy((void *)(txbuf + len), (void *)aux, aux_len);
        len += aux_len;
    }

    if (fc->ie_present) // ie
    {
#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
        if (radio_inst[pan_idx].sCslPeriod > 0)
        {
            radio_inst[pan_idx].sCslIeIndex = len;
            p_csl_ie = (csl_ie_t *)&txbuf[radio_inst[pan_idx].sCslIeIndex];
            p_csl_ie->len = 4;
            p_csl_ie->id = 0x1a;
            p_csl_ie->type = 0;
            len += sizeof(csl_ie_t);
        }
        else
        {
            radio_inst[pan_idx].sCslIeIndex = 0;
        }
#endif
#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
        if (radio_inst[pan_idx].enhAckProbingDataLen > 0)
        {
            radio_inst[pan_idx].sVendorIeIndex = len;
            p_vendor_ie = (vendor_ie_t *)&txbuf[radio_inst[pan_idx].sVendorIeIndex];
            p_vendor_ie->len = 6;
            p_vendor_ie->id = 0;
            p_vendor_ie->type = 0;
            len += sizeof(vendor_ie_t);
        }
        else
        {
            radio_inst[pan_idx].sVendorIeIndex = 0;
        }
#endif
    }
    else
    {
#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
        radio_inst[pan_idx].sCslIeIndex = 0;
#endif
#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
        radio_inst[pan_idx].sVendorIeIndex = 0;
#endif
    }

    return len;
}

#define TXNS_OFFSET     0
#define CCAFAIL_OFFSET  5

    void txn_handler(uint8_t pan_idx, uint32_t arg)
    {
        uint8_t tx_status = mac_GetTxNStatus();
        if (tx_status & (1<<TXNS_OFFSET))
        {
            if ((tx_status & (1<<CCAFAIL_OFFSET)) ||
                (mac_GetTxNTermedStatus()))
            {
                BEE_EventSend(TX_CCA_FAIL, pan_idx);
                if (pan_idx == 0) otSysEventSignalPending();
                else zbSysEventSignalPending();
            }
            else
            {
                BEE_EventSend(TX_NO_ACK, pan_idx);
                if (pan_idx == 0) otSysEventSignalPending();
                else zbSysEventSignalPending();
            }
        }
        else
        {
            if (otMacFrameIsAckRequested(&radio_inst[pan_idx].sTransmitFrame))
            {
                if (otMacFrameIsVersion2015(&radio_inst[pan_idx].sTransmitFrame))
                {
                }
                else
                {
                    radio_inst[pan_idx].rx_queue[radio_inst[pan_idx].rx_tail].sReceivedFrames.mPsdu = &radio_inst[pan_idx].rx_queue[radio_inst[pan_idx].rx_tail].sReceivedPsdu[1];
                    otMacFrameGenerateImmAck(&radio_inst[pan_idx].sTransmitFrame, mac_GetImmAckPendingBit(), &radio_inst[pan_idx].rx_queue[radio_inst[pan_idx].rx_tail].sReceivedFrames);
                    radio_inst[pan_idx].rx_tail = (radio_inst[pan_idx].rx_tail + 1) % RX_BUF_SIZE;
                    BEE_EventSend(RX_OK, pan_idx);
                    if (pan_idx == 0) otSysEventSignalPending();
                    else zbSysEventSignalPending();
                }
            }
            else
            {
                BEE_EventSend(TX_OK, pan_idx);
                if (pan_idx == 0) otSysEventSignalPending();
                else zbSysEventSignalPending();
            }
        }
    }

    void rxely_handler(uint8_t pan_idx, uint32_t arg)
    {
        fc_t fc_ack;
        struct
        {
            aux_sec_ctl_t sec_ctl;
            uint32_t frame_counter;
            uint8_t key_id;
        } __attribute__((packed)) aux;

        do
        {
#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
            if (FRAME_VER_2015 == mac_GetRxFrmVersion())
            {
                if (!mac_GetRxFrmAckReq())
                {
                    radio_inst[pan_idx].sAckedWithFramePending = false;
                    break;
                }

                uint16_t panid;
                uint16_t saddr;
                uint64_t laddr;

                radio_inst[pan_idx].enhack_frm_sec_en = mac_GetRxFrmSecEn();
                fc_ack.type = FRAME_TYPE_ACK;
                fc_ack.sec_en = radio_inst[pan_idx].enhack_frm_sec_en;
                if (mac_GetSrcMatchStatus())
                {
                    mac_ClrSrcMatchStatus();
                    radio_inst[pan_idx].sAckedWithFramePending = true;
                }
                else
                {
                    radio_inst[pan_idx].sAckedWithFramePending = false;
                }
                fc_ack.pending = radio_inst[pan_idx].sAckedWithFramePending;
                fc_ack.ack_req = 0;
                fc_ack.panid_compress = mac_GetRxFrmPanidCompress();
                fc_ack.rsv = 0;
                fc_ack.seq_num_suppress = 0;
                fc_ack.ie_present = 0;
#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
                if (radio_inst[pan_idx].sCslPeriod > 0) { fc_ack.ie_present = 1; }
#endif
#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
                if (radio_inst[pan_idx].enhAckProbingDataLen > 0) { fc_ack.ie_present = 1; }
#endif
                fc_ack.dst_addr_mode = mac_GetRxFrmSrcAddrMode();
                fc_ack.ver = FRAME_VER_2015;
                fc_ack.src_addr_mode = 0;

                if (radio_inst[pan_idx].enhack_frm_sec_en)
                {
                    aux.sec_ctl.sec_level = mac_GetRxFrmSecLevel();
                    aux.sec_ctl.key_id_mode = mac_GetRxFrmSecKeyIdMode();
                    aux.sec_ctl.frame_counter_supp = 0;
                    aux.sec_ctl.asn_in_nonce = 0;
                    aux.sec_ctl.rsv = 0;
                    // set frame counter
                    aux.frame_counter = radio_inst[pan_idx].sMacFrameCounter;
                    aux.key_id = mac_GetRxFrmSecKeyId();
                }

                panid = mpan_GetPANId(pan_idx);

                if (fc_ack.dst_addr_mode == ADDR_MODE_SHORT)
                {
                    saddr = mac_GetRxFrmShortAddr();
                    radio_inst[pan_idx].enhack_frm_len = generate_ieee_enhack_frame(&radio_inst[pan_idx].sEnhAckPsdu[MAC_FRAME_TX_HDR_LEN],
                                                                IEEE802154_MAX_LENGTH, &fc_ack, mac_GetRxFrmSeq(), panid, (uint8_t *)&saddr, sizeof(aux),
                                                                (uint8_t *)&aux, pan_idx);
                }
                else
                {
                    laddr = mac_GetRxFrmLongAddr();
                    radio_inst[pan_idx].enhack_frm_len = generate_ieee_enhack_frame(&radio_inst[pan_idx].sEnhAckPsdu[MAC_FRAME_TX_HDR_LEN],
                                                                IEEE802154_MAX_LENGTH, &fc_ack, mac_GetRxFrmSeq(), panid, (uint8_t *)&laddr, sizeof(aux),
                                                                (uint8_t *)&aux, pan_idx);
                }

                radio_inst[pan_idx].sEnhAckPsdu[0] = 0;
                radio_inst[pan_idx].sEnhAckPsdu[1] = radio_inst[pan_idx].enhack_frm_len + 2;
#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
                if (radio_inst[pan_idx].enhAckProbingDataLen > 0)
                {
                    mac_SetTxEnhAckPending(radio_inst[pan_idx].enhack_frm_len);
                }
                else
#endif
                {
                    BEE_tx_ack_started(&radio_inst[pan_idx].sEnhAckPsdu[1], 0, 0, pan_idx);
                    if (radio_inst[pan_idx].enhack_frm_sec_en)
                    {
                        mac_LoadTxEnhAckPayload(radio_inst[pan_idx].enhack_frm_len, radio_inst[pan_idx].enhack_frm_len, &radio_inst[pan_idx].sEnhAckPsdu[2]);
                        txAckProcessSecurity(&radio_inst[pan_idx].sEnhAckPsdu[1], pan_idx);
                    }
                    else
                    {
                        mac_LoadTxEnhAckPayload(0, radio_inst[pan_idx].enhack_frm_len, &radio_inst[pan_idx].sEnhAckPsdu[2]);
                    }
                    mac_TrigTxEnhAck(true, radio_inst[pan_idx].enhack_frm_sec_en);
                    radio_inst[pan_idx].enhack_frm_len = 0;
                    if (radio_inst[pan_idx].enhack_frm_sec_en)
                    {
                        radio_inst[pan_idx].enhack_frm_sec_en = false;
                        radio_inst[pan_idx].sMacFrameCounter++;
                    }
                }
            }
            else
#endif
            {
                if (!mac_GetRxFrmAckReq())
                {
                    radio_inst[pan_idx].sAckedWithFramePending = false;
                    break;
                }
                if (mac_GetSrcMatchStatus())
                {
                    mac_ClrSrcMatchStatus();
                    radio_inst[pan_idx].sAckedWithFramePending = true;
                }
                else
                {
                    radio_inst[pan_idx].sAckedWithFramePending = false;
                }
            }
        }
        while (0);
    }

    void rxdone_handler(uint8_t pan_idx, uint32_t arg)
    {
        mac_Rx(radio_inst[pan_idx].rx_queue[radio_inst[pan_idx].rx_tail].sReceivedPsdu);
        readFrame(&radio_inst[pan_idx].rx_queue[radio_inst[pan_idx].rx_tail], pan_idx);
        radio_inst[pan_idx].rx_tail = (radio_inst[pan_idx].rx_tail + 1) % RX_BUF_SIZE;
        BEE_EventSend(RX_OK, pan_idx);
        if (pan_idx == 0) otSysEventSignalPending();
        else zbSysEventSignalPending();
    }

    void btcmp0_handler(uint8_t pan_idx, uint32_t arg)
    {
        milli_handler(0);
    }

    void btcmp1_handler(uint8_t pan_idx, uint32_t arg)
    {
        micro_handler(0);
    }

    void edscan_handler(uint8_t pan_idx, uint32_t arg)
    {
        os_sem_give(radio_inst[pan_idx].edscan_done);
    }

#ifdef RT_PLATFORM_BB2ULTRA
    void btcmp4_handler(uint8_t pan_idx, uint32_t arg)
    {
        milli_handler(1);
    }

    void btcmp5_handler(uint8_t pan_idx, uint32_t arg)
    {
        micro_handler(1);
    }
#endif
