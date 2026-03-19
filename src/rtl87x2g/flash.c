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

#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <common/logging.hpp>
#include "platform-bee.h"
#include "iot_persistent_config.h"
//#include "rom-utility.h"

#define FLASH_CTRL_FCTL_BUSY 0x00000080

#define FLASH_PAGE_SIZE 4096
#define FLASH_PAGE_NUM (THREAD_PER_SIZE / FLASH_PAGE_SIZE)
#define FLASH_SWAP_SIZE (FLASH_PAGE_SIZE * (FLASH_PAGE_NUM / 2))

uint32_t __attribute__((weak)) thread_persistent_addr_get(void)
{
    return THREAD_PER_ADDR;
}

/* Convert a settings offset to the physical address within the flash settings pages */
static uint32_t flashPhysAddr(uint8_t aSwapIndex, uint32_t aOffset)
{
    uint32_t address = thread_persistent_addr_get() + aOffset;

    if (aSwapIndex)
    {
        address += FLASH_SWAP_SIZE;
    }

    return address;
}

void otPlatFlashInit(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
}

uint32_t otPlatFlashGetSwapSize(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return FLASH_SWAP_SIZE;
}

void otPlatFlashErase(otInstance *aInstance, uint8_t aSwapIndex)
{
    uint32_t addr;
    uint32_t i;
    OT_UNUSED_VARIABLE(aInstance);
    addr = flashPhysAddr(aSwapIndex, 0);
    //__disable_irq();
    for (i = 0; i < (FLASH_SWAP_SIZE / FLASH_PAGE_SIZE); i++)
    {
        flash_nor_erase_locked((addr + FLASH_PAGE_SIZE * i), FLASH_NOR_ERASE_SECTOR);
    }
    //__enable_irq();
}

void otPlatFlashWrite(otInstance *aInstance, uint8_t aSwapIndex, uint32_t aOffset,
                      const void *aData, uint32_t aSize)
{
    OT_UNUSED_VARIABLE(aInstance);
    //__disable_irq();
    uint8_t *buf = malloc(aSize);
    memcpy(buf, aData, aSize);
    flash_nor_write_locked(flashPhysAddr(aSwapIndex, aOffset), buf, aSize);
    free(buf);
    //__enable_irq();
}

void otPlatFlashRead(otInstance *aInstance, uint8_t aSwapIndex, uint32_t aOffset, uint8_t *aData,
                     uint32_t aSize)
{
    OT_UNUSED_VARIABLE(aInstance);
    //__disable_irq();
    flash_nor_read_locked(flashPhysAddr(aSwapIndex, aOffset), aData, aSize);
    //__enable_irq();
}

