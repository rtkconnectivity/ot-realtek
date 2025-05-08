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

/*============================================================================*
 *               Define to prevent recursive inclusion
 *============================================================================*/
#ifndef __MEM_CONFIG__
#define __MEM_CONFIG__

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup MEM_CONFIG Memory Configure
  * @brief Memory configuration for user application
  * @{
  */
// Build Bank Configure
//#define BUILD_BANK                  0   //Bank0 as default

//Enable RAM Code
#define FEATURE_RAM_CODE            0

/** @brief encrypt app or not */
#define FEATURE_ENCRYPTION          0

//default disable trustzone
#define FEATURE_TRUSTZONE_ENABLE    0

/*============================================================================*
  *                            Memory layout
  *============================================================================*/
#define SPIC0_ADDR            0x04000000
#define SPIC0_SIZE            (64*1024*1024)
#define SPIC1_ADDR            0x08000000
#define SPIC1_SIZE            (64*1024*1024)
#define SPIC2_ADDR            0x10000000
#define SPIC2_SIZE            (256*1024*1024)

#define ITCM1_ADDR            0x00100000
#define ITCM1_SIZE            (192*1024)

#define DTCM0_ADDR            0x00130000
#define DTCM0_SIZE            (64*1024)

#define DTCM1_ADDR            0x00140000
#define DTCM1_SIZE            (64*1024)

#define DATA_SRAM_ADDR        0x00200000
#define DATA_SRAM_SIZE        (16*1024)

#define BUFFER_RAM_ADDR       0x00280000
#define BUFFER_RAM_SIZE       (48*1024)

#define VECTOR_TABLE_ITEMS    (187)

/*============================================================================*
  *                    Non Secure Buffer RAM layout
  *============================================================================*/
#define BUFFER_ON_GLOBAL_SIZE   (1024 + 512)  //must sync with lowerstack

/* --------------------The following macros should not be modified!------------------------- */
#define BUFFER_ON_HEAP_ADDR             (BUFFER_RAM_ADDR + BUFFER_ON_GLOBAL_SIZE)
#define BUFFER_ON_HEAP_SIZE             (BUFFER_RAM_SIZE - BUFFER_ON_GLOBAL_SIZE)

/*============================================================================*
  *                       Non Secure Ext Data SRAM layout
  *============================================================================*/
#define EXT_DATA_SRAM_GLOBAL_SIZE       (0*1024)

/* --------------------The following macros should not be modified!------------------------- */
#define EXT_DATA_SRAM_GLOBAL_ADDR       DATA_SRAM_ADDR
#define EXT_DATA_SRAM_HEAP_ADDR         (EXT_DATA_SRAM_GLOBAL_ADDR + EXT_DATA_SRAM_GLOBAL_SIZE)
#define EXT_DATA_SRAM_HEAP_SIZE         (DATA_SRAM_SIZE - EXT_DATA_SRAM_GLOBAL_SIZE)


/*=======================================================================================*
  *                                 ITCM1 + DTCM0 + DTCM1 layout
  *=====================================================================================*/
/* RAM(ITCM1):          Bee4 size: 192K
 * RAM(DTCM0):          Bee4 size:  64K
 * RAM(DTCM1):          Bee4 size:  64K
 * Total RAM(TCM):      Bee4 size: 320K
example:
   a) non-secure total size:                          240K
      1) non-secure upperstack ram         3K
      2) non-secure APP ram                147K
      3) non-secure tcm heap               90K
   b) secure app total size:                          0K
      1) secure app ram                    0K
      2) secure tcm heap                   0K
   c) non-secure system reserved size:                60K
      1) non-secure main stack             3K
      2) non-secure rom global             7K - 64B
      3) non-secure lowstack rom global    11K + 64B
      4) non-secure stack patch ram        25K
      5) non-secure patch global and code  14K
   d) secure system reserved size:                    20K
      1) secure boot patch ram             12K
      2) secure main stack                 3K
      3) secure rom global                 5K
*/
#define S_RAM_APP_RESERVED_SIZE         (0*1024)
#define S_RAM_APP_SIZE                  (0*1024)


#define NS_RAM_UPPERSTACK_SIZE          (3*1024)
#define NS_RAM_APP_SIZE                 (151*1024)

/* --------------------The following macros should not be modified!------------------------- */
#define TCM_START_ADDR                  ITCM1_ADDR
#define TCM_TOTAL_SIZE                  (ITCM1_SIZE + DTCM0_SIZE + DTCM1_SIZE)

#define S_RAM_SYSTEM_RESERVED_SIZE      (20*1024)
#define NS_RAM_SYSTEM_RESERVED_SIZE     (60*1024)
#define NS_RAM_APP_RESERVED_SIZE        (TCM_TOTAL_SIZE - S_RAM_SYSTEM_RESERVED_SIZE - NS_RAM_SYSTEM_RESERVED_SIZE - S_RAM_APP_RESERVED_SIZE)

#if (S_RAM_SYSTEM_RESERVED_SIZE % 4096 != 0) || (NS_RAM_SYSTEM_RESERVED_SIZE % 4096 != 0) || (S_RAM_APP_RESERVED_SIZE % 4096 != 0)
#error "memory Config IDAU region size is unaligned with 4KB"
#endif

#if (S_RAM_SYSTEM_RESERVED_SIZE + NS_RAM_SYSTEM_RESERVED_SIZE + S_RAM_APP_RESERVED_SIZE > TCM_TOTAL_SIZE)
#error "TCM Config size error"
#endif

/* --------------------The following macros should not be modified!------------------------- */
#define NS_RAM_UPPERSTACK_ADDR          (TCM_START_ADDR)
#define NS_RAM_APP_ADDR                 (NS_RAM_UPPERSTACK_ADDR + NS_RAM_UPPERSTACK_SIZE)
#define NS_HEAP_ADDR                    (NS_RAM_APP_ADDR + NS_RAM_APP_SIZE)
#define NS_HEAP_SIZE                    (NS_RAM_APP_RESERVED_SIZE - NS_RAM_APP_SIZE - NS_RAM_UPPERSTACK_SIZE)

#define S_RAM_APP_ADDR                  (TCM_START_ADDR + NS_RAM_APP_RESERVED_SIZE)
#define S_HEAP_ADDR                     (S_RAM_APP_ADDR + S_RAM_APP_SIZE)
#define S_HEAP_SIZE                     (S_RAM_APP_RESERVED_SIZE - S_RAM_APP_SIZE)


#if (S_RAM_APP_SIZE > S_RAM_APP_RESERVED_SIZE)
#error "secure app ram size  config error"
#endif

#if (NS_RAM_UPPERSTACK_SIZE + NS_RAM_APP_SIZE > NS_RAM_APP_RESERVED_SIZE)
#error "non-secure app ram size config error"
#endif

/* --------------------The following macros should not be modified after rom tapout ------------------------- */
#define S_RAM_ROM_GLOBAL_SIZE           (5*1024)
#define S_RAM_MAIN_STACK_SIZE           (3*1024)
#define S_RAM_BOOT_PATCH_SIZE           (12*1024)

#define NS_RAM_PATCH_SIZE               (14*1024)
#define NS_RAM_STACK_PATCH_SIZE         (25*1024)
#define NS_RAM_LOWSTACK_GLOBAL_SIZE     (11*1024 + 64)
#define NS_RAM_ROM_GLOBAL_SIZE          (7*1024 - 64)
#define NS_RAM_MAIN_STACK_SIZE          (3*1024)

#if (S_RAM_ROM_GLOBAL_SIZE + S_RAM_MAIN_STACK_SIZE + S_RAM_BOOT_PATCH_SIZE != S_RAM_SYSTEM_RESERVED_SIZE)
#error "secure system size error!"
#endif

#if (NS_RAM_PATCH_SIZE + NS_RAM_STACK_PATCH_SIZE + NS_RAM_LOWSTACK_GLOBAL_SIZE + NS_RAM_ROM_GLOBAL_SIZE + NS_RAM_MAIN_STACK_SIZE != NS_RAM_SYSTEM_RESERVED_SIZE)
#error "non-secure system size error!"
#endif

/* -------------------------The following macros should not be modified!------------------------------------ */
#define S_RAM_ROM_GLOBAL_ADDR           (TCM_START_ADDR + TCM_TOTAL_SIZE - S_RAM_ROM_GLOBAL_SIZE)
#define S_RAM_MAIN_STACK_START_ADDR     (S_RAM_ROM_GLOBAL_ADDR - S_RAM_MAIN_STACK_SIZE)
#define S_RAM_BOOT_PATCH_ADDR           (S_RAM_MAIN_STACK_START_ADDR - S_RAM_BOOT_PATCH_SIZE)
#define NS_RAM_PATCH_ADDR               (S_RAM_BOOT_PATCH_ADDR - NS_RAM_PATCH_SIZE)
#define NS_RAM_STACK_PATCH_ADDR         (NS_RAM_PATCH_ADDR - NS_RAM_STACK_PATCH_SIZE)
#define NS_RAM_LOWSTACK_GLOBAL_ADDR     (NS_RAM_STACK_PATCH_ADDR - NS_RAM_LOWSTACK_GLOBAL_SIZE)
#define NS_RAM_ROM_GLOBAL_ADDR          (NS_RAM_LOWSTACK_GLOBAL_ADDR - NS_RAM_ROM_GLOBAL_SIZE)
#define NS_RAM_MAIN_STACK_START_ADDR    (NS_RAM_ROM_GLOBAL_ADDR - NS_RAM_MAIN_STACK_SIZE)


#define S_RAM_VECTOR_ADDR               (S_RAM_ROM_GLOBAL_ADDR)
#define S_RAM_VECTOR_SIZE               (VECTOR_TABLE_ITEMS*4)

#define NS_RAM_VECTOR_ADDR              (NS_RAM_ROM_GLOBAL_ADDR)
#define NS_RAM_VECTOR_SIZE              (VECTOR_TABLE_ITEMS*4)
/* ------------------------------------------------------------------------------------------------------- */

/** @} */ /* End of group MEM_CONFIG_Exported_Constents */



#ifdef __cplusplus
}
#endif


/** @} */ /* End of group MEM_CONFIG */

#endif

