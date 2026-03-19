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

#ifndef _FLASH_MAP_H_
#define _FLASH_MAP_H_

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
*                        Flash Layout
*============================================================================*/
/*  Flash total size                                4096KB
example:
    1) Reserved:                                          4K (0x04000000)
    2) Factory Data:                                      4K (0x04000000)
    3) OEM Header:                                        4K (0x04001000)
    4) Bank0 Boot Patch Code:                            32K (0x04002000)
    5) Bank1 Boot Patch Code:                            32K (0x0400A000)
    6) OTA Bank0:                                      1332K (0x04012000)
        a) OTA Header                    4K (0x04012000)
        b) System Patch code            32K (0x04013000)
        c) BT Stack Patch code          60K (0x0401B000)
        d) BT Host code                212K (0x0402A000)
        e) APP code                   1024K (0x0405F000)
        f) APP Config File               0K (0x0415F000)
        g) APP data1                     0K (0x0415F000)
        h) APP data2                     0K (0x0415F000)
        i) APP data3                     0K (0x0415F000)
        j) APP data4                     0K (0x0415F000)
        k) APP data5                     0K (0x0415F000)
        l) APP data6                     0K (0x0415F000)
    7) OTA Bank1:                                         0K (0x0415F000)
        a) OTA Header                    0K (0x00000000)
        b) System Patch code             0K (0x00000000)
        c) BT Stack Patch code           0K (0x00000000)
        d) BT Host code                  0K (0x00000000)
        e) APP code                      0K (0x00000000)
        f) APP Config File               0K (0x00000000)
        g) APP data1                     0K (0x00000000)
        h) APP data2                     0K (0x00000000)
        i) APP data3                     0K (0x00000000)
        j) APP data4                     0K (0x00000000)
        k) APP data5                     0K (0x00000000)
        l) APP data6                     0K (0x00000000)
    8) Bank0 Secure APP Code:                            16K (0x0415F000)
    9) Bank0 Secure APP Data:                             0K (0x04163000)
    10) Bank1 Secure APP Code:                            0K (0x04163000)
    11) Bank1 Secure APP Data:                            0K (0x04163000)
    12) RO DATA1:                                         0K (0x04163000)
    13) RO DATA2:                                         0K (0x04163000)
    14) RO DATA3:                                         0K (0x04163000)
    15) RO DATA4:                                         0K (0x04163000)
    16) RO DATA5:                                         0K (0x04163000)
    17) RO DATA6:                                         0K (0x04163000)
    18) OTA Tmp:                                       1024K (0x04163000)
    19) FTL:                                             16K (0x04263000)
    20) user data1:                                       0K (0x04267000)
    21) user data2:                                       0K (0x04267000)
    22) user data3:                                       0K (0x04267000)
    23) user data4:                                       0K (0x04267000)
    24) user data5:                                       0K (0x04267000)
    25) user data6:                                       0K (0x04267000)
    26) user data7:                                       0K (0x04267000)
    27) user data8:                                       0K (0x04267000)
    29) APP Defined Section1:                            40K (0x04267000)
    2:) APP Defined Section2:                             0K (0x04271000)
*/

/*============================================================================*
*            Flash Layout Configuration (Generated by FlashMapGenerateTool)
*============================================================================*/

#define FLASH_START_ADDR                0x04000000  //Fixed
#define FLASH_MAX_SIZE                  0x00400000  //4096K Bytes

/* ========== High Level Flash Layout Configuration ========== */
#define FACTORY_DATA_ADDR               0x04000000
#define FACTORY_DATA_SIZE               0x00001000  //4K Bytes
#define OEM_CFG_ADDR                    0x04001000
#define OEM_CFG_SIZE                    0x00001000  //4K Bytes
#define BANK0_BOOT_PATCH_ADDR           0x04002000
#define BANK0_BOOT_PATCH_SIZE           0x00008000  //32K Bytes
#define BANK1_BOOT_PATCH_ADDR           0x0400A000
#define BANK1_BOOT_PATCH_SIZE           0x00008000  //32K Bytes
#define OTA_BANK0_ADDR                  0x04012000
#define OTA_BANK0_SIZE                  0x0014D000  //1332K Bytes
#define OTA_BANK1_ADDR                  0x0415F000
#define OTA_BANK1_SIZE                  0x00000000  //0K Bytes
#define BANK0_SECURE_APP_ADDR           0x0415F000
#define BANK0_SECURE_APP_SIZE           0x00004000  //16K Bytes
#define BANK0_SECURE_APP_DATA_ADDR      0x04163000
#define BANK0_SECURE_APP_DATA_SIZE      0x00000000  //0K Bytes
#define BANK1_SECURE_APP_ADDR           0x04163000
#define BANK1_SECURE_APP_SIZE           0x00000000  //0K Bytes
#define BANK1_SECURE_APP_DATA_ADDR      0x04163000
#define BANK1_SECURE_APP_DATA_SIZE      0x00000000  //0K Bytes
#define RO_DATA1_ADDR                   0x04163000
#define RO_DATA1_SIZE                   0x00000000  //0K Bytes
#define RO_DATA2_ADDR                   0x04163000
#define RO_DATA2_SIZE                   0x00000000  //0K Bytes
#define RO_DATA3_ADDR                   0x04163000
#define RO_DATA3_SIZE                   0x00000000  //0K Bytes
#define RO_DATA4_ADDR                   0x04163000
#define RO_DATA4_SIZE                   0x00000000  //0K Bytes
#define RO_DATA5_ADDR                   0x04163000
#define RO_DATA5_SIZE                   0x00000000  //0K Bytes
#define RO_DATA6_ADDR                   0x04163000
#define RO_DATA6_SIZE                   0x00000000  //0K Bytes
#define OTA_TMP_ADDR                    0x04163000
#define OTA_TMP_SIZE                    0x00100000  //1024K Bytes
#define FTL_ADDR                        0x04263000
#define FTL_SIZE                        0x00004000  //16K Bytes
#define USER_DATA1_ADDR                 0x04267000
#define USER_DATA1_SIZE                 0x00000000  //0K Bytes
#define USER_DATA2_ADDR                 0x04267000
#define USER_DATA2_SIZE                 0x00000000  //0K Bytes
#define USER_DATA3_ADDR                 0x04267000
#define USER_DATA3_SIZE                 0x00000000  //0K Bytes
#define USER_DATA4_ADDR                 0x04267000
#define USER_DATA4_SIZE                 0x00000000  //0K Bytes
#define USER_DATA5_ADDR                 0x04267000
#define USER_DATA5_SIZE                 0x00000000  //0K Bytes
#define USER_DATA6_ADDR                 0x04267000
#define USER_DATA6_SIZE                 0x00000000  //0K Bytes
#define USER_DATA7_ADDR                 0x04267000
#define USER_DATA7_SIZE                 0x00000000  //0K Bytes
#define USER_DATA8_ADDR                 0x04267000
#define USER_DATA8_SIZE                 0x00000000  //0K Bytes
#define BKP_DATA1_ADDR                  0x04267000
#define BKP_DATA1_SIZE                  0x0000A000  //40K Bytes
#define BKP_DATA2_ADDR                  0x04271000
#define BKP_DATA2_SIZE                  0x00000000  //0K Bytes

/* ========== OTA Bank0 Flash Layout Configuration ========== */
#define BANK0_OTA_HDR_ADDR              0x04012000
#define BANK0_OTA_HDR_SIZE              0x00001000  //4K Bytes
#define BANK0_SYSTEM_PATCH_ADDR         0x04013000
#define BANK0_SYSTEM_PATCH_SIZE         0x00008000  //32K Bytes
#define BANK0_BT_STACK_PATCH_ADDR       0x0401B000
#define BANK0_BT_STACK_PATCH_SIZE       0x0000F000  //60K Bytes
#define BANK0_BT_HOST_ADDR              0x0402A000
#define BANK0_BT_HOST_SIZE              0x00035000  //212K Bytes
#define BANK0_APP_ADDR                  0x0405F000
#define BANK0_APP_SIZE                  0x00100000  //1024K Bytes
#define BANK0_APP_CONFIG_FILE_ADDR      0x0415F000
#define BANK0_APP_CONFIG_FILE_SIZE      0x00000000  //0K Bytes
#define BANK0_APP_DATA1_ADDR            0x0415F000
#define BANK0_APP_DATA1_SIZE            0x00000000  //0K Bytes
#define BANK0_APP_DATA2_ADDR            0x0415F000
#define BANK0_APP_DATA2_SIZE            0x00000000  //0K Bytes
#define BANK0_APP_DATA3_ADDR            0x0415F000
#define BANK0_APP_DATA3_SIZE            0x00000000  //0K Bytes
#define BANK0_APP_DATA4_ADDR            0x0415F000
#define BANK0_APP_DATA4_SIZE            0x00000000  //0K Bytes
#define BANK0_APP_DATA5_ADDR            0x0415F000
#define BANK0_APP_DATA5_SIZE            0x00000000  //0K Bytes
#define BANK0_APP_DATA6_ADDR            0x0415F000
#define BANK0_APP_DATA6_SIZE            0x00000000  //0K Bytes

/* ========== OTA Bank1 Flash Layout Configuration ========== */
#define BANK1_OTA_HDR_ADDR              0x00000000
#define BANK1_OTA_HDR_SIZE              0x00000000  //0K Bytes
#define BANK1_SYSTEM_PATCH_ADDR         0x00000000
#define BANK1_SYSTEM_PATCH_SIZE         0x00000000  //0K Bytes
#define BANK1_BT_STACK_PATCH_ADDR       0x00000000
#define BANK1_BT_STACK_PATCH_SIZE       0x00000000  //0K Bytes
#define BANK1_BT_HOST_ADDR              0x00000000
#define BANK1_BT_HOST_SIZE              0x00000000  //0K Bytes
#define BANK1_APP_ADDR                  0x00000000
#define BANK1_APP_SIZE                  0x00000000  //0K Bytes
#define BANK1_APP_CONFIG_FILE_ADDR      0x00000000
#define BANK1_APP_CONFIG_FILE_SIZE      0x00000000  //0K Bytes
#define BANK1_APP_DATA1_ADDR            0x00000000
#define BANK1_APP_DATA1_SIZE            0x00000000  //0K Bytes
#define BANK1_APP_DATA2_ADDR            0x00000000
#define BANK1_APP_DATA2_SIZE            0x00000000  //0K Bytes
#define BANK1_APP_DATA3_ADDR            0x00000000
#define BANK1_APP_DATA3_SIZE            0x00000000  //0K Bytes
#define BANK1_APP_DATA4_ADDR            0x00000000
#define BANK1_APP_DATA4_SIZE            0x00000000  //0K Bytes
#define BANK1_APP_DATA5_ADDR            0x00000000
#define BANK1_APP_DATA5_SIZE            0x00000000  //0K Bytes
#define BANK1_APP_DATA6_ADDR            0x00000000
#define BANK1_APP_DATA6_SIZE            0x00000000  //0K Bytes


#ifdef __cplusplus
}
#endif
/** @} */ /* _FLASH_MAP_H_ */
#endif
