#
#  Copyright (c) 2025, The OpenThread Authors.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. Neither the name of the copyright holder nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
if(${RT_PLATFORM} STREQUAL "rtl8852d")
    include(${PROJECT_SOURCE_DIR}/cmake/fetch_openthread.cmake)
endif()

if(${RT_PLATFORM} STREQUAL "bee4")
set(REALTEK_SDK_ROOT
    ${PROJECT_SOURCE_DIR}/third_party/Realtek/rtl87x2g_sdk
)
set(REALTEK_SDK_INCPATH
    ${REALTEK_SDK_ROOT}/subsys/freertos
    ${REALTEK_SDK_ROOT}/subsys/osif/inc
    ${REALTEK_SDK_ROOT}/subsys/bluetooth/bt_host/inc
    ${REALTEK_SDK_ROOT}/include/rtl87x2g
    ${REALTEK_SDK_ROOT}/include/rtl87x2g/nsc
    ${REALTEK_SDK_ROOT}/include/rtl87x2g/cmsis/Core/Include
    ${REALTEK_SDK_ROOT}/bsp/driver/
    ${REALTEK_SDK_ROOT}/bsp/driver/pinmux/inc
    ${REALTEK_SDK_ROOT}/bsp/driver/rcc/inc
    ${REALTEK_SDK_ROOT}/bsp/driver/nvic/inc
    ${REALTEK_SDK_ROOT}/bsp/driver/uart/inc
    ${REALTEK_SDK_ROOT}/bsp/driver/dma/inc
    ${REALTEK_SDK_ROOT}/bsp/driver/gpio/inc
    ${REALTEK_SDK_ROOT}/bsp/driver/spi/inc
    ${REALTEK_SDK_ROOT}/bsp/driver/i2c/inc
    ${REALTEK_SDK_ROOT}/bsp/driver/tim/inc
    ${REALTEK_SDK_ROOT}/bsp/driver/wdt/inc
    ${REALTEK_SDK_ROOT}/bsp/driver/ir/inc
    ${REALTEK_SDK_ROOT}/bsp/driver/project/rtl87x2g/inc
    ${REALTEK_SDK_ROOT}/bsp/sdk_lib/inc
    ${REALTEK_SDK_ROOT}/bsp/sdk_lib/inc_int
    ${REALTEK_SDK_ROOT}/bsp/power
)
endif()

if(${RT_PLATFORM} STREQUAL "bee3plus")
set(REALTEK_SDK_ROOT
    ${PROJECT_SOURCE_DIR}/../../../
)
set(REALTEK_SDK_INCPATH
    ${REALTEK_SDK_ROOT}/sdk/inc/platform
)
endif()

if(${RT_PLATFORM} STREQUAL "bb2ultra")
set(REALTEK_SDK_ROOT
    ${PROJECT_SOURCE_DIR}/../HoneyComb_crb14
)
set(REALTEK_SDK_INCPATH
    ${REALTEK_SDK_ROOT}/sdk/inc/app
    ${REALTEK_SDK_ROOT}/sdk/inc/os
    ${REALTEK_SDK_ROOT}/sdk/inc/rtl87x3eu/io
    ${REALTEK_SDK_ROOT}/sdk/inc/stack
    ${REALTEK_SDK_ROOT}/sdk/inc/rtl87x3eu/platform/cmsis
    ${REALTEK_SDK_ROOT}/sdk/inc/rtl87x3eu/platform
    ${REALTEK_SDK_ROOT}/sdk/inc/framework
    ${REALTEK_SDK_ROOT}/sdk/inc/framework/system
    ${REALTEK_SDK_ROOT}/sdk/inc/framework/audio
    ${REALTEK_SDK_ROOT}/sdk/inc/framework/bt
    ${REALTEK_SDK_ROOT}/sdk/inc/framework/remote
    ${REALTEK_SDK_ROOT}/sdk/inc/framework/console
    ${REALTEK_SDK_ROOT}/sdk/inc/rtl87x3eu/hal
    ${REALTEK_SDK_ROOT}/sdk/inc/bluetooth/gap/gap_lib
    ${REALTEK_SDK_ROOT}/sdk/inc/bluetooth/gap
    ${REALTEK_SDK_ROOT}/sdk/inc/bluetooth/leaudio
    ${REALTEK_SDK_ROOT}/sdk/inc/bluetooth/profile
    ${REALTEK_SDK_ROOT}/sdk/inc/bluetooth/profile/server
    ${REALTEK_SDK_ROOT}/sdk/inc/bluetooth/profile/client
    ${REALTEK_SDK_ROOT}/sdk/inc/service
    ${REALTEK_SDK_ROOT}/sdk/inc/service/gfps
    ${REALTEK_SDK_ROOT}/sdk/inc/service/bisto
    ${REALTEK_SDK_ROOT}/sdk/inc/service/xiaoai
    ${REALTEK_SDK_ROOT}/sdk/inc/service/xiaowei
    ${REALTEK_SDK_ROOT}/sdk/inc/service/ama
    ${REALTEK_SDK_ROOT}/sdk/inc/service/teams
    ${REALTEK_SDK_ROOT}/sdk/inc/service/teams/asp_service
    ${REALTEK_SDK_ROOT}/sdk/inc/service/teams/hid
    ${REALTEK_SDK_ROOT}/sdk/inc/hal/power
    ${REALTEK_SDK_ROOT}/sdk/bin/rtl87x3eu/upperstack_stamp
    ${REALTEK_SDK_ROOT}/sdk/bin/rtl87x3eu/flash_map_config
    ${REALTEK_SDK_ROOT}/sdk/inc/rtl87x3eu/hal/io
    ${REALTEK_SDK_ROOT}/sdk/inc/hal/platform
    ${REALTEK_SDK_ROOT}/sdk/inc/hal/io
    ${REALTEK_SDK_ROOT}/sdk/inc/framework/sysm
    ${REALTEK_SDK_ROOT}/sdk/inc/platform
    ${REALTEK_SDK_ROOT}/sdk/bin/rtl87x3eu
)
endif()
