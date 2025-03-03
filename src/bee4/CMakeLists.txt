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

if(${BUILD_BOARD_TARGET} STREQUAL "evb" OR ${BUILD_BOARD_TARGET} STREQUAL "evb_dual")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DBOARD_EVB")
endif()
if(${BUILD_BOARD_TARGET} STREQUAL "rtl8771guv")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DBOARD_RTL8771GUV -DBUILD_USB")
endif()
if(${BUILD_BOARD_TARGET} STREQUAL "rtl8771gtv")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DBOARD_RTL8771GTV")
endif()
if(${BUILD_BOARD_TARGET} STREQUAL "rtl8777g" OR ${BUILD_BOARD_TARGET} STREQUAL "rtl8777g_dual" OR ${BUILD_BOARD_TARGET} STREQUAL "rtl8777g_apple")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DBOARD_RTL8777G -DBUILD_USB")
endif()
if(${BUILD_BOARD_TARGET} STREQUAL "rtl8777g_cert")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DBOARD_RTL8777G -DBUILD_CERT")
endif()
if(${BUILD_BOARD_TARGET} STREQUAL "rtl8777g_rpc")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DBOARD_RTL8777G")
endif()
if(${BUILD_BOARD_TARGET} STREQUAL "rtl8777g_test")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DBOARD_RTL8777G")
endif()

if(${BUILD_TYPE} STREQUAL "dev")
add_library(bee4-internal
    # mac driver
    ${REALTEK_SDK_ROOT}/subsys/mac_driver/private/mac_driver_ext.c
    ${REALTEK_SDK_ROOT}/subsys/mac_driver/private/bee4/patch.c
)

target_include_directories(bee4-internal
    PRIVATE
        ${REALTEK_SDK_INCPATH}
        ${REALTEK_SDK_ROOT}/subsys/mac_driver
        ${REALTEK_SDK_ROOT}/subsys/mac_driver/portable/bee4
        ${CMAKE_CURRENT_SOURCE_DIR}/${BUILD_TARGET}
        ${CMAKE_CURRENT_SOURCE_DIR}/common
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/../ROMExport/rtl87x2g/inc
        ${PROJECT_SOURCE_DIR}/../ROMExport/rtl87x2g/inc/nsc
)
endif()

if (NOT OT_EXTERNAL_MBEDTLS)
target_compile_definitions(ot-config INTERFACE
        MBEDTLS_USER_CONFIG_FILE="${RT_PLATFORM}-mbedtls-config.h"
)
else()

add_library(bee4-mbedtls INTERFACE)

target_link_libraries(bee4-mbedtls
    INTERFACE
        ${REALTEK_SDK_ROOT}/lib/bee4/libmbedtls.a
        ${REALTEK_SDK_ROOT}/lib/bee4/libmbedx509.a
        ${REALTEK_SDK_ROOT}/lib/bee4/libmbedcrypto.a
)

target_include_directories(ot-config
    INTERFACE
        ${REALTEK_SDK_ROOT}/subsys/mbedtls
        ${REALTEK_SDK_ROOT}/subsys/mbedtls/repo/include
)

target_compile_definitions(ot-config INTERFACE
        MBEDTLS_CONFIG_FILE="mbedtls-config.h"
        MBEDTLS_USER_CONFIG_FILE="${RT_PLATFORM}-mbedtls-config.h"
)
endif()

add_library(openthread-bee4
    ../alarm.c
    ../diag.c
    ../entropy.c
    flash.c
    logging.c
    ../misc.c
    ../radio.c
    system.c
    uart.c
    # start up and entry point
    "${REALTEK_SDK_ROOT}/bsp/boot/rtl87x2g/startup_rtl.c"
    system_rtl.c
    zb_main.c
    thread_task.c
    # crypto
    # app
    dbg_printf.c
    # profile
    # peripheral
    "${REALTEK_SDK_ROOT}/bsp/driver/pinmux/src/rtl87x2g/rtl_pinmux.c"
    "${REALTEK_SDK_ROOT}/bsp/driver/rcc/src/rtl87x2g/rtl_rcc.c"
    "${REALTEK_SDK_ROOT}/bsp/driver/nvic/src/rtl87x2g/rtl_nvic.c"
    "${REALTEK_SDK_ROOT}/bsp/driver/tim/src/rtl_common/rtl_tim.c"
    "${REALTEK_SDK_ROOT}/bsp/driver/uart/src/rtl_common/rtl_uart.c"
    "${REALTEK_SDK_ROOT}/bsp/driver/dma/src/rtl_common/rtl_gdma.c"
    "${REALTEK_SDK_ROOT}/bsp/driver/gpio/src/rtl_common/rtl_gpio.c"
    # utils
    "${PROJECT_SOURCE_DIR}/openthread/examples/platforms/utils/mac_frame.cpp"
    "${PROJECT_SOURCE_DIR}/openthread/examples/platforms/utils/link_metrics.cpp"
    ../mac_frame_add.cpp
)

set_target_properties(
    openthread-bee4
    PROPERTIES
        C_STANDARD 99
        CXX_STANDARD 11
)

target_link_directories(openthread-bee4
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/${BUILD_TARGET}
        ${REALTEK_SDK_ROOT}/lib/${RT_PLATFORM}
        ${REALTEK_SDK_ROOT}/subsys/lwip/for_matter/lib
)

target_link_libraries(openthread-bee4
    PRIVATE
        ${OT_MBEDTLS}
        ot-config
        bee4-internal
        "${REALTEK_SDK_ROOT}/subsys/usb/usb_lib/lib/rtl87x2g/gcc/libusb.a"
        "${REALTEK_SDK_ROOT}/subsys/usb/usb_hal/lib/rtl87x2g/gcc/libusb_hal.a"
        "${REALTEK_SDK_ROOT}/subsys/bluetooth/gap_ext/lib/rtl87x2g/bt_host_0_0/gcc/libgap_utils.a"
        "${REALTEK_SDK_ROOT}/subsys/mac_driver/portable/bee4/bee4-internal.axf"
        "${REALTEK_SDK_ROOT}/bsp/driver/driver_lib/lib/rtl87x2g/gcc/librtl87x2g_io.a"
        "${REALTEK_SDK_ROOT}/bsp/sdk_lib/lib/rtl87x2g/gcc/librtl87x2g_sdk.a"
        "${REALTEK_SDK_ROOT}/bin/rtl87x2g/rom_lib/libROM.a"
        "${REALTEK_SDK_ROOT}/bin/rtl87x2g/rom_lib/libROM_NS.a"
        "${REALTEK_SDK_ROOT}/bin/rtl87x2g/rom_lib/ROM_CMSE_Lib.o"
)

set(TARGET_BUILD_LISTS
    "evb"
    "evb_dual"
    "rtl8777g_dual"
    "rtl8777g"
    "rtl8777g_cert"
    "rtl8777g_apple"
    "rtl8777g_rpc"
    "rtl8777g_test"
)

list(FIND TARGET_BUILD_LISTS "${BUILD_BOARD_TARGET}" FOUND_INDEX)

if(FOUND_INDEX GREATER -1)
    set(LD_FILE "${CMAKE_CURRENT_SOURCE_DIR}/${BUILD_TARGET}/app.ld.gen")
else()
    set(LD_FILE "${CMAKE_CURRENT_SOURCE_DIR}/${BUILD_TARGET}/app.ld")
endif()

target_link_options(openthread-bee4
    PUBLIC
        -T${LD_FILE}
        -Wl,--no-wchar-size-warning
        -Wl,--wrap,_malloc_r
        -Wl,--wrap,_free_r
        -Wl,--wrap,_realloc_r
        -Wl,--wrap,_calloc_r
        -Wl,--wrap,memcpy
        -Wl,--wrap,memset
        -Wl,--wrap,mac_EnterCritical_rom
        -Wl,--wrap,mac_ExitCritical_rom
        -Wl,--wrap,mac_RstRF_rom
        -Wl,--gc-sections
        -Wl,--print-memory-usage
        -Wl,--no-warn-rwx-segments
)

target_compile_definitions(openthread-bee4
    PUBLIC
        ${OT_PLATFORM_DEFINES}
        "BUILD_WITHOUT_FTL=1"
        "RT_PLATFORM_BEE4"
)

if(${OT_CMAKE_NINJA_TARGET} STREQUAL "ot-ncp-ftd")
    target_compile_definitions(openthread-bee4 PUBLIC "BUILD_NCP=1")
endif()
if(${OT_CMAKE_NINJA_TARGET} STREQUAL "ot-ncp-mtd")
    target_compile_definitions(openthread-bee4 PUBLIC "BUILD_NCP=1")
endif()
if(${OT_CMAKE_NINJA_TARGET} STREQUAL "ot-rcp")
    target_compile_definitions(openthread-bee4 PUBLIC "BUILD_RCP=1")
endif()

if(${ENABLE_CLI} STREQUAL "ON")
    target_compile_definitions(openthread-bee4 PUBLIC "ENABLE_CLI=1")
else()
if(${BUILD_BOARD_TARGET} STREQUAL "rtl8777g_cert")
    target_compile_definitions(openthread-bee4 PUBLIC "ENABLE_CLI=1")
else()
    target_compile_definitions(openthread-bee4 PUBLIC "ENABLE_CLI=0")
endif()
endif()

target_compile_options(openthread-bee4
    PRIVATE
        ${OT_CFLAGS}
        -Wno-attributes
)

target_include_directories(openthread-bee4
    PRIVATE
        ${OT_PUBLIC_INCLUDES}
        ${PROJECT_SOURCE_DIR}/src/core
        ${PROJECT_SOURCE_DIR}/openthread/examples/platforms
        ${REALTEK_SDK_INCPATH}
        ${REALTEK_SDK_ROOT}/subsys/mac_driver
        ${REALTEK_SDK_ROOT}/subsys/mac_driver/portable/bee4
        ${CMAKE_CURRENT_SOURCE_DIR}/${BUILD_TARGET}
        ${CMAKE_CURRENT_SOURCE_DIR}/common
        ${PROJECT_SOURCE_DIR}/third_party/Realtek/bee4/common
        ${REALTEK_SDK_ROOT}/subsys/usb/usb_hal/inc
        ${REALTEK_SDK_ROOT}/subsys/usb/usb_lib/inc/class
        ${REALTEK_SDK_ROOT}/subsys/usb/usb_lib/inc/composite
)

if(${BUILD_TYPE} STREQUAL "dev")
add_custom_command(
    TARGET openthread-bee4
    POST_BUILD
    COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libbee4-internal.a ${REALTEK_SDK_ROOT}/lib/${RT_PLATFORM}/
    COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libmbedcrypto.a ${REALTEK_SDK_ROOT}/lib/${RT_PLATFORM}/
    COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libmbedtls.a ${REALTEK_SDK_ROOT}/lib/${RT_PLATFORM}/
    COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libmbedx509.a ${REALTEK_SDK_ROOT}/lib/${RT_PLATFORM}/
    COMMAND [ -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/liblwip.a ] && cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/liblwip.a ${REALTEK_SDK_ROOT}/subsys/lwip/for_matter/lib/ || echo "liblwip.a not exist"
    COMMAND cd ${PROJECT_SOURCE_DIR}/openthread && git reset --hard HEAD && git clean -f -d
)
endif()

##################################################
# matter-port
##################################################

if(${OT_CMAKE_NINJA_TARGET} STREQUAL "matter-cli-ftd" OR ${OT_CMAKE_NINJA_TARGET} STREQUAL "matter-cli-mtd")

if(${ENABLE_PW_RPC} STREQUAL "ON")
    target_compile_definitions(openthread-bee4 PUBLIC "ENABLE_PW_RPC=1")
else()
    target_compile_definitions(openthread-bee4 PUBLIC "ENABLE_PW_RPC=0")
endif()

target_sources(openthread-bee4
    PRIVATE
    ${REALTEK_SDK_ROOT}/subsys/matter/vendor/kvmgr.c
    ${REALTEK_SDK_ROOT}/subsys/matter/vendor/matter_uart.c
    ${REALTEK_SDK_ROOT}/subsys/matter/vendor/matter_gpio.c
    ${REALTEK_SDK_ROOT}/subsys/matter/vendor/matter_kvs.c
    ${REALTEK_SDK_ROOT}/subsys/matter/vendor/matter_led.c
    ${REALTEK_SDK_ROOT}/subsys/matter/vendor/matter_utils.c
    ${REALTEK_SDK_ROOT}/subsys/matter/vendor/factory_data/factory.pb.c
    ${REALTEK_SDK_ROOT}/subsys/matter/vendor/factory_data/pb_common.c
    ${REALTEK_SDK_ROOT}/subsys/matter/vendor/factory_data/pb_decode.c
    ${REALTEK_SDK_ROOT}/subsys/matter/vendor/factory_data/pb_encode.c
    ${REALTEK_SDK_ROOT}/subsys/matter/vendor/matter_ble/matter_ble.c
    ${REALTEK_SDK_ROOT}/subsys/matter/vendor/matter_ble/matter_ble_service.c
    ${REALTEK_SDK_ROOT}/subsys/matter/vendor/matter_ble/matter_ble_task.c
)

if(matter_enable_ota_requestor)
target_include_directories(openthread-bee4
    PRIVATE
        ${REALTEK_SDK_ROOT}/subsys/dfu
)

target_sources(openthread-bee4
    PRIVATE
    ${REALTEK_SDK_ROOT}/subsys/dfu/dfu_common.c
    ${REALTEK_SDK_ROOT}/subsys/matter/vendor/matter_ota.c
)
endif()

if(matter_enable_ble_ota)
target_compile_definitions(openthread-bee4 PUBLIC
    SUPPORT_BLE_OTA=1
    SUPPORT_NORMAL_OTA=0
)

target_include_directories(openthread-bee4
    PRIVATE
        ${REALTEK_SDK_ROOT}/subsys/dfu
)

target_sources(openthread-bee4
    PRIVATE
    ${REALTEK_SDK_ROOT}/subsys/dfu/ble_dfu_transport.c
    ${REALTEK_SDK_ROOT}/subsys/dfu/dfu_app.c
    ${REALTEK_SDK_ROOT}/subsys/dfu/dfu_common.c
    ${REALTEK_SDK_ROOT}/subsys/dfu/dfu_service.c
    ${REALTEK_SDK_ROOT}/subsys/dfu/ota_service.c
)
endif()

if(matter_enable_factory_data EQUAL 1)
    target_compile_definitions(openthread-bee4 PUBLIC "CONFIG_FACTORY_DATA_ENCRYPTION=0")
elseif(matter_enable_factory_data EQUAL 2)
    target_compile_definitions(openthread-bee4 PUBLIC "CONFIG_FACTORY_DATA_ENCRYPTION=1")
endif()

if(matter_dac_key_encryption EQUAL 1)
    target_compile_definitions(openthread-bee4 PUBLIC "CONFIG_DAC_KEY_ENC=1")
endif()

target_link_options(openthread-bee4
    PUBLIC
        "LINKER:SHELL:-L ${PROJECT_BINARY_DIR}/lib"
        "LINKER:SHELL:-L ${PROJECT_BINARY_DIR}/chip/lib"
)

target_compile_definitions(openthread-bee4
    PUBLIC
        "BUILD_MATTER"
)

target_include_directories(openthread-bee4
    PRIVATE
        ${REALTEK_SDK_ROOT}/subsys/matter/vendor
        ${REALTEK_SDK_ROOT}/subsys/matter/vendor/matter_ble
)
else()
target_link_options(openthread-bee4
    PUBLIC
        -Wl,--wrap,otTaskletsSignalPending
)
endif()


##################################################
# cli-uart
##################################################

add_library(cli_uart
    ${PROJECT_SOURCE_DIR}/openthread/examples/apps/cli/cli_uart.cpp
)

target_include_directories(cli_uart
    PRIVATE
        $<TARGET_PROPERTY:ot-config,INTERFACE_INCLUDE_DIRECTORIES>
)

target_compile_definitions(cli_uart
    PRIVATE
        $<TARGET_PROPERTY:ot-config,INTERFACE_COMPILE_DEFINITIONS>
)

##################################################
# matter-cli-ftd
##################################################

add_executable(matter-cli-ftd
    main_ns.c
)

target_include_directories(matter-cli-ftd
    PRIVATE
        $<TARGET_PROPERTY:ot-config,INTERFACE_INCLUDE_DIRECTORIES>
)

target_compile_definitions(matter-cli-ftd
    PRIVATE
        $<TARGET_PROPERTY:ot-config,INTERFACE_COMPILE_DEFINITIONS>
        "DLPS_EN=0"
)

if (matter_enable_rpc)
    target_link_libraries(matter-cli-ftd
        -Wl,--start-group
        chip_main
        CHIP
        PwRpc
        cli_uart
        openthread-cli-ftd
        openthread-bee4
        openthread-ftd
        lwip
        -Wl,--end-group
        ot-config-ftd
        ot-config
    )
else()
    target_link_libraries(matter-cli-ftd
        -Wl,--start-group
        chip_main
        CHIP
        cli_uart
        openthread-cli-ftd
        openthread-bee4
        openthread-ftd
        lwip
        -Wl,--end-group
        ot-config-ftd
        ot-config
    )
endif()

##################################################
# matter-cli-mtd
##################################################

add_executable(matter-cli-mtd
    main_ns.c
    ${REALTEK_SDK_ROOT}/bsp/power/io_dlps.c
)

target_include_directories(matter-cli-mtd
    PRIVATE
        $<TARGET_PROPERTY:ot-config,INTERFACE_INCLUDE_DIRECTORIES>
)

target_compile_definitions(matter-cli-mtd
    PRIVATE
        $<TARGET_PROPERTY:ot-config,INTERFACE_COMPILE_DEFINITIONS>
        "DLPS_EN=0"
)

if(${BUILD_BOARD_TARGET} STREQUAL "rtl8777g" OR ${BUILD_BOARD_TARGET} STREQUAL "rtl8777g_dual" OR ${BUILD_BOARD_TARGET} STREQUAL "rtl8777g_apple")
target_compile_definitions(matter-cli-mtd
    PRIVATE
        "BUILD_USB"
)
endif()

target_link_libraries(matter-cli-mtd
    -Wl,--start-group
    chip_main
    CHIP
    cli_uart
    openthread-cli-mtd
    openthread-bee4
    openthread-mtd
    lwip
    -Wl,--end-group
    ot-config-mtd
    ot-config
)

if(${BUILD_TYPE} STREQUAL "dev")
add_custom_command(
    TARGET matter-cli-mtd
    POST_BUILD
    COMMAND cp -f ${PROJECT_SOURCE_DIR}/src/bee4/common/threading_alt.h ${REALTEK_SDK_ROOT}/subsys/mbedtls/repo/include/
   # COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libbee4-internal.a ${REALTEK_SDK_ROOT}/subsys/matter/lib/rtl87x2g/
    COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libchip_main.a ${REALTEK_SDK_ROOT}/subsys/matter/lib/rtl87x2g/
    COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libcli_uart.a ${REALTEK_SDK_ROOT}/subsys/matter/lib/rtl87x2g/
    COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/liblwip.a ${REALTEK_SDK_ROOT}/subsys/matter/lib/rtl87x2g/
   # COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libmbedcrypto.a ${REALTEK_SDK_ROOT}/subsys/mbedtls/lib/
   # COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libmbedtls.a ${REALTEK_SDK_ROOT}/subsys/mbedtls/lib/
   # COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libmbedx509.a ${REALTEK_SDK_ROOT}/subsys/mbedtls/lib/
    COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libopenthread-bee4.a ${REALTEK_SDK_ROOT}/subsys/matter/lib/rtl87x2g/
    COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libopenthread-cli-mtd.a ${REALTEK_SDK_ROOT}/subsys/matter/lib/rtl87x2g/
    COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libopenthread-mtd.a ${REALTEK_SDK_ROOT}/subsys/matter/lib/rtl87x2g/
    COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libtcplp-mtd.a ${REALTEK_SDK_ROOT}/subsys/matter/lib/rtl87x2g/
    COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/../chip/lib/libCHIP.a ${REALTEK_SDK_ROOT}/subsys/matter/lib/rtl87x2g/
)
endif()

# if(${BUILD_TYPE} STREQUAL "sdk")
# add_custom_command(
#     TARGET matter-cli-mtd
#     POST_BUILD
#     COMMAND cp -f ${PROJECT_SOURCE_DIR}/lib/bee4/libbee4-internal.a ${REALTEK_SDK_ROOT}/subsys/matter/lib/rtl87x2g/
#     COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libchip_main.a ${REALTEK_SDK_ROOT}/subsys/matter/lib/rtl87x2g/
#     COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libcli_uart.a ${REALTEK_SDK_ROOT}/subsys/matter/lib/rtl87x2g/
#     COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/liblwip.a ${REALTEK_SDK_ROOT}/subsys/matter/lib/rtl87x2g/
#     COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libopenthread-bee4.a ${REALTEK_SDK_ROOT}/subsys/matter/lib/rtl87x2g/
#     COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libopenthread-cli-mtd.a ${REALTEK_SDK_ROOT}/subsys/matter/lib/rtl87x2g/
#     COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libopenthread-mtd.a ${REALTEK_SDK_ROOT}/subsys/matter/lib/rtl87x2g/
#     COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libtcplp-mtd.a ${REALTEK_SDK_ROOT}/subsys/matter/lib/rtl87x2g/
#     COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/../chip/lib/libCHIP.a ${REALTEK_SDK_ROOT}/subsys/matter/lib/rtl87x2g/
# )
# endif()

##################################################
# ot-cli-ftd
##################################################
if(${OT_CMAKE_NINJA_TARGET} STREQUAL "ot-cli-ftd")

if(${BUILD_BOARD_TARGET} STREQUAL "rtl8777g_test")
target_sources(ot-cli-ftd
    PRIVATE
    ${REALTEK_SDK_ROOT}/../../../src/bee4/rtl8777g_test/main.c
    ${REALTEK_SDK_ROOT}/bsp/power/io_dlps.c
    ${REALTEK_SDK_ROOT}/subsys/bluetooth/gatt_profile/src/server/bas.c
    ${REALTEK_SDK_ROOT}/subsys/zb-mac-test/test/rtk_mac/simple_ble_service_nus.c
    ${REALTEK_SDK_ROOT}/subsys/bluetooth/gatt_profile/src/client/ancs_client.c
    ${REALTEK_SDK_ROOT}/samples/bluetooth/ble_peripheral/src/ancs.c
    ${REALTEK_SDK_ROOT}/samples/bluetooth/ble_peripheral/src/app_task.c
    ${REALTEK_SDK_ROOT}/samples/bluetooth/ble_peripheral/src/peripheral_app.c
)

target_include_directories(ot-cli-ftd
    PRIVATE
        ${REALTEK_SDK_ROOT}/../../../src/bee4/${BUILD_TARGET}
        ${REALTEK_SDK_ROOT}/bsp/driver/ir/inc
        ${REALTEK_SDK_ROOT}/subsys/bluetooth/gatt_profile/inc/server
        ${REALTEK_SDK_ROOT}/subsys/zb-mac-test/test/rtk_mac
        ${REALTEK_SDK_ROOT}/subsys/bluetooth/gatt_profile/inc/client
        ${REALTEK_SDK_ROOT}/samples/bluetooth/ble_peripheral/src
        ${REALTEK_SDK_ROOT}/bin/rtl87x2g/bt_host_image/bt_host_0_0
)

target_compile_options(ot-cli-ftd
    PRIVATE
        -include ${REALTEK_SDK_ROOT}/samples/bluetooth/ble_peripheral/src/app_flags.h
)
else()
target_sources(ot-cli-ftd
    PRIVATE
    ${REALTEK_SDK_ROOT}/../../../src/bee4/main_ns.c
)

target_include_directories(ot-cli-ftd
    PRIVATE
        ${REALTEK_SDK_ROOT}/../../../src/bee4/${BUILD_TARGET}
)
endif()
target_compile_definitions(ot-cli-ftd
    PRIVATE
        "DLPS_EN=0"
)

endif()

##################################################
# ot-cli-mtd
##################################################
if(${OT_CMAKE_NINJA_TARGET} STREQUAL "ot-cli-mtd")
target_sources(ot-cli-mtd
    PRIVATE
    ${REALTEK_SDK_ROOT}/../../../src/bee4/main_ns.c
    ${REALTEK_SDK_ROOT}/bsp/power/io_dlps.c
)

target_include_directories(ot-cli-mtd
    PRIVATE
        ${REALTEK_SDK_ROOT}/../../../src/bee4/${BUILD_TARGET}
)

target_compile_definitions(ot-cli-mtd
    PRIVATE
        "DLPS_EN=1"
)

if(${BUILD_BOARD_TARGET} STREQUAL "rtl8777g" OR ${BUILD_BOARD_TARGET} STREQUAL "rtl8777g_dual" OR ${BUILD_BOARD_TARGET} STREQUAL "rtl8777g_apple")
target_compile_definitions(ot-cli-mtd
    PRIVATE
        "BUILD_USB"
)
endif()

target_compile_definitions(openthread-bee4
    PRIVATE
        "DLPS_EN=1"
)
endif()

##################################################
# ot-rcp
##################################################
if(${OT_CMAKE_NINJA_TARGET} STREQUAL "ot-rcp")

if(${BUILD_TYPE} STREQUAL "dev")
add_library(rtk_sign
    STATIC
    ./internal/rtk_sign/rtk_sign.c
)

target_include_directories(rtk_sign
    PRIVATE
    ${PROJECT_SOURCE_DIR}/src/bee4/internal/rtk_sign
    ${REALTEK_SDK_INCPATH}
    ${REALTEK_SDK_ROOT}/subsys/osif/inc
    ${REALTEK_SDK_ROOT}/subsys/mbedtls/repo/include
)

target_link_libraries(openthread-bee4
    PRIVATE
        rtk_sign
        ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libmbedcrypto.a
)

add_custom_command(
    TARGET openthread-bee4
    POST_BUILD
    COMMAND cp -f ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/librtk_sign.a ${REALTEK_SDK_ROOT}/lib/bee4/
)
else()
target_link_libraries(openthread-bee4
    PRIVATE
        ${REALTEK_SDK_ROOT}/lib/bee4/librtk_sign.a
        ${REALTEK_SDK_ROOT}/lib/bee4/libmbedcrypto.a
)
endif()

target_include_directories(openthread-bee4
    PRIVATE
        ${REALTEK_SDK_ROOT}/subsys/cfu
        ${REALTEK_SDK_ROOT}/subsys/dfu
        ${PROJECT_SOURCE_DIR}/src/bee4/internal/cfu
        ${PROJECT_SOURCE_DIR}/src/bee4/internal/config_param
)

target_sources(openthread-bee4
    PRIVATE
    ${REALTEK_SDK_ROOT}/subsys/cfu/cfu.c
    ${PROJECT_SOURCE_DIR}/src/bee4/internal/cfu/cfu_application.c
    ${PROJECT_SOURCE_DIR}/src/bee4/internal/cfu/cfu_task.c
    ${PROJECT_SOURCE_DIR}/src/bee4/internal/cfu/cfu_callback.c
    "${REALTEK_SDK_ROOT}/subsys/dfu/dfu_common.c"
    ${PROJECT_SOURCE_DIR}/src/bee4/internal/config_param/config_param_handle.c
)

if(${BUILD_BOARD_TARGET} STREQUAL "rtl8771guv" OR ${BUILD_BOARD_TARGET} STREQUAL "rtl8777g")
target_sources(openthread-bee4
    PRIVATE
    ${PROJECT_SOURCE_DIR}/src/bee4/internal/cfu/usb_cfu_handle.c
)
elseif(${BUILD_BOARD_TARGET} STREQUAL "rtl8771gtv")
target_sources(openthread-bee4
    PRIVATE
    ${PROJECT_SOURCE_DIR}/src/bee4/internal/cfu/uart_cfu_handle.c
    ${PROJECT_SOURCE_DIR}/src/bee4/internal/cfu/loop_queue.c
    ${PROJECT_SOURCE_DIR}/src/bee4/internal/cfu/uart_transport.c
)
endif()

target_sources(ot-rcp
    PRIVATE
    ${REALTEK_SDK_ROOT}/../../../src/bee4/main_ns.c
)

target_include_directories(ot-rcp
    PRIVATE
        ${REALTEK_SDK_ROOT}/../../../src/bee4/${BUILD_TARGET}
)

target_compile_definitions(ot-rcp
    PRIVATE
        "DLPS_EN=0"
)

if(${BUILD_BOARD_TARGET} STREQUAL "rtl8771guv" OR ${BUILD_BOARD_TARGET} STREQUAL "rtl8777g")
target_compile_definitions(ot-rcp
    PRIVATE
        "BUILD_USB"
)
endif()

endif()
