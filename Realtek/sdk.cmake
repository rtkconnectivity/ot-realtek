if(${RT_PLATFORM} STREQUAL "rtl8852d")
    include(${PROJECT_SOURCE_DIR}/cmake/fetch_openthread.cmake)
endif()

if(${RT_PLATFORM} STREQUAL "bee4")
set(REALTEK_SDK_ROOT
    ${PROJECT_SOURCE_DIR}/../..
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
