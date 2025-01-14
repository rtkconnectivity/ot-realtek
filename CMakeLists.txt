#
#  Copyright (c) 2020, The OpenThread Authors.
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

cmake_minimum_required(VERSION 3.10.2)
project(ot-bee VERSION 0.2.0 LANGUAGES C CXX ASM)

set(RT_PLATFORM_VALUES
    "bee4"
    "bee3plus"
    "rtl8852d"
    "rtl8922d"
    "bb2ultra"
)
set_property(CACHE RT_PLATFORM PROPERTY STRINGS ${RT_PLATFORM_VALUES})
if(NOT RT_PLATFORM IN_LIST RT_PLATFORM_VALUES)
    message(FATAL_ERROR "Please select a supported platform: ${RT_PLATFORM_VALUES}")
endif()

set(OT_PLATFORM_LIB "openthread-${RT_PLATFORM}")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

add_subdirectory(openthread)
include(Realtek/${BUILD_TYPE}.cmake)

target_compile_definitions(ot-config INTERFACE
    OPENTHREAD_CONFIG_FILE="openthread-core-${RT_PLATFORM}-config.h"
    OPENTHREAD_CORE_CONFIG_PLATFORM_CHECK_FILE="openthread-core-${RT_PLATFORM}-config-check.h"
)

if(${OT_CMAKE_NINJA_TARGET} STREQUAL "ot-rcp")
target_compile_definitions(ot-config INTERFACE
    OPENTHREAD_PROJECT_CORE_CONFIG_FILE="openthread-core-${BUILD_TARGET}-rcp-config.h"
)
else()
target_compile_definitions(ot-config INTERFACE
    OPENTHREAD_PROJECT_CORE_CONFIG_FILE="openthread-core-${RT_PLATFORM}-config.h"
)
endif()

target_include_directories(ot-config INTERFACE
    ${PROJECT_SOURCE_DIR}/openthread/examples/platforms
    ${PROJECT_SOURCE_DIR}/vendor/${RT_PLATFORM}
    ${REALTEK_SDK_INCPATH}
)

add_subdirectory(vendor/${RT_PLATFORM})
if(${OT_CMAKE_NINJA_TARGET} STREQUAL "matter-cli-ftd" OR ${OT_CMAKE_NINJA_TARGET} STREQUAL "matter-cli-mtd")
set (bee_sdk_root "${REALTEK_SDK_ROOT}")
set (bee_matter_root $ENV{BEE_MATTER})
set (matter_output_path ${PROJECT_BINARY_DIR})
set (matter_example_path $ENV{MATTER_EXAMPLE_PATH})
set (matter_config_path $ENV{MATTER_CONFIG_PATH})
get_filename_component(SDK_ROOT ${bee_sdk_root} REALPATH)
get_filename_component(CHIP_ROOT ${bee_matter_root} REALPATH)

set (matter_enable_rpc $ENV{MATTER_ENABLE_PW_RPC})
set (matter_enable_shell $ENV{MATTER_ENABLE_SHELL})
set (matter_enable_otcli $ENV{MATTER_ENABLE_OTCLI})
set (matter_enable_ota_requestor $ENV{MATTER_ENABLE_OTA_REQUESTOR})
set (matter_enable_persistentstorage_audit $ENV{MATTER_ENABLE_PERSISTENTSTORAGE_AUDIT})
set (matter_enable_openthread $ENV{MATTER_ENABLE_OPENTHREAD})

if(${OT_CMAKE_NINJA_TARGET} STREQUAL "matter-cli-ftd")
set (matter_enable_ftd 1)
endif()

if(${OT_CMAKE_NINJA_TARGET} STREQUAL "matter-cli-mtd")
set (matter_enable_mtd 1)
set (matter_enable_med $ENV{MATTER_ENABLE_MED})
endif()

set (matter_enable_factory_data $ENV{MATTER_ENABLE_FACTORY_DATA})
if(matter_enable_factory_data GREATER 0)
    set(matter_enable_factory_data 1)
    set(matter_dac_key_encryption $ENV{MATTER_DAC_KEY_ENCRYPTION})
endif()


set (openthread_root "${CMAKE_CURRENT_SOURCE_DIR}/openthread")

set(bee_lwip_path ${REALTEK_SDK_ROOT}/subsys/lwip/for_matter)
include(${bee_lwip_path}/lwip.cmake)

include(includepath.cmake)
include(${CHIP_ROOT}/src/app/chip_data_model.cmake)
include(${matter_example_path}/chip_main.cmake)
include(${matter_config_path}/chip.cmake)

add_dependencies(chip_main chip-gn)
endif()
