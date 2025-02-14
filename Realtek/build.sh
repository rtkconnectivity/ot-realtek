#!/bin/bash
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

set -euxo pipefail

export OUT_FOLDER="$1"
export OT_SRCDIR="$(pwd)/third_party/openthread/ot-realtek"
BOARD_TARGET="$2"
APP_NAME="$3"
OT_CMAKE_NINJA_TARGET="$4"

readonly OT_OPTIONS=(
    "-DCMAKE_BUILD_TYPE=Release"
    "-DOT_PLATFORM=external"
    "-DOT_SLAAC=ON"

    "-DOT_BORDER_AGENT=ON"
    "-DOT_BORDER_ROUTER=ON"
    "-DOT_COMMISSIONER=ON"
    "-DOT_JOINER=ON"
    "-DOT_DNS_CLIENT=ON"
    "-DOT_DHCP6_SERVER=ON"
    "-DOT_DHCP6_CLIENT=ON"
    "-DOT_COAP=ON"
    "-DOT_COAPS=ON"
    "-DOT_COAP_OBSERVE=ON"
#    "-DOT_DIAGNOSTIC=ON"
    "-DOT_LINK_RAW=ON"
    "-DOT_MAC_FILTER=ON"
#    "-DOT_MTD_NETDIAG=ON"
    "-DOT_SERVICE=ON"
    "-DOT_UDP_FORWARD=ON"
    "-DOT_ECDSA=ON"
    "-DOT_SNTP_CLIENT=ON"
    "-DOT_CHILD_SUPERVISION=ON"
    "-DOT_JAM_DETECTION=ON"
    "-DOT_LOG_LEVEL_DYNAMIC=ON"
    "-DOT_EXTERNAL_HEAP=ON"
    "-DOT_SRP_CLIENT=ON"
    "-DOT_DUA=ON"
    "-DOT_BACKBONE_ROUTER=ON"
    "-DOT_MLR=ON"
    "-DOT_CSL_RECEIVER=ON"
    "-DOT_LINK_METRICS_INITIATOR=ON"
    "-DOT_LINK_METRICS_SUBJECT=ON"
    "-DOT_CHANNEL_MONITOR=OFF"
)
readonly OT_OPTIONS_MINIMAL=(
    "-DCMAKE_BUILD_TYPE=Release"
    "-DOT_PLATFORM=external"
    "-DOT_SLAAC=ON"
    "-DOT_LOG_LEVEL_DYNAMIC=ON"
    "-DOT_CSL_RECEIVER=ON"
    "-DOT_CHANNEL_MONITOR=OFF"
)

#to add more by modify ld
is_create_ld_gen()
{
    local ldgentargets=("rtl8777g" "evb" "evb_dual" "rtl8777g_dual" \
    "rtl8777g_cert" "rtl8777g_apple" "rtl8777g_rpc" "rtl8777g_test")

    for ldgentarget in "${ldgentargets[@]}"; do
        if [ "${BOARD_TARGET}" == "${ldgentarget}" ]; then
            return 0
        fi
    done

    return 1
}

is_rtl8777g_signal_target()
{
    if [ ${BOARD_TARGET} == "rtl8777g" ] || [ ${BOARD_TARGET} == "rtl8777g_cert" ] \
    || [ ${BOARD_TARGET} == "rtl8777g_apple" ] || [ ${BOARD_TARGET} == "rtl8777g_rpc" ]; then
        return 0
    else
        return 1
    fi
}

is_dual_target()
{
    if [ ${BOARD_TARGET} == "evb_dual" ] || [ ${BOARD_TARGET} == "rtl8777g_dual" ]; then
        return 0
    else
        return 1
    fi
}

build_matter()
{
    ENABLE_PERSISTENTSTORAGE_AUDIT=${ENABLE_PERSISTENTSTORAGE_AUDIT:-0}
    export MATTER_ENABLE_PERSISTENTSTORAGE_AUDIT=$ENABLE_PERSISTENTSTORAGE_AUDIT
    export BEE_MATTER=${OT_SRCDIR}/../../..
    export MATTER_CONFIG_PATH=${BEE_MATTER}/config/realtek_bee

    CONFIG_CHIP_FACTORY_DATA=${CONFIG_CHIP_FACTORY_DATA:-0}
    CONFIG_CHIP_DAC_KEY_ENCRYPTION=${CONFIG_CHIP_DAC_KEY_ENCRYPTION:-0}
    export MATTER_ENABLE_FACTORY_DATA=$CONFIG_CHIP_FACTORY_DATA
    export MATTER_DAC_KEY_ENCRYPTION=$CONFIG_CHIP_DAC_KEY_ENCRYPTION

    CONFIG_MED_DEVICE=${CONFIG_MED_DEVICE:-0}
    if [ "$OT_CMAKE_NINJA_TARGET" == "matter-cli-mtd" ]; then
        export MATTER_ENABLE_MED=$CONFIG_MED_DEVICE
    else
        unset MATTER_ENABLE_MED
    fi

    #only for signal bank
    CONFIG_BLE_OTA=${CONFIG_BLE_OTA:-0}
    export MATTER_ENABLE_BLE_OTA=$CONFIG_BLE_OTA

    if is_dual_target; then
        export MATTER_ENABLE_OTA_REQUESTOR=1
    else
        export MATTER_ENABLE_OTA_REQUESTOR=0
    fi

    export MATTER_EXAMPLE_PATH=${BEE_MATTER}/examples/${APP_NAME}/realtek_bee
    echo "MATTER_EXAMPLE_PATH at: ${MATTER_EXAMPLE_PATH}"

    if [ "${MATTER_UART_SELECT}" == "pw_rpc" ]; then
        export MATTER_ENABLE_PW_RPC=1
        export MATTER_ENABLE_SHELL=0
        export MATTER_ENABLE_OTCLI=0
    elif [ "${MATTER_UART_SELECT}" == "shell" ]; then
        export MATTER_ENABLE_PW_RPC=0
        export MATTER_ENABLE_SHELL=1
        export MATTER_ENABLE_OTCLI=1
    else
        export MATTER_ENABLE_PW_RPC=0
        export MATTER_ENABLE_SHELL=0
        export MATTER_ENABLE_OTCLI=0
    fi
}

build()
{
    mkdir -p "${OUT_FOLDER}"
    cd "${OUT_FOLDER}"

    cmake -GNinja -DOT_COMPILE_WARNING_AS_ERROR=ON "$@" "${OT_SRCDIR}"

    if [[ -n ${OT_CMAKE_NINJA_TARGET[*]} ]]; then
        ninja "${OT_CMAKE_NINJA_TARGET[@]}"
    else
        ninja
    fi
}

pre_build()
{
    if is_create_ld_gen >/dev/null 2>&1; then
       ${OT_SRCDIR}/Realtek/pre_build ${BUILD_BANK} ${target}
    fi
}

post_build()
{
    if is_create_ld_gen >/dev/null 2>&1; then
        ${OT_SRCDIR}/Realtek/post_build ${BUILD_BANK} ${OT_CMAKE_NINJA_TARGET}
        rm -f ${OT_SRCDIR}/src/${platform}/${target}/*.gen
    fi
}

set_sdk_path()
{
   case "${platform}" in
       bee4)
           export REALTEK_SDK_PATH="${OT_SRCDIR}/third_party/Realtek/rtl87x2g_sdk/"
           ;;
       *)
   esac
}

main()
{
    local platform="bee4"
    local build_type="sdk"
    local target="$2"
    local options

    MATTER_UART_SELECT="noshell"

    if is_rtl8777g_signal_target; then
        target="rtl8777g"
    fi

    if is_create_ld_gen >/dev/null 2>&1; then
        BUILD_BANK="bank0"
    fi

    if is_dual_target;then
        OTA_VERSION="$5"
        if [ $# -eq 6 ];then
            MATTER_UART_SELECT="$6"
        fi
    elif [ $# -eq 5 ];then
        MATTER_UART_SELECT="$5"
    fi

    set_sdk_path

    options+=("-DBUILD_TYPE=${build_type}")
    options+=("${OT_OPTIONS[@]}")
    options+=("-DRT_PLATFORM=${platform}")
    options+=("-DBUILD_TARGET=${target}")
    options+=("-DOT_NCP_VENDOR_HOOK_SOURCE=${OT_SRCDIR}/src/${platform}/example_vendor_hook.cpp")
    options+=("-DOT_CMAKE_NINJA_TARGET=${OT_CMAKE_NINJA_TARGET}")
    case "${platform}" in
        bee4)
        options+=("-DBUILD_BOARD_TARGET=${BOARD_TARGET}")
        options+=("-DOT_EXTERNAL_MBEDTLS=bee4-mbedtls")
        case "${OT_CMAKE_NINJA_TARGET}" in
            matter-cli-ftd | matter-cli-mtd)
                case "${BOARD_TARGET}" in
                    rtl8777g_apple)
                        cd ${OT_SRCDIR}/openthread && git checkout 171f94c30 && cd ${OT_SRCDIR}
                        ;;
                    *)
                        cd ${OT_SRCDIR}/openthread && git checkout thread-reference-20230706 && cd ${OT_SRCDIR}
                        ;;
                esac
                build_matter
                options+=("-DCMAKE_TOOLCHAIN_FILE=src/${platform}/arm-none-eabi.cmake")
                options+=("-DOPENTHREAD_ENABLE=ON")
                case "${MATTER_UART_SELECT}" in
                     shell)
                        options+=("-DENABLE_CLI=ON")
                        options+=("-DENABLE_PW_RPC=OFF")
                        ;;
                     pw_rpc)
                        options+=("-DENABLE_CLI=ON")
                        options+=("-DENABLE_PW_RPC=ON")
                        ;;
                     noshell)
                        options+=("-DENABLE_CLI=OFF")
                        options+=("-DENABLE_PW_RPC=OFF")
                        ;;
                esac
                ;;
            *)
                ;;
        esac
        ;;
    esac

    options+=("$@")

    pre_build
    build "${options[@]}"
    post_build

    #for bank1 build
    if is_dual_target >/dev/null 2>&1;then
        rm -rf ${OUT_FOLDER}/bank0
        mv ${OUT_FOLDER}/build ${OUT_FOLDER}/bank0
        mkdir -p ${OUT_FOLDER}/ota
        cp -f ${OUT_FOLDER}/bank0/bin/*MP_dev*.bin ${OUT_FOLDER}/ota/
        mv ${OUT_FOLDER}/bank0 ${OUT_FOLDER}/build

        BUILD_BANK="bank1"
        pre_build
        build "${options[@]}"
        post_build

        ${OT_SRCDIR}/Realtek/matter_ota_pack ${target} ${APP_NAME} ${OTA_VERSION}
    elif [ ${MATTER_ENABLE_OTA_REQUESTOR} -eq 1 ]; then
        mkdir -p ${OUT_FOLDER}/ota
        ${OT_SRCDIR}/Realtek/matter_ota_pack ${target} ${APP_NAME}
    fi
}

main "$@"
