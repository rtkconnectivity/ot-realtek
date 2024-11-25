/**
*********************************************************************************************************
*               Copyright(c) 2023, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file      rtk_sign.c
* @brief     This file provides rtk sign for user.
* @details
* @author
* @date      2023-11-27
* @version   v1.0
*********************************************************************************************************
*/

/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include "rtk_sign.h"
#include "mbedtls/ecdsa.h"
#include "mem_types.h"
#include "os_mem.h"
#include "trace.h"
#include "string.h"
#include "system_rtl876x.h"
#include "app_section.h"

/*============================================================================*
 *                                  Macros
 *============================================================================*/
#define ECDSA_RANDOM_DATA_LEN 32
#define RTL8771GUV 0x16
#define RTL8771GTV 0x18
#define ARRAY_TO_MBEDTLS_MPI_UINT(mpi_uint, a)             \
    {                                        \
        mpi_uint = ((mbedtls_mpi_uint)(*(a + 0)) << 24) +  \
                   ((mbedtls_mpi_uint)(*(a + 1)) << 16) + \
                   ((mbedtls_mpi_uint)(*(a + 2)) << 8) +  \
                   ((mbedtls_mpi_uint)(*(a + 3)) << 0);   \
    }

/*============================================================================*
 *                              Local Variables
 *============================================================================*/
static uint8_t ecdsa_random_data[ECDSA_RANDOM_DATA_LEN];
static uint8_t b_sign_data[64];

static uint8_t rtk_data[32] =
{
    0xde, 0x53, 0x93, 0x93, 0x7a, 0x9a, 0xb7, 0xc1,
    0xe3, 0x33, 0x50, 0xfd, 0xca, 0x2c, 0xb8, 0x0b,
    0x26, 0x7e, 0x84, 0x14, 0xcb, 0x1d, 0x96, 0x1a,
    0xd0, 0x40, 0xa9, 0xf8, 0x93, 0x1b, 0x51, 0x22
};

// static uint8_t rtk_data[32] = {
//     0x9e, 0x5e, 0xc2, 0xc9, 0xa8, 0xd5, 0x8b, 0x1a,
//     0xe2, 0x83, 0x8a, 0x67, 0x28, 0x67, 0x9a, 0xba,
//     0xfe, 0xf6, 0xf7, 0x1e, 0xfa, 0xeb, 0x5b, 0xc3,
//     0x12, 0xaf, 0x39, 0xd1, 0x32, 0x1c, 0xba, 0xa6
// };

/*============================================================================*
 *                              Global Variables
 *============================================================================*/

/*============================================================================*
 *                              Functions Declaration
 *============================================================================*/
static int rtk_sign_f_rng(void *p_rng, unsigned char *output, size_t output_len)
{
    size_t use_len = (output_len / 4 + 1) * 4;

    unsigned char buf[use_len];

    memset(buf, 0, use_len);

    for (int i = 0; i < (use_len / 4); ++i)
    {
        *(uint32_t *)(buf + i * 4) = rand();
    }

    memcpy(output, buf, output_len);

    return (0);
}

static void rtk_sign_convert_mpi1(uint8_t *sign_data, mbedtls_mpi *R, mbedtls_mpi *S)
{
    R->n = 16;
    R->s = 1;
    R->p = os_mem_zalloc(RAM_TYPE_DATA_ON, 16 * 4);
    memcpy(R->p, sign_data, 32);
    S->n = 16;
    S->s = 1;
    S->p = os_mem_zalloc(RAM_TYPE_DATA_ON, 16 * 4);
    memcpy(S->p, sign_data + 32, 32);
}

static void rtk_sign_swap_buf(const uint8_t *src, uint8_t *dst, uint32_t len)
{
    uint32_t i;

    for (i = 0; i < len; i++)
    {
        dst[len - 1 - i] = src[i];
    }
}

static void rtk_sign_print_random_data(const uint8_t *data, uint8_t data_len)
{
    if (data_len < ECDSA_RANDOM_DATA_LEN)
    {
        APP_PRINT_INFO1("[rtk_sign_print_random_data] data_len = %d", data_len);
        return;
    }
    APP_PRINT_INFO8("[rtk_sign_print_random_data] 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
                    *data, *(data + 1), *(data + 2), *(data + 3), *(data + 4), *(data + 5), *(data + 6), *(data + 7));
    APP_PRINT_INFO8("[rtk_sign_print_random_data] 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
                    *(data + 8), *(data + 9), *(data + 10), *(data + 11), *(data + 12), *(data + 13), *(data + 14),
                    *(data + 15));
    APP_PRINT_INFO8("[rtk_sign_print_random_data] 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
                    *(data + 16), *(data + 17), *(data + 18), *(data + 19), *(data + 20), *(data + 21), *(data + 22),
                    *(data + 23));
    APP_PRINT_INFO8("[rtk_sign_print_random_data] 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
                    *(data + 24), *(data + 25), *(data + 26), *(data + 27), *(data + 28), *(data + 20), *(data + 30),
                    *(data + 31));
}

static void rtk_sign_swap_uint8_to_mbedtls_mpi_uint(uint8_t *input, mbedtls_mpi_uint *output)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        ARRAY_TO_MBEDTLS_MPI_UINT(output[i], (input + 28 - i * 4));
        // APP_PRINT_INFO1("[rtk_sign_swap_uint8_to_mbedtls_mpi_uint] 0x%x", output[i]);
    }
}

// static void rtk_sign_verify(mbedtls_mpi r, mbedtls_mpi s, mbedtls_ecp_keypair *ctx_b)
// {
//     mbedtls_mpi_uint Qx_ext[8] = { 0xE1199CB1, 0xC1B51697, 0xB7DD20C6, 0xFFCA5D5F, 0x818621DE, 0x0E22B79C, 0x24EF0F44, 0xAD3FCA4D, };
//     mbedtls_mpi_uint Qy_ext[8] = { 0x190E2250, 0x8E1038D0, 0x1EBF483B, 0x9CAD99BF, 0x56E2B928, 0xD86100AE, 0x16C8DABD, 0x48D633B8, };
//     mbedtls_mpi_uint Qz_ext[1] = { 0x01 };

//     uint32_t b_hash_len = 32;
//     uint8_t b_hash_data1[32];
//     uint32_t ret;

//     rtk_sign_swap_buf(ecdsa_random_data, b_hash_data1, 32);

//     ctx_b->Q.X.n = 8;
//     ctx_b->Q.X.s = 1;
//     ctx_b->Q.X.p = Qx_ext;

//     ctx_b->Q.Y.n = 8;
//     ctx_b->Q.Y.s = 1;
//     ctx_b->Q.Y.p = Qy_ext;

//     ctx_b->Q.Z.n = 1;
//     ctx_b->Q.Z.s = 1;
//     ctx_b->Q.Z.p = Qz_ext;

//     memcpy(b_sign_data, r.p, 32);
//     memcpy(b_sign_data + 32, s.p, 32);

//     rtk_sign_convert_mpi1(b_sign_data, &r, &s);

//     ret = mbedtls_ecdsa_verify(&(ctx_b->grp), b_hash_data1, b_hash_len, &(ctx_b->Q), &r, &s);
//     if (ret != 0)
//     {
//         APP_PRINT_INFO1("ECDSA verify fail, %x", ret);
//     }
//     else
//     {
//         APP_PRINT_INFO0("ECDSA verify success");
//     }
// }

static void rtk_sign_create_sign_data(void)
{
    uint32_t ret;
    mbedtls_mpi_uint d_ext[8] = {0};

    mbedtls_mpi b_R, b_S;
    mbedtls_mpi_init(&b_R);
    mbedtls_mpi_init(&b_S);

    uint8_t b_hash_data[32];
    uint32_t b_hash_len = 32;

    mbedtls_ecp_keypair *ctx_b = (mbedtls_ecp_keypair *)os_mem_zalloc(RAM_TYPE_DATA_ON,
                                                                      sizeof(mbedtls_ecp_keypair));
    if (ctx_b != NULL)
    {
        mbedtls_ecp_keypair_init(ctx_b);
    }
    mbedtls_ecp_group_load(&(ctx_b->grp), mbedtls_ecp_curve_list()->grp_id);
    APP_PRINT_INFO1("[rtk_sign_create_sign_data] grp_id = %d", mbedtls_ecp_curve_list()->grp_id);

    rtk_sign_swap_uint8_to_mbedtls_mpi_uint(rtk_data, d_ext);

    ctx_b->d.p = d_ext;
    ctx_b->d.n = 8;
    ctx_b->d.s = 1;

    uint8_t b_hash_data1[32];
    rtk_sign_swap_buf(ecdsa_random_data, b_hash_data1, 32);

    // b sign data and send it to a
    ret = mbedtls_ecdsa_sign(&(ctx_b->grp), &b_R, &b_S, &(ctx_b->d), b_hash_data1, b_hash_len,
                             rtk_sign_f_rng,
                             NULL);
    if (ret != 0)
    {
        APP_PRINT_INFO0("[rtk_sign_create_sign_data] sign fail");
    }
    else
    {
        APP_PRINT_INFO0("[rtk_sign_create_sign_data] sign success");
    }

    memcpy(b_sign_data, b_R.p, 32);
    memcpy(b_sign_data + 32, b_S.p, 32);

    // rtk_sign_verify(b_R, b_S, ctx_b);

    os_mem_free(ctx_b);
    os_mem_free(b_R.p);
    os_mem_free(b_S.p);
}

static bool rtk_sign_check_package_id(void)
{
    extern uint8_t get_ic_type(void);
    uint8_t ic_type = get_ic_type();
    bool result = false;

    if ((RTL8771GUV == ic_type) || (RTL8771GTV == ic_type))
    {
        result = true;
    }

    // APP_PRINT_INFO1("[rtk_sign_check_package_id] ic_type=0x%x,", ic_type);
    APP_PRINT_INFO1("[rtk_sign_check_package_id] result=%d", result);

    return result;
}

/*============================================================================*
 *                              Global Functions
 *============================================================================*/
void rtk_sign_get_sign_data(uint8_t *result)
{
    APP_PRINT_INFO0("[rtk_sign_get_sign_data]");

    // rtk_sign_print_random_data(b_sign_data, ECDSA_RANDOM_DATA_LEN);
    memcpy(result, b_sign_data, 64);
}

void rtk_sign_set_random_data(const uint8_t *data, uint8_t data_len)
{
    APP_PRINT_INFO1("[rtk_sign_set_random_data] data_len=%d", data_len);
    rtk_sign_print_random_data(data, data_len);

    if (false == rtk_sign_check_package_id())
    {
        return;
    }

    if (ECDSA_RANDOM_DATA_LEN == data_len)
    {
        memcpy(ecdsa_random_data, data, ECDSA_RANDOM_DATA_LEN);
        rtk_sign_create_sign_data();
    }
}

/******************* (C) COPYRIGHT 2023 Realtek Semiconductor Corporation *****END OF FILE****/
