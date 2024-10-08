/****************************************************************************
* Copyright (c) 2022 Microchip Technology Inc. and its subsidiaries.
* You may use this software and any derivatives exclusively with
* Microchip products.
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".
* NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
* INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
* AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.
* TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
* CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF
* FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
* MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE
* OF THESE TERMS.
*****************************************************************************/

#ifndef SPDM_COMMON_H
#define SPDM_COMMON_H

#include "mctp_base.h"
#include "mctp.h"
#include "spdm_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SPDM_BSS0_ATTR                                     __attribute__((section("spdm_bss0")))
#define SPDM_BSS0_ATTR_1024ALIGNED                           __attribute__((section("spdm_bss0_1024aligned")))
#define SPDM_BSS0_ATTR_8ALIGNED                             __attribute__((section("spdm_bss0_8aligned")))
#define SPDM_BSS1_ATTR                                     __attribute__((section("spdm_bss1")))
#define SPDM_BSS1_ATTR_8ALIGNED                             __attribute__((section("spdm_bss1_8aligned")))
#define SPDM_BSS1_ATTR_4ALIGNED                             __attribute__((section("spdm_bss1_4aligned")))
#define SPDM_BSS2_ATTR                                     __attribute__((section("spdm_bss2")))
/*src to lib*/
#define SPDM_STACK_ATTR                         __attribute__((section("spdm_stack")))
#define SPDM_BSS2_ATTR_8ALIGNED                 __attribute__((section("spdm_bss2_8aligned")))
#define SPT_STACK_ATTR                          __attribute__((section("spt_stack")))
#define SPT_BSS_ATTR_256ALIGN                   __attribute__((section("spt_bss_attr_256align")))
#define SHA384_LEN_BYTES                   (48U)
#define SB_KEY_HASH_SIZE_MAX                   (48U)
#define SHA_MODE_384    (4u)
/* --------------- AP_CFG Authentication Information ---------------- */

/* AP_CFG region authentication algo - see enum SB_PARSER_AUTH_ALGO*/
#define APCFG_AUTH_TYPE    SB_AUTHALGO_ECDSA_P384

/* Hash Table authentication algo - see enum SB_PARSER_AUTH_ALGO*/
#define HASH_TABLE_AUTH_TYPE            SB_AUTHALGO_ECDSA_P384
/*end*/
#define PVT_KEY_CODE_LENGTH               (96U)
#define PUB_KEY_CODE_LENGTH               (96U)
#define SPDM_SHA384_LEN                   (48U)
#define CURVE_384_SZ                      (48U)
#define DHE_SECRET_SIZE                   (96U)

/* Attestation - Hasd Pointer for Chain */
#define MAXIMUM_HEAD_PTR_CHAIN             8U

/* Attestation certificate Maximum*/
#define MAXIMUM_CERTIFICATE                64U


typedef union
{
    struct
    {
        uint8_t signature_r_term[CURVE_384_SZ];
        uint8_t signature_s_term[CURVE_384_SZ];
    };
    uint8_t ecdsa_signature[CURVE_384_SZ*2];
} ecdsa_signature_t;

typedef struct CFG_CERT
{
    /* Attestation Head Pointer for Chain */
    uint8_t head_ptr_chain[MAXIMUM_HEAD_PTR_CHAIN];      // 0x412

    /* Tail Pointer for Certificate n */
    uint8_t tail_ptr_certificate[MAXIMUM_CERTIFICATE];    // 0x41A

    /* Certificate chain to slot assignment */
    uint8_t certificate_chain_slot[4];                   // 0x45A

} __attribute__((packed)) CFG_CERT;

/******************************************************************************/
/** safe_subraction_16
* Process safe subtraction of two variables of uint16
* @param uint16_t  - operand 1
* @param uint16_t  - operand 2
* @param uint16_t* - resultant pointer
* @return uint8_t  - 0 success
*******************************************************************************/
uint8_t safe_subraction_16(uint16_t minuend, uint16_t subtrahend, uint16_t * rslt);

/******************************************************************************/
/** safe_subraction_8
* Process safe subtraction of two variables of uint8
* @param uint8_t  - operand 1
* @param uint8_t  - operand 2
* @param uint8_t* - resultant pointer
* @return uint8_t  - 0 success
*******************************************************************************/
uint8_t safe_subraction_8(uint8_t minuend, uint8_t subtrahend, uint8_t * rslt);

#ifdef __cplusplus
}
#endif

#endif