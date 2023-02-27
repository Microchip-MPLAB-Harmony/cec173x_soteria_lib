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

#include "../mctp/mctp_base.h"
#include "../mctp/mctp.h"
#include "spdm_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SPDM_BSS0_ATTR                                     __attribute__((section("spdm_bss0")))
#define SPDM_BSS1_ATTR                                     __attribute__((section("spdm_bss1")))
#define SPDM_BSS2_ATTR                                     __attribute__((section("spdm_bss2")))
#define SPDM_STACK_ATTR 					               __attribute__((section("spdm_stack_buf")))
#define SPDM_BUF_ATTR 			     		               __attribute__((section("spdm_buf")))

#define PVT_KEY_CODE_LENGTH               (96U)
#define SPDM_SHA384_LEN                   (48U)
#define CURVE_384_SZ                      (48U)

/* Attestation - Hasd Pointer for Chain */
#define MAXIMUM_HEAD_PTR_CHAIN             8U

/* Attestation certificate Maximum*/
#define MAXIMUM_CERTIFICATE                64U

#define STATUS_OK 0U

#define is_add_safe(sum, aug_or_add) ((sum) < (aug_or_add) ? 0 : 1) // Coverity INT30-C Postcondition Test
#define is_sub_safe(ui_a, ui_b) ((ui_a) < (ui_b) ? 0 : 1) // Coverity INT30-C Precondition Test

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

#ifdef __cplusplus
}
#endif

#endif