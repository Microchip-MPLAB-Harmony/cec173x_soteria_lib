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

#ifndef PLDM_COMMON_H
#define PLDM_COMMON_H

#include <stdint.h>
#include <stddef.h>
#include "definitions.h"
#include "pldm_config.h"
#include "../spdm/spdm_task.h"
#include "common.h"
#include "pmci.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PLDM_BSS0_ATTR                                     __attribute__((section("pldm_bss0")))
#define PLDM_BSS1_ATTR                                     __attribute__((section("pldm_bss1")))
#define PLDM_BSS2_ATTR                                     __attribute__((section("pldm_bss2")))

#define PLDM_COMP_IDENTIFIER_TAG0 0x1020
#define PLDM_COMP_IDENTIFIER_TAG1 0x1021
#define PLDM_COMP_IDENTIFIER_KHB_TAG0 0x1024
#define PLDM_COMP_IDENTIFIER_KHB_TAG1 0x1025
#define PLDM_COMP_IDENTIFIER_KHB1_TAG1 0x102D

#define PLDM_COMP_IDENTIFIER_APCFG0 0x4000
#define PLDM_COMP_IDENTIFIER_APCFG1 0x4011

#define PLDM_COMP_IDENTIFIER_HT0_AP0C0 0x8000
#define PLDM_COMP_IDENTIFIER_HT1_AP0C0 0x8001
#define PLDM_COMP_IDENTIFIER_HT2_AP0C0 0x8002
#define PLDM_COMP_IDENTIFIER_HT0_AP0C1 0x8010
#define PLDM_COMP_IDENTIFIER_HT1_AP0C1 0x8011
#define PLDM_COMP_IDENTIFIER_HT2_AP0C1 0x8012
#define PLDM_COMP_IDENTIFIER_HT0_AP1C0 0x8100
#define PLDM_COMP_IDENTIFIER_HT1_AP1C0 0x8101
#define PLDM_COMP_IDENTIFIER_HT2_AP1C0 0x8102
#define PLDM_COMP_IDENTIFIER_HT0_AP1C1 0x8110
#define PLDM_COMP_IDENTIFIER_HT1_AP1C1 0x8111
#define PLDM_COMP_IDENTIFIER_HT2_AP1C1 0x8112

#define PLDM_COMP_IDENTIFIER_AP_BA_PTR0 0x3000
#define PLDM_COMP_IDENTIFIER_AP_BA_PTR1 0x3001

#define PLDM_COMP_IDENTIFIER_AP_KHB0 0x6000
#define PLDM_COMP_IDENTIFIER_AP_KHB1 0x6001

#define PLDM_COMP_IDENTIFIER_APFW_0 0x2000

#define PLDM_COMP_IDENTIFIER_BYTE_MATCH_INT_SPI_BASE 0x5000

#define INVALID_ECFWKHB_IDENTIFIER(x) ((x != PLDM_COMP_IDENTIFIER_KHB_TAG0) && \
                                       (x != PLDM_COMP_IDENTIFIER_KHB_TAG1) && \
                                       (x != PLDM_COMP_IDENTIFIER_KHB1_TAG1))

#define INVALID_HASH_COMP_IDENTIFIER(x) ((x != PLDM_COMP_IDENTIFIER_HT0_AP0C0) && \
                                       (x != PLDM_COMP_IDENTIFIER_HT1_AP0C0) && \
                                       (x != PLDM_COMP_IDENTIFIER_HT2_AP0C0) && \
                                       (x != PLDM_COMP_IDENTIFIER_HT0_AP0C1) && \
                                       (x != PLDM_COMP_IDENTIFIER_HT1_AP0C1) && \
                                       (x != PLDM_COMP_IDENTIFIER_HT2_AP0C1) && \
                                       (x != PLDM_COMP_IDENTIFIER_HT0_AP1C0) && \
                                       (x != PLDM_COMP_IDENTIFIER_HT1_AP1C0) && \
                                       (x != PLDM_COMP_IDENTIFIER_HT2_AP1C0) && \
                                       (x != PLDM_COMP_IDENTIFIER_HT0_AP1C1) && \
                                       (x != PLDM_COMP_IDENTIFIER_HT1_AP1C1) && \
                                       (x != PLDM_COMP_IDENTIFIER_HT2_AP1C1))

/** Components */
enum COMPONENTS
{
    COMPONENT_0 = 0,
    COMPONENT_1,
    COMPONENT_MAX
};

/** AP's */
enum APx
{
    AP_0 = 0,
    AP_1,
    AP_MAX
};

#define NO_OF_BYTE_MATCH_SUPPORT_IN_INT_FLASH 2

#ifdef __cplusplus
}
#endif

#endif