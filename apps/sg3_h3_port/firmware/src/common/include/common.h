/*****************************************************************************
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

#ifndef COMMON_H
#define COMMON_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <xc.h>

#include "trace.h"

#include "peripheral/gpio/plib_gpio.h"
#include "peripheral/qmspi/plib_qmspi_common.h"
#include "peripheral/qmspi/plib_qmspi0.h"
#include "peripheral/qmspi/plib_qmspi1.h"
#include "peripheral/wdt/plib_wdt.h"
#include "peripheral/ec_reg_bank/plib_ec_reg_bank.h"
#include "interrupt/interrupt_api.h"
#include "vtr_mon/vtr_mon_api.h"
#include "gpio/gpio_api.h"
#include "uart/uart_api.h"
#include "qmspi/qmspi_api.h"

#include "ahb_api_mpu.h"
#include "rom_api_mpu.h"
#include "pmci.h"
#include "platform.h"

#define SHA_384_LEN     48u

#include "../data_iso/data_iso_checks_rom_api.h"
#include "config/cec1736_s0_2zw_ev19k07a/mctp/mctp.h"
#include "spt/spt_drv.h"
extern void zero_out_bss(void);
#ifdef __cplusplus
extern "C" {
#endif

/** AP's */
enum APx
{
    AP_0 = 0,
    AP_1,
    AP_MAX
};

/** Components */
enum COMPONENTS
{
    COMPONENT_0 = 0,
    COMPONENT_1,
    COMPONENT_MAX
};

#define NO_OF_BYTE_MATCH_SUPPORT_IN_INT_FLASH 2
#define NO_OF_APFW_SUPPORT_IN_EXT 16

#ifdef __cplusplus
}
#endif

#endif /* COMMON_H */
