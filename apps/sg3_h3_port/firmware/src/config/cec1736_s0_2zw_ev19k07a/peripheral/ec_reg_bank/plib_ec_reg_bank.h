/*******************************************************************************
  Interface definition of EC Register Bank PLIB.

  Company:
    Microchip Technology Inc.

  File Name:
    plib_ec_reg_bank.h

  Summary:
    Interface definition of the EC Register Bank Plib.

  Description:
    This file defines the interface for the EC Register Bank Plib.
    It allows user to setup timeout duration and restart watch dog timer.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2022 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
// DOM-IGNORE-END

#ifndef PLIB_EC_REG_BANK_H    // Guards against multiple inclusion
#define PLIB_EC_REG_BANK_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Interface
// *****************************************************************************
// *****************************************************************************

typedef enum
{
    DISABLED,
    VTR_PAD_MON_DEB_CTRL_1MS,
    VTR_PAD_MON_DEB_CTRL_10MS,
    VTR_PAD_MON_DEB_CTRL_100MS,
}VTR_PAD_MON_DEB_CTRL;

typedef enum
{
    VTR1_PAD_MON_STS_PU = EC_REG_BANK_PD_MON_STS_VTR1_PU_STS_Msk,
    VTR1_PAD_MON_STS_PD = EC_REG_BANK_PD_MON_STS_VTR1_PD_STS_Msk,
    VTR1_PAD_MON_STS_CS = EC_REG_BANK_PD_MON_STS_VTR1_CS_STS_Msk,
}VTR1_PAD_MON_STS;

typedef enum
{
    VTR2_PAD_MON_STS_PU = EC_REG_BANK_PD_MON_STS_VTR2_PU_STS_Msk,
    VTR2_PAD_MON_STS_PD = EC_REG_BANK_PD_MON_STS_VTR2_PD_STS_Msk,
    VTR2_PAD_MON_STS_CS = EC_REG_BANK_PD_MON_STS_VTR2_CS_STS_Msk,
}VTR2_PAD_MON_STS;

typedef void (*EC_REG_BANK_CALLBACK)(uintptr_t context);

typedef struct
{
    EC_REG_BANK_CALLBACK   callback;
    uintptr_t      context;
} EC_REG_BANK_OBJECT ;

void EC_REG_BANK_Initialize( void );

uint32_t EC_REG_BANK_AHBErrorAddrGet(void);

void EC_REG_BANK_AHBErrorAddrClr(void);

void EC_REG_BANK_AHBErrorEnable(void);

void EC_REG_BANK_AHBErrorDisable(void);

void EC_REG_BANK_AltNVICVectorsEnable(void);

void EC_REG_BANK_VTR1PadMonDebounceCtrl(VTR_PAD_MON_DEB_CTRL ctrl);

void EC_REG_BANK_VTR1PadMonOverrideEn(void);

void EC_REG_BANK_VTR1PadMonOverrideDis(void);

void EC_REG_BANK_VTR1PadMonOverrideInpDis(void);

void EC_REG_BANK_VTR1PadMonOverrideInpEn(void);

void EC_REG_BANK_VTR1PadMonOverrideProtEn(void);

void EC_REG_BANK_VTR1PadMonOverrideProtDis(void);

void EC_REG_BANK_VTR1PadMonPUIntEn(void);

void EC_REG_BANK_VTR1PadMonPUIntDis(void);

void EC_REG_BANK_VTR1PadMonPDIntEn(void);

void EC_REG_BANK_VTR1PadMonPDIntDis(void);

void EC_REG_BANK_VTR2PadMonDebounceCtrl(VTR_PAD_MON_DEB_CTRL ctrl);

void EC_REG_BANK_VTR2PadMonOverrideEn(void);

void EC_REG_BANK_VTR2PadMonOverrideDis(void);

void EC_REG_BANK_VTR2PadMonOverrideInpDis(void);

void EC_REG_BANK_VTR2PadMonOverrideInpEn(void);

void EC_REG_BANK_VTR2PadMonOverrideProtEn(void);

void EC_REG_BANK_VTR2PadMonOverrideProtDis(void);

void EC_REG_BANK_VTR2PadMonPUIntEn(void);

void EC_REG_BANK_VTR2PadMonPUIntDis(void);

void EC_REG_BANK_VTR2PadMonPDIntEn(void);

void EC_REG_BANK_VTR2PadMonPDIntDis(void);

VTR1_PAD_MON_STS EC_REG_BANK_VTR1PadMonStatusGet(void);

void EC_REG_BANK_VTR1PadMonStatusClr(VTR1_PAD_MON_STS statusBitMask);

VTR2_PAD_MON_STS EC_REG_BANK_VTR2PadMonStatusGet(void);

void EC_REG_BANK_VTR2PadMonStatusClr(VTR2_PAD_MON_STS statusBitMask);



void EC_REG_BANK_VTR1_CallbackRegister( EC_REG_BANK_CALLBACK callback, uintptr_t context );

void VTR1_PAD_MON_GRP_InterruptHandler(void);


void EC_REG_BANK_VTR2_CallbackRegister( EC_REG_BANK_CALLBACK callback, uintptr_t context );

void VTR2_PAD_MON_GRP_InterruptHandler(void);




// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
// DOM-IGNORE-END

#endif // PLIB_EC_REG_BANK_H
