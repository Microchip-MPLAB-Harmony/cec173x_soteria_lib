/*******************************************************************************
  EC Register Bank Peripheral Library

  Company:
    Microchip Technology Inc.

  File Name:
    plib_ec_reg_bank.c

  Summary:
    EC Register Bank Source File

  Description:
    None

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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "device.h"
#include "interrupts.h"
#include "peripheral/ecia/plib_ecia.h"
#include "plib_ec_reg_bank.h"

// *****************************************************************************
// *****************************************************************************
// Section: EC_REG_BANK Implementation
// *****************************************************************************
// *****************************************************************************

EC_REG_BANK_OBJECT ec_reg_bank[2] = {0};

void EC_REG_BANK_Initialize( void )
{
    
    
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_CTRL = EC_REG_BANK_PD_MON_CTRL_VTR1_PROTECN_Msk | EC_REG_BANK_PD_MON_CTRL_VTR2_PROTECN_Msk;
    
    
}
uint32_t EC_REG_BANK_AHBErrorAddrGet(void)
{
    return EC_REG_BANK_REGS->EC_REG_BANK_AHB_ERR_ADDR;
}

void EC_REG_BANK_AHBErrorAddrClr(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_AHB_ERR_ADDR = 0;
}

void EC_REG_BANK_AHBErrorEnable(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_AHB_ERR_CTRL = 0;
}

void EC_REG_BANK_AHBErrorDisable(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_AHB_ERR_CTRL = 0;
}

void EC_REG_BANK_AltNVICVectorsEnable(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_INTR_CTRL = 1;
}

void EC_REG_BANK_VTR1PadMonDebounceCtrl(VTR_PAD_MON_DEB_CTRL ctrl)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_CTRL = (ctrl << EC_REG_BANK_PD_MON_CTRL_CTRL_VTR1_Pos);
}

void EC_REG_BANK_VTR1PadMonOverrideEn(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_CTRL |= EC_REG_BANK_PD_MON_CTRL_OVRD_VTR1_Msk;
}

void EC_REG_BANK_VTR1PadMonOverrideDis(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_CTRL &= ~EC_REG_BANK_PD_MON_CTRL_OVRD_VTR1_Msk;
}

void EC_REG_BANK_VTR1PadMonOverrideInpDis(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_CTRL |= EC_REG_BANK_PD_MON_CTRL_VTR1_INPT_DIS_Msk;
}

void EC_REG_BANK_VTR1PadMonOverrideInpEn(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_CTRL &= ~EC_REG_BANK_PD_MON_CTRL_VTR1_INPT_DIS_Msk;
}

void EC_REG_BANK_VTR1PadMonOverrideProtEn(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_CTRL &= ~EC_REG_BANK_PD_MON_CTRL_VTR1_PROTECN_Msk;
}

void EC_REG_BANK_VTR1PadMonOverrideProtDis(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_CTRL |= EC_REG_BANK_PD_MON_CTRL_VTR1_PROTECN_Msk;
}

void EC_REG_BANK_VTR1PadMonPUIntEn(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_INT_EN |= EC_REG_BANK_PD_MON_INT_EN_VTR1_PU_INTEN_Msk;
}

void EC_REG_BANK_VTR1PadMonPUIntDis(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_INT_EN &= ~EC_REG_BANK_PD_MON_INT_EN_VTR1_PU_INTEN_Msk;
}

void EC_REG_BANK_VTR1PadMonPDIntEn(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_INT_EN |= EC_REG_BANK_PD_MON_INT_EN_VTR1_PD_INTEN_Msk;
}

void EC_REG_BANK_VTR1PadMonPDIntDis(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_INT_EN &= ~EC_REG_BANK_PD_MON_INT_EN_VTR1_PD_INTEN_Msk;
}

void EC_REG_BANK_VTR2PadMonDebounceCtrl(VTR_PAD_MON_DEB_CTRL ctrl)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_CTRL = (ctrl << EC_REG_BANK_PD_MON_CTRL_CTRL_VTR2_Pos);
}

void EC_REG_BANK_VTR2PadMonOverrideEn(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_CTRL |= EC_REG_BANK_PD_MON_CTRL_OVRD_VTR2_Msk;
}

void EC_REG_BANK_VTR2PadMonOverrideDis(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_CTRL &= ~EC_REG_BANK_PD_MON_CTRL_OVRD_VTR2_Msk;
}

void EC_REG_BANK_VTR2PadMonOverrideInpDis(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_CTRL |= EC_REG_BANK_PD_MON_CTRL_VTR2_INPT_DIS_Msk;
}

void EC_REG_BANK_VTR2PadMonOverrideInpEn(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_CTRL &= ~EC_REG_BANK_PD_MON_CTRL_VTR2_INPT_DIS_Msk;
}

void EC_REG_BANK_VTR2PadMonOverrideProtEn(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_CTRL &= ~EC_REG_BANK_PD_MON_CTRL_VTR2_PROTECN_Msk;
}

void EC_REG_BANK_VTR2PadMonOverrideProtDis(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_CTRL |= EC_REG_BANK_PD_MON_CTRL_VTR2_PROTECN_Msk;
}

void EC_REG_BANK_VTR2PadMonPUIntEn(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_INT_EN |= EC_REG_BANK_PD_MON_INT_EN_VTR2_PU_INTEN_Msk;
}

void EC_REG_BANK_VTR2PadMonPUIntDis(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_INT_EN &= ~EC_REG_BANK_PD_MON_INT_EN_VTR2_PU_INTEN_Msk;
}

void EC_REG_BANK_VTR2PadMonPDIntEn(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_INT_EN |= EC_REG_BANK_PD_MON_INT_EN_VTR2_PD_INTEN_Msk;
}

void EC_REG_BANK_VTR2PadMonPDIntDis(void)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_INT_EN &= ~EC_REG_BANK_PD_MON_INT_EN_VTR2_PD_INTEN_Msk;
}

uint32_t EC_REG_BANK_PadMonStatusGet(void)
{
    return EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_STS & EC_REG_BANK_PD_MON_STS_Msk;
}

void EC_REG_BANK_PadMonStatusClr(uint32_t statusBitMask)
{
    EC_REG_BANK_REGS->EC_REG_BANK_PD_MON_STS = statusBitMask;
}


