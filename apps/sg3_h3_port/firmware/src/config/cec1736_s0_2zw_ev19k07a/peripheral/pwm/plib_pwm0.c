/*******************************************************************************
  PWM Timer Peripheral Library Interface Source File

  Company
    Microchip Technology Inc.

  File Name
    plib_pwm0.c

  Summary
    PWM0 peripheral library source file.

  Description
    This file implements the interface to the PWM Timer peripheral library.  This
    library provides access to and control of the associated peripheral
    instance.

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
#include "plib_pwm0.h"

void PWM0_Initialize(void)
{
    PWM0_REGS->PWM_CFG &= ~PWM_CFG_PWM_EN_Msk;

    PWM0_REGS->PWM_CNT_ON = 6250;

    PWM0_REGS->PWM_CNT_OFF = 6250;

    PWM0_REGS->PWM_CFG = PWM_CFG_CLK_SEL(1) | PWM_CFG_CLK_PRE_DIV(0) ;
}


void PWM0_Start(void)
{
    PWM0_REGS->PWM_CFG |= PWM_CFG_PWM_EN_Msk;
}

void PWM0_Stop (void)
{
    PWM0_REGS->PWM_CFG &= ~PWM_CFG_PWM_EN_Msk;
}

void PWM0_OnCountSet (uint16_t onCount)
{
    PWM0_REGS->PWM_CNT_ON = onCount;
}

void PWM0_OffCountSet (uint16_t offCount)
{
    PWM0_REGS->PWM_CNT_OFF = offCount;
}

void PWM0_ClkSelect (PWM_CLK_SEL pwmClk)
{
    PWM0_REGS->PWM_CFG = (PWM0_REGS->PWM_CFG & ~PWM_CFG_CLK_SEL_Msk) | (pwmClk << PWM_CFG_CLK_SEL_Pos);
}

void PWM0_ClkDividerSet (uint8_t divVal)
{
    PWM0_REGS->PWM_CFG = (PWM0_REGS->PWM_CFG & ~PWM_CFG_CLK_PRE_DIV_Msk) | (divVal << PWM_CFG_CLK_PRE_DIV_Pos);
}

void PWM0_OutputConfig (PWM_OUTPUT pwmOutput)
{
    PWM0_REGS->PWM_CFG = (PWM0_REGS->PWM_CFG & ~PWM_CFG_INV_Msk) | (pwmOutput << PWM_CFG_INV_Pos);
}

