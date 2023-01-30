/*******************************************************************************
  RTOS Timer Peripheral Library Interface Source File

  Company
    Microchip Technology Inc.

  File Name
    plib_cct.c

  Summary
    CCT peripheral library source file.

  Description
    This file implements the interface to the RTOS Timer peripheral library.  This
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
#include "plib_cct.h"
#include "peripheral/ecia/plib_ecia.h"





void CCT_Initialize(void)
{
    CCT_REGS->CCT_CTRL |= CCT_CTRL_FREE_RST_Msk;

    while (CCT_REGS->CCT_CTRL & CCT_CTRL_FREE_RST_Msk);

    CCT_REGS->CCT_CTRL = CCT_CTRL_ACT_Msk | CCT_CTRL_CMP_EN0_Msk | CCT_CTRL_TCLK(0x4);



    
    CCT_REGS->CCT_COMP0 = 32768;
    
}

/* Brings timer block in running state */
void CCT_TimerActivate( void )
{
    CCT_REGS->CCT_CTRL |= CCT_CTRL_ACT_Msk;
}

/* Power down timer block and all clocks are gated */
void CCT_TimerDeActivate( void )
{
    CCT_REGS->CCT_CTRL &= ~CCT_CTRL_ACT_Msk;
}

/* Start the FRT counter */
void CCT_FreeRunningTimerStart( void )
{
    CCT_REGS->CCT_CTRL |= CCT_CTRL_FREE_EN_Msk;
}

/* Stop but do not reset the FRT counter */
void CCT_FreeRunningTimerStop( void )
{
    CCT_REGS->CCT_CTRL &= ~CCT_CTRL_FREE_EN_Msk;
}

/* Stop and reset the FRT counter to 0 */
void CCT_FreeRunningTimerReset( void )
{
    CCT_REGS->CCT_CTRL |= CCT_CTRL_FREE_RST_Msk;

    while (CCT_REGS->CCT_CTRL & CCT_CTRL_FREE_RST_Msk);
}

uint32_t CCT_FreeRunningTimerGet( void )
{
    return CCT_REGS->CCT_FREE_RUN;
}

void CCT_FreeRunningTimerSet( uint32_t count)
{
    /* Save current value of FREE_EN bit before stoping the timer */
    uint32_t cct_ctrl = CCT_REGS->CCT_CTRL & CCT_CTRL_FREE_EN_Msk;

    /* Stop the FRT before updating the count */
    CCT_REGS->CCT_CTRL &= ~CCT_CTRL_FREE_EN_Msk;

    CCT_REGS->CCT_FREE_RUN = count;

    /* Re-start the FRT */
    CCT_REGS->CCT_CTRL |= cct_ctrl;
}

void CCT_FreqDivSet( uint32_t div )
{
    CCT_REGS->CCT_CTRL = (CCT_REGS->CCT_CTRL & ~CCT_CTRL_TCLK_Msk) | (div << CCT_CTRL_TCLK_Pos);
}

uint32_t CCT_FrequencyGet(void)
{
    uint32_t freq_div = (CCT_REGS->CCT_CTRL & CCT_CTRL_TCLK_Msk) >> CCT_CTRL_TCLK_Pos;
    uint32_t freq = 48000000/(freq_div + 1);
    return freq;
}




uint32_t CCT_CompareChannel0PeriodGet( void )
{
    return CCT_REGS->CCT_COMP0;
}

void CCT_CompareChannel0PeriodSet( uint32_t period )
{
    CCT_REGS->CCT_COMP0 = period;
}

void CCT_CompareChannel0Enable( void )
{
    CCT_REGS->CCT_CTRL |= CCT_CTRL_CMP_EN0_Msk;
}

void CCT_CompareChannel0Disable( void )
{
    CCT_REGS->CCT_CTRL &= ~CCT_CTRL_CMP_EN0_Msk;
}

void CCT_CompareChannel0OutputSet( void )
{
    CCT_REGS->CCT_CTRL = (CCT_REGS->CCT_CTRL & ~(CCT_CTRL_CMP_CLR0_Msk)) | CCT_CTRL_CMP_SET0_Msk;
}

void CCT_CompareChannel0OutputClear( void )
{
    CCT_REGS->CCT_CTRL = (CCT_REGS->CCT_CTRL & ~(CCT_CTRL_CMP_SET0_Msk)) | CCT_CTRL_CMP_CLR0_Msk;
}

void CCT_CompareChannel0InterruptEnable( void )
{
    ECIA_GIRQSourceEnable(ECIA_AGG_INT_SRC_CCT_CMP0);
}

void CCT_CompareChannel0InterruptDisable( void )
{
    ECIA_GIRQSourceEnable(ECIA_AGG_INT_SRC_CCT_CMP0);
}





