/*******************************************************************************
  WDT Peripheral Library

  Company:
    Microchip Technology Inc.

  File Name:
    plib_wdt.c

  Summary:
    WDT Source File

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
#include "plib_wdt.h"

// *****************************************************************************
// *****************************************************************************
// Section: WDT Implementation
// *****************************************************************************
// *****************************************************************************
static WDT_OBJECT wdt;





void WDT_Initialize( void )
{
    WDT_REGS->WDT_LOAD = 500;

    WDT_REGS->WDT_CTRL = WDT_CTRL_JTAG_STL_Msk | WDT_CTRL_WDT_RST_Msk;

    WDT_REGS->WDT_IEN = WDT_IEN_Msk;

}
void WDT_Enable(void)
{
    WDT_REGS->WDT_CTRL |= WDT_CTRL_WDT_EN_Msk;
}

void WDT_Disable(void)
{
    WDT_REGS->WDT_CTRL &= ~WDT_CTRL_WDT_EN_Msk;
}

bool WDT_IsEnabled( void )
{
    return ((bool)WDT_REGS->WDT_CTRL & WDT_CTRL_WDT_EN_Msk);
}

void WDT_Clear(void)
{
    /* Clear WDT timer */
    WDT_REGS->WDT_KICK = 0;
}

uint16_t WDT_CountGet(void)
{
    /* Return WDT timer counter */
    return WDT_REGS->WDT_CNT;
}

bool WDT_isPowerFailWDTEventSet(void)
{
    if ((VTR_REG_BANK_REGS->VTR_REG_BANK_PFRS & VTR_REG_BANK_PFRS_WDT_EVT_Msk) != 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void WDT_PowerFailWDTEventClear(void)
{
    if (VTR_REG_BANK_REGS->VTR_REG_BANK_PFRS & VTR_REG_BANK_PFRS_WDT_EVT_Msk)
    {
        /* Write 1 to clear this bit */
        VTR_REG_BANK_REGS->VTR_REG_BANK_PFRS |= VTR_REG_BANK_PFRS_WDT_EVT_Msk;
    }
}

void WDT_PeriodLoad(uint16_t period)
{
    WDT_REGS->WDT_LOAD = period;
}


void WDT_TimeoutActionSet(WDT_TIMEOUT_ACTION action)
{
    WDT_REGS->WDT_CTRL = (WDT_REGS->WDT_CTRL & ~WDT_CTRL_WDT_RST_Msk) | (action << WDT_CTRL_WDT_RST_Pos);
}

void WDT_CallbackRegister( WDT_CALLBACK callback, uintptr_t context )
{
   wdt.callback = callback;
   wdt.context = context;
}

void WDT_GRP_InterruptHandler( void )
{
    if (ECIA_GIRQResultGet(ECIA_AGG_INT_SRC_WDT))
    {
        WDT_REGS->WDT_STS = WDT_STS_WDT_EV_IRQ_Msk;

        ECIA_GIRQSourceClear(ECIA_AGG_INT_SRC_WDT);

        if(wdt.callback != NULL)
        {
            wdt.callback(wdt.context);
        }
    }
}
