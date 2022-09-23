/*******************************************************************************
 PCR PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_pcr.c

  Summary:
    PCR PLIB Implementation File.

  Description:
    None

*******************************************************************************/

/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
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

#include "plib_pcr.h"
#include "device.h"

#define PCR_PERIPH_RESET_LOCK                     (0xA6382D4DUL)
#define PCR_PERIPH_RESET_UNLOCK                   (0xA6382D4CUL)

void PCR_Initialize (void)
{
    /* Wait for PLL to lock. Output of PLL is 96 MHz. */
    while ((PCR_REGS->PCR_OSC_ID & PCR_OSC_ID_PLL_LOCK_Msk) == 0U)
    {
        /* Wait for PLL to lock */
    }

    /* Configure Processor Clock divide */
    PCR_REGS->PCR_PROC_CLK_CTRL = PCR_PROC_CLK_CTRL_DIV(0x1);

    /* Select slow clock divide */
    PCR_REGS->PCR_SLOW_CLK_CTRL = PCR_SLOW_CLK_CTRL_DIV(480);

    PCR_REGS->PCR_SLP_EN_0 = 0x0;
    PCR_REGS->PCR_SLP_EN_1 = 0x0;
    PCR_REGS->PCR_SLP_EN_3 = 0x0;
    PCR_REGS->PCR_SLP_EN_4 = 0x0;

    PCR_REGS->PCR_EC_PRIV_EN0 = 0x0;
    PCR_REGS->PCR_EC_PRIV_EN1 = 0x0;
    PCR_REGS->PCR_EC_PRIV_EN3 = 0x0;
    PCR_REGS->PCR_EC_PRIV_EN4 = 0x0;
}

void PCR_PeripheralResetLock (void)
{
    PCR_REGS->PCR_PERIPH_RST_EN_LOCK = PCR_PERIPH_RESET_LOCK;
}

void PCR_PeripheralResetUnLock (void)
{
    PCR_REGS->PCR_PERIPH_RST_EN_LOCK = PCR_PERIPH_RESET_UNLOCK;
}

void PCR_PrivilegeEnLock (void)
{
    PCR_REGS->PCR_PRIV_EN_LOCK |= PCR_PRIV_EN_LOCK_LOCK_EN_Msk;
}

void PCR_PrivilegeEnUnLock (void)
{
    PCR_REGS->PCR_PRIV_EN_LOCK &= ~PCR_PRIV_EN_LOCK_LOCK_EN_Msk;
}



void PCR_SleepEnable0 (PCR_SLEEP_EN0 blockId)
{
    uint32_t blockIdMsk = (uint32_t)blockId & PCR_SLP_EN_0_Msk;
    PCR_REGS->PCR_SLP_EN_0 |= blockIdMsk;
}

void PCR_SleepDisable0 (PCR_SLEEP_EN0 blockId)
{
    uint32_t blockIdMsk = (uint32_t)blockId & PCR_SLP_EN_0_Msk;
    PCR_REGS->PCR_SLP_EN_0 &= ~blockIdMsk;
}


void PCR_SleepEnable1 (PCR_SLEEP_EN1 blockId)
{
    uint32_t blockIdMsk = (uint32_t)blockId & PCR_SLP_EN_1_Msk;
    PCR_REGS->PCR_SLP_EN_1 |= blockIdMsk;
}

void PCR_SleepDisable1 (PCR_SLEEP_EN1 blockId)
{
    uint32_t blockIdMsk = (uint32_t)blockId & PCR_SLP_EN_1_Msk;
    PCR_REGS->PCR_SLP_EN_1 &= ~blockIdMsk;
}


void PCR_SleepEnable3 (PCR_SLEEP_EN3 blockId)
{
    uint32_t blockIdMsk = (uint32_t)blockId & PCR_SLP_EN_3_Msk;
    PCR_REGS->PCR_SLP_EN_3 |= blockIdMsk;
}

void PCR_SleepDisable3 (PCR_SLEEP_EN3 blockId)
{
    uint32_t blockIdMsk = (uint32_t)blockId & PCR_SLP_EN_3_Msk;
    PCR_REGS->PCR_SLP_EN_3 &= ~blockIdMsk;
}


void PCR_SleepEnable4 (PCR_SLEEP_EN4 blockId)
{
    uint32_t blockIdMsk = (uint32_t)blockId & PCR_SLP_EN_4_Msk;
    PCR_REGS->PCR_SLP_EN_4 |= blockIdMsk;
}

void PCR_SleepDisable4 (PCR_SLEEP_EN4 blockId)
{
    uint32_t blockIdMsk = (uint32_t)blockId & PCR_SLP_EN_4_Msk;
    PCR_REGS->PCR_SLP_EN_4 &= ~blockIdMsk;
}



void PCR_PrivilegeEnable0 (PCR_PRIV_EN0 blockId)
{
    uint32_t blockIdMsk = (uint32_t)blockId & PCR_EC_PRIV_EN0_Msk;
    PCR_REGS->PCR_EC_PRIV_EN0 |= blockIdMsk;
}

void PCR_PrivilegeDisable0 (PCR_PRIV_EN0 blockId)
{
    uint32_t blockIdMsk = (uint32_t)blockId & PCR_EC_PRIV_EN0_Msk;
    PCR_REGS->PCR_EC_PRIV_EN0 &= ~blockIdMsk;
}


void PCR_PrivilegeEnable1 (PCR_PRIV_EN1 blockId)
{
    uint32_t blockIdMsk = (uint32_t)blockId & PCR_EC_PRIV_EN1_Msk;
    PCR_REGS->PCR_EC_PRIV_EN1 |= blockIdMsk;
}

void PCR_PrivilegeDisable1 (PCR_PRIV_EN1 blockId)
{
    uint32_t blockIdMsk = (uint32_t)blockId & PCR_EC_PRIV_EN1_Msk;
    PCR_REGS->PCR_EC_PRIV_EN1 &= ~blockIdMsk;
}


void PCR_PrivilegeEnable3 (PCR_PRIV_EN3 blockId)
{
    uint32_t blockIdMsk = (uint32_t)blockId & PCR_EC_PRIV_EN3_Msk;
    PCR_REGS->PCR_EC_PRIV_EN3 |= blockIdMsk;
}

void PCR_PrivilegeDisable3 (PCR_PRIV_EN3 blockId)
{
    uint32_t blockIdMsk = (uint32_t)blockId & PCR_EC_PRIV_EN3_Msk;
    PCR_REGS->PCR_EC_PRIV_EN3 &= ~blockIdMsk;
}


void PCR_PrivilegeEnable4 (PCR_PRIV_EN4 blockId)
{
    uint32_t blockIdMsk = (uint32_t)blockId & PCR_EC_PRIV_EN4_Msk;
    PCR_REGS->PCR_EC_PRIV_EN4 |= blockIdMsk;
}

void PCR_PrivilegeDisable4 (PCR_PRIV_EN4 blockId)
{
    uint32_t blockIdMsk = (uint32_t)blockId & PCR_EC_PRIV_EN4_Msk;
    PCR_REGS->PCR_EC_PRIV_EN4 &= ~blockIdMsk;
}



void PCR_ResetEnable0 (PCR_RESET_EN0 blockId)
{
    uint32_t blockIdMsk = (uint32_t)blockId & PCR_RST_EN_0_Msk;
    PCR_REGS->PCR_RST_EN_0 |= blockIdMsk;
}


void PCR_ResetEnable1 (PCR_RESET_EN1 blockId)
{
    uint32_t blockIdMsk = (uint32_t)blockId & PCR_RST_EN_1_Msk;
    PCR_REGS->PCR_RST_EN_1 |= blockIdMsk;
}


void PCR_ResetEnable3 (PCR_RESET_EN3 blockId)
{
    uint32_t blockIdMsk = (uint32_t)blockId & PCR_RST_EN_3_Msk;
    PCR_REGS->PCR_RST_EN_3 |= blockIdMsk;
}


void PCR_ResetEnable4 (PCR_RESET_EN4 blockId)
{
    uint32_t blockIdMsk = (uint32_t)blockId & PCR_RST_EN_4_Msk;
    PCR_REGS->PCR_RST_EN_4 |= blockIdMsk;
}

