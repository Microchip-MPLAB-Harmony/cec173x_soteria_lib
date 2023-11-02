/*******************************************************************************
  SPI Peripheral Target Peripheral Library Interface Source File

  Company
    Microchip Technology Inc.

  File Name
    plib_spt1.c

  Summary
    SPT1 peripheral library source file.

  Description
    This file implements the interface to the SPI Peripheral Target peripheral
    library. This library provides access to and control of the associated
    peripheral instance.

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
#include "plib_spt1.h"
#include "peripheral/ecia/plib_ecia.h"

static SPT_CALLBACK_OBJ SPT1_CallbackObject;

void SPT1_Initialize(void)
{

    SPT1_REGS->SPT_SPI_CFG = SPT_SPI_CFG_WAIT_TIME(4) | SPT_SPI_CFG_TAR_TIM_SEL(0x3) ;

    SPT1_REGS->SPT_SPI_IEN = 0x0;

    SPT1_REGS->SPT_EC_IEN = 0x4001;


    SPT1_REGS->SPT_SYS_CFG = 0x100f2;
}

void SPT1_WaitTimeSet(uint8_t wait_time)
{
    SPT1_REGS->SPT_SPI_CFG = (SPT1_REGS->SPT_SPI_CFG & ~SPT_SPI_CFG_WAIT_TIME_Msk) | SPT_SPI_CFG_WAIT_TIME(wait_time);
}

void SPT1_TARTimeSet(uint8_t tar_cycles)
{
    SPT1_REGS->SPT_SPI_CFG = (SPT1_REGS->SPT_SPI_CFG & ~SPT_SPI_CFG_TAR_TIM_SEL_Msk) | SPT_SPI_CFG_TAR_TIM_SEL(tar_cycles);
}

void SPT1_QuadModeEnable(void)
{
    SPT1_REGS->SPT_SPI_CFG |= SPT_SPI_CFG_SNG_QUD_SEL_Msk;
}

void SPT1_QuadModeDisable(void)
{
    SPT1_REGS->SPT_SPI_CFG &= ~SPT_SPI_CFG_SNG_QUD_SEL_Msk;
}

void SPT1_ECInterruptEnable(SPT_EC_INT int_en)
{
    SPT1_REGS->SPT_EC_IEN = (uint32_t)int_en;
}

void SPT1_CallbackRegister( SPT_CALLBACK callback, uintptr_t context )
{
    SPT1_CallbackObject.callback = callback;

    SPT1_CallbackObject.context = context;
}


uint32_t SPT1_ECStatusRegGet(void)
{
    return SPT1_REGS->SPT_SPI_EC_STS;
}

void SPT1_ECStatusRegClear(uint32_t bitmask)
{
    SPT1_REGS->SPT_SPI_EC_STS = bitmask;
}

void SPT1_MEM0Config(uint32_t bar, uint32_t wr_lim, uint32_t rd_lim)
{
    SPT1_REGS->SPT_MEM_BAR0 = bar;
    SPT1_REGS->SPT_MEM_WR_LIM0 = wr_lim;
    SPT1_REGS->SPT_MEM_RD_LIM0 = rd_lim;
    SPT1_REGS->SPT_MEM_CFG |= SPT_MEM_CFG_BAR_EN0_SEL_Msk;
}

void SPT1_MEM1Config(uint32_t bar, uint32_t wr_lim, uint32_t rd_lim)
{
    SPT1_REGS->SPT_MEM_BAR1 = bar;
    SPT1_REGS->SPT_MEM_WR_LIM1 = wr_lim;
    SPT1_REGS->SPT_MEM_RD_LIM1 = rd_lim;
    SPT1_REGS->SPT_MEM_CFG |= SPT_MEM_CFG_BAR_EN1_SEL_Msk;
}

void SPT1_MEM0Enable(void)
{
    SPT1_REGS->SPT_MEM_CFG |= SPT_MEM_CFG_BAR_EN0_SEL_Msk;
}

void SPT1_MEM1Enable(void)
{
    SPT1_REGS->SPT_MEM_CFG |= SPT_MEM_CFG_BAR_EN1_SEL_Msk;
}

void SPT1_MEM0Disable(void)
{
    SPT1_REGS->SPT_MEM_CFG &= ~SPT_MEM_CFG_BAR_EN0_SEL_Msk;
}

void SPT1_MEM1Disable(void)
{
    SPT1_REGS->SPT_MEM_CFG &= ~SPT_MEM_CFG_BAR_EN1_SEL_Msk;
}

uint32_t SPT1_RXFIFOByteCountGet(void)
{
    return SPT1_REGS->SPT_RXF_BYTE_CNT;
}

uint32_t SPT1_TXFIFOByteCountGet(void)
{
    return SPT1_REGS->SPT_TXF_BYTE_CNT;
}

uint32_t SPT1_RXFIFOBaseAddrGet(void)
{
    return SPT1_REGS->SPT_RXF_HOST_BAR;
}

uint32_t SPT1_TXFIFOBaseAddrGet(void)
{
    return SPT1_REGS->SPT_TXF_HOST_BAR;
}

void SPT1_Enable(void)
{
    SPT1_REGS->SPT_SYS_CFG |= SPT_SYS_CFG_SPI_SLV_EN_Msk;
}

void SPT1_Disable(void)
{
    SPT1_REGS->SPT_SYS_CFG &= ~SPT_SYS_CFG_SPI_SLV_EN_Msk;
}


uint32_t SPT1_HostToECMBXRead(void)
{
    return SPT1_REGS->SPT_SPIM2EC_MBX;
}

void SPT1_HostToECMBXClr(void)
{
    SPT1_REGS->SPT_SPIM2EC_MBX = 0xFFFFFFFFU;
}

void SPT1_ECToHostMBXWrite(uint32_t val)
{
    SPT1_REGS->SPT_EC2SPIM_MBX = val;
}

uint32_t SPT1_ECToHostMBXRead(void)
{
    return SPT1_REGS->SPT_EC2SPIM_MBX;
}


void SPT1_GRP_InterruptHandler(void)
{
    if (ECIA_GIRQResultGet(ECIA_AGG_INT_SRC_SPT1) != 0U)
    {
        uint32_t status = SPT1_REGS->SPT_SPI_EC_STS;
        
        ECIA_GIRQSourceClear(ECIA_AGG_INT_SRC_SPT1);

        if (SPT1_CallbackObject.callback != NULL)
        {
            SPT1_CallbackObject.callback(status, SPT1_CallbackObject.context);
        }
        
        /* Clear the status bits */
        SPT1_REGS->SPT_SPI_EC_STS = status;
    }
}


