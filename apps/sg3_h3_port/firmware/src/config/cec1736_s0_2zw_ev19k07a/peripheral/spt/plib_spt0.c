/*******************************************************************************
  SPI Peripheral Target Peripheral Library Interface Source File

  Company
    Microchip Technology Inc.

  File Name
    plib_spt0.c

  Summary
    SPT0 peripheral library source file.

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
#include "plib_spt0.h"
#include "peripheral/ecia/plib_ecia.h"

static SPT_CALLBACK_OBJ SPT0_CallbackObject;

void SPT0_Initialize(void)
{

    SPT0_REGS->SPT_SPI_CFG = SPT_SPI_CFG_WAIT_TIME(4) | SPT_SPI_CFG_TAR_TIM_SEL(0x3) ;

    SPT0_REGS->SPT_SPI_IEN = 0x0;

    SPT0_REGS->SPT_EC_IEN = 0x4001;


    SPT0_REGS->SPT_SYS_CFG = 0x100f2;
}

void SPT0_WaitTimeSet(uint8_t wait_time)
{
    SPT0_REGS->SPT_SPI_CFG = (SPT0_REGS->SPT_SPI_CFG & ~SPT_SPI_CFG_WAIT_TIME_Msk) | SPT_SPI_CFG_WAIT_TIME(wait_time);
}

void SPT0_TARTimeSet(uint8_t tar_cycles)
{
    SPT0_REGS->SPT_SPI_CFG = (SPT0_REGS->SPT_SPI_CFG & ~SPT_SPI_CFG_TAR_TIM_SEL_Msk) | SPT_SPI_CFG_TAR_TIM_SEL(tar_cycles);
}

void SPT0_QuadModeEnable(void)
{
    SPT0_REGS->SPT_SPI_CFG |= SPT_SPI_CFG_SNG_QUD_SEL_Msk;
}

void SPT0_QuadModeDisable(void)
{
    SPT0_REGS->SPT_SPI_CFG &= ~SPT_SPI_CFG_SNG_QUD_SEL_Msk;
}

void SPT0_ECInterruptEnable(SPT_EC_INT int_en)
{
    SPT0_REGS->SPT_EC_IEN = (uint32_t)int_en;
}

void SPT0_CallbackRegister( SPT_CALLBACK callback, uintptr_t context )
{
    SPT0_CallbackObject.callback = callback;

    SPT0_CallbackObject.context = context;
}


uint32_t SPT0_ECStatusRegGet(void)
{
    return SPT0_REGS->SPT_SPI_EC_STS;
}

void SPT0_ECStatusRegClear(uint32_t bitmask)
{
    SPT0_REGS->SPT_SPI_EC_STS = bitmask;
}

void SPT0_MEM0Config(uint32_t bar, uint32_t wr_lim, uint32_t rd_lim)
{
    SPT0_REGS->SPT_MEM_BAR0 = bar;
    SPT0_REGS->SPT_MEM_WR_LIM0 = wr_lim;
    SPT0_REGS->SPT_MEM_RD_LIM0 = rd_lim;
    SPT0_REGS->SPT_MEM_CFG |= SPT_MEM_CFG_BAR_EN0_SEL_Msk;
}

void SPT0_MEM1Config(uint32_t bar, uint32_t wr_lim, uint32_t rd_lim)
{
    SPT0_REGS->SPT_MEM_BAR1 = bar;
    SPT0_REGS->SPT_MEM_WR_LIM1 = wr_lim;
    SPT0_REGS->SPT_MEM_RD_LIM1 = rd_lim;
    SPT0_REGS->SPT_MEM_CFG |= SPT_MEM_CFG_BAR_EN1_SEL_Msk;
}

void SPT0_MEM0Enable(void)
{
    SPT0_REGS->SPT_MEM_CFG |= SPT_MEM_CFG_BAR_EN0_SEL_Msk;
}

void SPT0_MEM1Enable(void)
{
    SPT0_REGS->SPT_MEM_CFG |= SPT_MEM_CFG_BAR_EN1_SEL_Msk;
}

void SPT0_MEM0Disable(void)
{
    SPT0_REGS->SPT_MEM_CFG &= ~SPT_MEM_CFG_BAR_EN0_SEL_Msk;
}

void SPT0_MEM1Disable(void)
{
    SPT0_REGS->SPT_MEM_CFG &= ~SPT_MEM_CFG_BAR_EN1_SEL_Msk;
}

uint32_t SPT0_RXFIFOByteCountGet(void)
{
    return SPT0_REGS->SPT_RXF_BYTE_CNT;
}

uint32_t SPT0_TXFIFOByteCountGet(void)
{
    return SPT0_REGS->SPT_TXF_BYTE_CNT;
}

uint32_t SPT0_RXFIFOBaseAddrGet(void)
{
    return SPT0_REGS->SPT_RXF_HOST_BAR;
}

uint32_t SPT0_TXFIFOBaseAddrGet(void)
{
    return SPT0_REGS->SPT_TXF_HOST_BAR;
}

void SPT0_Enable(void)
{
    SPT0_REGS->SPT_SYS_CFG |= SPT_SYS_CFG_SPI_SLV_EN_Msk;
}

void SPT0_Disable(void)
{
    SPT0_REGS->SPT_SYS_CFG &= ~SPT_SYS_CFG_SPI_SLV_EN_Msk;
}


uint32_t SPT0_HostToECMBXRead(void)
{
    return SPT0_REGS->SPT_SPIM2EC_MBX;
}

void SPT0_HostToECMBXClr(void)
{
    SPT0_REGS->SPT_SPIM2EC_MBX = 0xFFFFFFFFU;
}

void SPT0_ECToHostMBXWrite(uint32_t val)
{
    SPT0_REGS->SPT_EC2SPIM_MBX = val;
}

uint32_t SPT0_ECToHostMBXRead(void)
{
    return SPT0_REGS->SPT_EC2SPIM_MBX;
}


void SPT0_GRP_InterruptHandler(void)
{
    if (ECIA_GIRQResultGet(ECIA_AGG_INT_SRC_SPT0) != 0U)
    {
        uint32_t status = SPT0_REGS->SPT_SPI_EC_STS;
        
        ECIA_GIRQSourceClear(ECIA_AGG_INT_SRC_SPT0);

        if (SPT0_CallbackObject.callback != NULL)
        {
            SPT0_CallbackObject.callback(status, SPT0_CallbackObject.context);
        }
        
        /* Clear the status bits */
        SPT0_REGS->SPT_SPI_EC_STS = status;
    }
}


