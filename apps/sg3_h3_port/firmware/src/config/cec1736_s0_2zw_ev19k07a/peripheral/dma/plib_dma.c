/*******************************************************************************
  Direct Memory Access Controller (DMA) PLIB

  Company
    Microchip Technology Inc.

  File Name
    plib_dma.c

  Summary
    Source for DMA peripheral library interface Implementation.

  Description
    This file defines the interface to the DMA peripheral library. This
    library provides access to and control of the DMA controller.

  Remarks:
    None.

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

#include "plib_dma.h"
#include "interrupts.h"

#define NOP()    asm("NOP")

static dma_chan00_registers_t* DMA_ChannelBaseAddrGet(DMA_CHANNEL ch)
{
    return (dma_chan00_registers_t*)(DMA_CHAN00_BASE_ADDRESS + ((uint32_t)ch * 64U));
}

void DMA_Initialize( void )
{
    dma_chan00_registers_t* dmaChRegs;


    /* Reset DMA module */
    DMA_MAIN_REGS->DMA_MAIN_ACTRST = DMA_MAIN_ACTRST_SOFT_RST_Msk;

   /***************** Configure DMA channel 0 ********************/
   dmaChRegs = DMA_ChannelBaseAddrGet(DMA_CHANNEL_0);

   dmaChRegs->DMA_CHAN00_CTRL = DMA_CHAN00_CTRL_TX_DIR(0x0) | DMA_CHAN00_CTRL_DIS_HW_FLOW_CTRL(0) | DMA_CHAN00_CTRL_HW_FLOW_CTRL_DEV(1) | DMA_CHAN00_CTRL_TRANS_SIZE(0x1) | DMA_CHAN00_CTRL_INC_DEV_ADDR(false) | DMA_CHAN00_CTRL_INC_MEM_ADDR(true);
   dmaChRegs->DMA_CHAN00_ACTIVATE = DMA_CHAN00_ACTIVATE_CHN_Msk;

   /***************** Configure DMA channel 1 ********************/
   dmaChRegs = DMA_ChannelBaseAddrGet(DMA_CHANNEL_1);

   dmaChRegs->DMA_CHAN00_CTRL = DMA_CHAN00_CTRL_TX_DIR(0x0) | DMA_CHAN00_CTRL_DIS_HW_FLOW_CTRL(0) | DMA_CHAN00_CTRL_HW_FLOW_CTRL_DEV(0) | DMA_CHAN00_CTRL_TRANS_SIZE(0x1) | DMA_CHAN00_CTRL_INC_DEV_ADDR(false) | DMA_CHAN00_CTRL_INC_MEM_ADDR(true);
   dmaChRegs->DMA_CHAN00_ACTIVATE = DMA_CHAN00_ACTIVATE_CHN_Msk;

   /***************** Configure DMA channel 2 ********************/
   dmaChRegs = DMA_ChannelBaseAddrGet(DMA_CHANNEL_2);

   dmaChRegs->DMA_CHAN00_CTRL = DMA_CHAN00_CTRL_TX_DIR(0x0) | DMA_CHAN00_CTRL_DIS_HW_FLOW_CTRL(0) | DMA_CHAN00_CTRL_HW_FLOW_CTRL_DEV(3) | DMA_CHAN00_CTRL_TRANS_SIZE(0x1) | DMA_CHAN00_CTRL_INC_DEV_ADDR(false) | DMA_CHAN00_CTRL_INC_MEM_ADDR(true);
   dmaChRegs->DMA_CHAN00_ACTIVATE = DMA_CHAN00_ACTIVATE_CHN_Msk;

   /***************** Configure DMA channel 3 ********************/
   dmaChRegs = DMA_ChannelBaseAddrGet(DMA_CHANNEL_3);

   dmaChRegs->DMA_CHAN00_CTRL = DMA_CHAN00_CTRL_TX_DIR(0x0) | DMA_CHAN00_CTRL_DIS_HW_FLOW_CTRL(0) | DMA_CHAN00_CTRL_HW_FLOW_CTRL_DEV(2) | DMA_CHAN00_CTRL_TRANS_SIZE(0x1) | DMA_CHAN00_CTRL_INC_DEV_ADDR(false) | DMA_CHAN00_CTRL_INC_MEM_ADDR(true);
   dmaChRegs->DMA_CHAN00_ACTIVATE = DMA_CHAN00_ACTIVATE_CHN_Msk;

   /***************** Configure DMA channel 4 ********************/
   dmaChRegs = DMA_ChannelBaseAddrGet(DMA_CHANNEL_4);

   dmaChRegs->DMA_CHAN00_CTRL = DMA_CHAN00_CTRL_TX_DIR(0x0) | DMA_CHAN00_CTRL_DIS_HW_FLOW_CTRL(0) | DMA_CHAN00_CTRL_HW_FLOW_CTRL_DEV(5) | DMA_CHAN00_CTRL_TRANS_SIZE(0x1) | DMA_CHAN00_CTRL_INC_DEV_ADDR(false) | DMA_CHAN00_CTRL_INC_MEM_ADDR(true);
   dmaChRegs->DMA_CHAN00_ACTIVATE = DMA_CHAN00_ACTIVATE_CHN_Msk;

   /***************** Configure DMA channel 5 ********************/
   dmaChRegs = DMA_ChannelBaseAddrGet(DMA_CHANNEL_5);

   dmaChRegs->DMA_CHAN00_CTRL = DMA_CHAN00_CTRL_TX_DIR(0x0) | DMA_CHAN00_CTRL_DIS_HW_FLOW_CTRL(0) | DMA_CHAN00_CTRL_HW_FLOW_CTRL_DEV(4) | DMA_CHAN00_CTRL_TRANS_SIZE(0x1) | DMA_CHAN00_CTRL_INC_DEV_ADDR(false) | DMA_CHAN00_CTRL_INC_MEM_ADDR(true);
   dmaChRegs->DMA_CHAN00_ACTIVATE = DMA_CHAN00_ACTIVATE_CHN_Msk;

   /***************** Configure DMA channel 6 ********************/
   dmaChRegs = DMA_ChannelBaseAddrGet(DMA_CHANNEL_6);

   dmaChRegs->DMA_CHAN00_CTRL = DMA_CHAN00_CTRL_TX_DIR(0x0) | DMA_CHAN00_CTRL_DIS_HW_FLOW_CTRL(0) | DMA_CHAN00_CTRL_HW_FLOW_CTRL_DEV(7) | DMA_CHAN00_CTRL_TRANS_SIZE(0x1) | DMA_CHAN00_CTRL_INC_DEV_ADDR(false) | DMA_CHAN00_CTRL_INC_MEM_ADDR(true);
   dmaChRegs->DMA_CHAN00_ACTIVATE = DMA_CHAN00_ACTIVATE_CHN_Msk;

   /***************** Configure DMA channel 7 ********************/
   dmaChRegs = DMA_ChannelBaseAddrGet(DMA_CHANNEL_7);

   dmaChRegs->DMA_CHAN00_CTRL = DMA_CHAN00_CTRL_TX_DIR(0x0) | DMA_CHAN00_CTRL_DIS_HW_FLOW_CTRL(0) | DMA_CHAN00_CTRL_HW_FLOW_CTRL_DEV(6) | DMA_CHAN00_CTRL_TRANS_SIZE(0x1) | DMA_CHAN00_CTRL_INC_DEV_ADDR(false) | DMA_CHAN00_CTRL_INC_MEM_ADDR(true);
   dmaChRegs->DMA_CHAN00_ACTIVATE = DMA_CHAN00_ACTIVATE_CHN_Msk;

   /***************** Configure DMA channel 8 ********************/
   dmaChRegs = DMA_ChannelBaseAddrGet(DMA_CHANNEL_8);

   dmaChRegs->DMA_CHAN00_CTRL = DMA_CHAN00_CTRL_TX_DIR(0x0) | DMA_CHAN00_CTRL_DIS_HW_FLOW_CTRL(0) | DMA_CHAN00_CTRL_HW_FLOW_CTRL_DEV(9) | DMA_CHAN00_CTRL_TRANS_SIZE(0x1) | DMA_CHAN00_CTRL_INC_DEV_ADDR(false) | DMA_CHAN00_CTRL_INC_MEM_ADDR(true);
   dmaChRegs->DMA_CHAN00_ACTIVATE = DMA_CHAN00_ACTIVATE_CHN_Msk;

   /***************** Configure DMA channel 9 ********************/
   dmaChRegs = DMA_ChannelBaseAddrGet(DMA_CHANNEL_9);

   dmaChRegs->DMA_CHAN00_CTRL = DMA_CHAN00_CTRL_TX_DIR(0x0) | DMA_CHAN00_CTRL_DIS_HW_FLOW_CTRL(0) | DMA_CHAN00_CTRL_HW_FLOW_CTRL_DEV(8) | DMA_CHAN00_CTRL_TRANS_SIZE(0x1) | DMA_CHAN00_CTRL_INC_DEV_ADDR(false) | DMA_CHAN00_CTRL_INC_MEM_ADDR(true);
   dmaChRegs->DMA_CHAN00_ACTIVATE = DMA_CHAN00_ACTIVATE_CHN_Msk;


    /* Global enable */
    DMA_MAIN_REGS->DMA_MAIN_ACTRST = DMA_MAIN_ACTRST_ACT_Msk;
}


void DMA_MemIncrCfg(DMA_CHANNEL channel, uint8_t mem_incr_sts)
{
    dma_chan00_registers_t* dmaChRegs = DMA_ChannelBaseAddrGet(channel);

    dmaChRegs->DMA_CHAN00_CTRL = DMA_CHAN00_CTRL_INC_MEM_ADDR((bool)mem_incr_sts);
}

void DMA_ChannelAbort(DMA_CHANNEL channel)
{
    dma_chan00_registers_t* dmaChRegs = DMA_ChannelBaseAddrGet(channel);

    dmaChRegs->DMA_CHAN00_CTRL |= DMA_CHAN00_CTRL_TRANS_ABORT_Msk;

    NOP();NOP();NOP();NOP();NOP();NOP();

    dmaChRegs->DMA_CHAN00_CTRL &= ~DMA_CHAN00_CTRL_TRANS_ABORT_Msk;
}

void DMA_Activate(uint8_t activate)
{
    DMA_MAIN_REGS->DMA_MAIN_ACTRST |= activate;
}

void DMA_SoftReset(void)
{
    /* soft reset DMA */
    DMA_MAIN_REGS->DMA_MAIN_ACTRST |= DMA_MAIN_ACTRST_SOFT_RST_Msk;
    DMA_MAIN_REGS->DMA_MAIN_ACTRST |= DMA_MAIN_ACTRST_ACT_Msk;
}

void DMA_Enable(void)
{
    DMA_MAIN_REGS->DMA_MAIN_ACTRST |= DMA_MAIN_ACTRST_ACT_Msk;
}

void DMA_Disable(void)
{
    DMA_MAIN_REGS->DMA_MAIN_ACTRST &= ~DMA_MAIN_ACTRST_ACT_Msk;
}

void DMA_ChannelActivate(DMA_CHANNEL channel)
{
    dma_chan00_registers_t* dmaChRegs = DMA_ChannelBaseAddrGet(channel);
    dmaChRegs->DMA_CHAN00_ACTIVATE |= DMA_CHAN00_ACTIVATE_CHN_Msk;
}/* HW_DMA::activate */

/******************************************************************************/
/** HW_DMA::deactivate function.
 * This function clears the ACTIVATE bit
 * @param None
 * @return None
*******************************************************************************/
void DMA_ChannelDeactivate(DMA_CHANNEL channel)
{
    dma_chan00_registers_t* dmaChRegs = DMA_ChannelBaseAddrGet(channel);
    dmaChRegs->DMA_CHAN00_ACTIVATE &= ~DMA_CHAN00_ACTIVATE_CHN_Msk;
}/* HW_DMA::deactivate */

/******************************************************************************/
/** HW_DMA::stop function.
 * This function clears the RUN bit
 * @param None
 * @return None
*******************************************************************************/
void DMA_Stop(DMA_CHANNEL channel)
{
    dma_chan00_registers_t* dmaChRegs = DMA_ChannelBaseAddrGet(channel);
    dmaChRegs->DMA_CHAN00_CTRL &= ~DMA_CHAN00_CTRL_RUN_Msk;
}/* HW_DMA::stop */

/******************************************************************************/
/** HW_DMA::start function.
 * This function sets the RUN bit
 * @param None
 * @return None
*******************************************************************************/
void DMA_Start(DMA_CHANNEL channel)
{
    dma_chan00_registers_t* dmaChRegs = DMA_ChannelBaseAddrGet(channel);
    dmaChRegs->DMA_CHAN00_CTRL |= DMA_CHAN00_CTRL_RUN_Msk;
}/* HW_DMA::start */

uint8_t DMA_WaitTillFree(DMA_CHANNEL channel)
{
    uint32_t timeoutCounter=0;
    uint8_t retVal = 1;
    dma_chan00_registers_t* dmaChRegs = DMA_ChannelBaseAddrGet(channel);

    while((dmaChRegs->DMA_CHAN00_CTRL & DMA_CHAN00_CTRL_BUSY_Msk) != 0U)
    {
        if (timeoutCounter++ >= BUSY_TIMEOUT_COUNTER)
        {
            retVal = 0;
            break;
        }
    }
    return retVal;
}/* HW_DMA::wait_till_free */

void DMA_SetupTx(DMA_CHANNEL channel, const uint8_t device, const uint32_t dataAHBAddress, uint32_t dma_buffer_tx, const uint8_t transfer_bytes_count)
{
    /* Set the activate bit for the channel to operate */
    /* Activate bit is used to activate the dma device for the particular channel.
       It is relevant when setup_tx is called for the first time and
       the channel is not activated */

    dma_chan00_registers_t* dmaChRegs = DMA_ChannelBaseAddrGet(channel);

    DMA_ChannelActivate(channel);

    DMA_MAIN_REGS->DMA_MAIN_ACTRST = DMA_MAIN_ACTRST_ACT_Msk;

    /* Stop the Tx DMA before configuring it*/
    DMA_Stop(channel);

    if (0U == DMA_WaitTillFree(channel))
    {
        /* This should never happen, the trace here
         * will help to detect this condition during testing */
    }

    dmaChRegs->DMA_CHAN00_DSTART  = dataAHBAddress;

    /* Set DMI Start Address Register to beginning of buffer */
    dmaChRegs->DMA_CHAN00_MSTART = ((uint32_t)dma_buffer_tx);

    /* Set DMI End Address Register */
    dmaChRegs->DMA_CHAN00_MEND = ((uint32_t)(dma_buffer_tx + transfer_bytes_count));

    /* select device,direction */
    dmaChRegs->DMA_CHAN00_CTRL = (uint32_t)(((uint32_t)device<<DMA_CHAN00_CTRL_HW_FLOW_CTRL_DEV_Pos) | DMA_CHAN00_CTRL_TX_DIR_Msk
                                     | DMA_CHAN00_CTRL_INC_MEM_ADDR_Msk | DMA_CHAN00_CTRL_TRANS_SIZE(1));
}
void DMA_SetupRx(DMA_CHANNEL channel, const uint8_t device, const uint32_t dataAHBAddress, uint32_t dma_buffer_rx, const uint8_t transfer_bytes_count, const bool incrMemAddrFlag)
{
    uint32_t control_reg_value;
    dma_chan00_registers_t* dmaChRegs = DMA_ChannelBaseAddrGet(channel);

    /* Set the activate bit for the channel to operate */
    DMA_ChannelActivate(channel);

    /* Stop the Rx DMA before configuring it*/
    DMA_Stop(channel);

    if (0U == DMA_WaitTillFree(channel))
    {
        /* This should never happen, the trace here
         * will help to detect this condition during testing */
    }

    /* Assign DMA AHB Address */
    dmaChRegs->DMA_CHAN00_DSTART  = dataAHBAddress;

    /* Set DMI Start Address Register to beginning of receive buffer */
    dmaChRegs->DMA_CHAN00_MSTART = ((uint32_t)dma_buffer_rx);

    /* Set DMI End Address Register */
    dmaChRegs->DMA_CHAN00_MEND = ((uint32_t)(dma_buffer_rx + transfer_bytes_count));

    control_reg_value = (uint32_t)(((uint32_t)device<<DMA_CHAN00_CTRL_HW_FLOW_CTRL_DEV_Pos) | DMA_CHAN00_CTRL_TRANS_SIZE(1));

    if (incrMemAddrFlag)
    {
        control_reg_value |= DMA_CHAN00_CTRL_INC_MEM_ADDR_Msk;
    }

    /* select device,direction */
    dmaChRegs->DMA_CHAN00_CTRL = control_reg_value;
}

void DMA_SwitchTxToRx(DMA_CHANNEL channel, const uint8_t device, const uint32_t dataAHBAddress)
{
    dma_chan00_registers_t* dmaChRegs = DMA_ChannelBaseAddrGet(channel);

    /* Stop the Rx DMA before configuring it*/
    DMA_Stop(channel);

    if (0U == DMA_WaitTillFree(channel))
    {
        /* This should never happen, the trace here
         * will help to detect this condition during testing */
    }

    /* Assign DMA AHB Address */
    dmaChRegs->DMA_CHAN00_DSTART  = dataAHBAddress;

    /* Reset the control reg to ensure that subsequent OR operations are
     * reflected correctly. Most relevant when DMA switches over from Tx to Rx
     * with DIR changing from 1 to 0*/
    dmaChRegs->DMA_CHAN00_CTRL = 0;

    /* select device,direction */
    dmaChRegs->DMA_CHAN00_CTRL = (uint32_t)(((uint32_t)device<<DMA_CHAN00_CTRL_HW_FLOW_CTRL_DEV_Pos) |
                      DMA_CHAN00_CTRL_INC_MEM_ADDR_Msk | DMA_CHAN00_CTRL_TRANS_SIZE(1));
}

uint8_t DMA_GetDeviceId(const uint8_t device_name, const uint8_t device_instance)
{
    uint8_t device_id = 0;

    switch (device_name)
    {
        case 0:
            device_id = (uint8_t)(2U * device_instance);
            break;

        case 1:
            device_id = 1U + (uint8_t)(2U * device_instance);
            break;

        case 2:
            device_id = 4U;
            break;

        default:
            /* invalid device name */
            break;
    }

    return device_id;
}

void DMA_EnableInterrupt(DMA_CHANNEL channel)
{
    dma_chan00_registers_t* dmaChRegs = DMA_ChannelBaseAddrGet(channel);

    dmaChRegs->DMA_CHAN00_ISTS = DMA_CHAN00_ISTS_DONE_Msk;
    dmaChRegs->DMA_CHAN00_IEN = DMA_CHAN00_IEN_STS_EN_DONE_Msk;
}
void DMA_DisableInterrupt(DMA_CHANNEL channel)
{
    dma_chan00_registers_t* dmaChRegs = DMA_ChannelBaseAddrGet(channel);

    dmaChRegs->DMA_CHAN00_ISTS = DMA_CHAN00_ISTS_DONE_Msk;
    dmaChRegs->DMA_CHAN00_IEN = 0;
}
