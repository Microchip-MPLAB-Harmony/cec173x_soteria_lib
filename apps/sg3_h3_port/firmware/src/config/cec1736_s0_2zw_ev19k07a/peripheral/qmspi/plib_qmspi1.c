/*******************************************************************************
  QMSPI1 Peripheral Library Source File

  Company
    Microchip Technology Inc.

  File Name
    plib_qmspi1.c

  Summary
    QMSPI1 peripheral library interface.

  Description

  Remarks:

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
#include "interrupts.h"
#include "plib_qmspi1.h"
#include "../ecia/plib_ecia.h"


#define QMSPI_SOURCE_CLOCK_FREQUENCY (96000000U)
#define QMSPI_MAX_DESCR              (16U)
#define SWAP32(x)                    ((((x) & 0xffU) << 24U) | (((x) & 0xff00U) << 8U) | (((x) & 0xff0000U) >> 8U) | (((x) & 0xff000000U) >> 24U))
static QMSPI_OBJECT qmspi1Obj;
static const uint8_t qmspiIoMode[7U][3U] = {{0U, 0U ,0U}, /* IO mode for Command, Address, Data */
                                            {0U, 0U, 1U},
                                            {0U, 0U, 2U},
                                            {0U, 1U, 1U},
                                            {0U, 2U, 2U},
                                            {1U, 1U, 1U},
                                            {2U, 2U, 2U}};

// *****************************************************************************
// *****************************************************************************
// QMSPI1 Local Functions
// *****************************************************************************
// *****************************************************************************

static uint8_t QMSPI1_DMALenGet(uint32_t address, uint32_t size)
{
    if (((address | size) & 0x03U) == 0U)
    {
        return 2U;
    }
    else if (((address | size) & 0x01U) == 0U)
    {
        return 1U;
    }
    else
    {
        return 0U;
    }
}

static uint8_t QMSPI1_XferUnitLenGet(uint32_t size)
{
    if ((size & 0x0FU) == 0U)
    {
        return 3U;
    }
    else if ((size & 0x03U) == 0U)
    {
        return 2U;
    } else
    {
        return 1U;
    }
}

static uint8_t QMSPI1_XferUnitShiftGet(uint32_t size)
{
    if ((size & 0x0FU) == 0U)
    {
        return 4U;
    }
    else if ((size & 0x03U) == 0U)
    {
        return 2U;
    }
    else
    {
        return 0U;
    }
}

// *****************************************************************************
// *****************************************************************************
// QMSPI1 PLIB Interface Routines
// *****************************************************************************
// *****************************************************************************

void QMSPI1_Initialize(void)
{
    // Reset the QMSPI Block
    QMSPI1_REGS->QMSPI_MODE = QMSPI_MODE_SOFT_RESET_Msk;

    QMSPI1_REGS->QMSPI_MODE = QMSPI_MODE_CLK_DIV(8U);

    // Activate the QMSPI Block
    QMSPI1_REGS->QMSPI_MODE |= QMSPI_MODE_ACT_Msk;
}

void QMSPI1_ChipSelectSetup(QMSPI_CHIP_SELECT chipSelect)
{
    QMSPI1_REGS->QMSPI_MODE = (QMSPI1_REGS->QMSPI_MODE & ~QMSPI_MODE_CS_Msk) | QMSPI_MODE_CS(chipSelect);
}

bool QMSPI1_TransferSetup (QMSPI_TRANSFER_SETUP *setup)
{
    uint32_t clock_divide;
    bool setupStatus = false;

    if (setup != NULL)
    {
        clock_divide = QMSPI_SOURCE_CLOCK_FREQUENCY / setup->clockFrequency;

        if (clock_divide >= 256U)
        {
            clock_divide = 0;
        }

        QMSPI1_REGS->QMSPI_MODE = ((QMSPI1_REGS->QMSPI_MODE & ~QMSPI_MODE_CLK_DIV_Msk) | QMSPI_MODE_CLK_DIV(clock_divide))
                                                 | ((QMSPI1_REGS->QMSPI_MODE & ~QMSPI_MODE_CPOL_Msk) | (uint32_t)setup->clockPolarity)
                                                 | ((QMSPI1_REGS->QMSPI_MODE & ~QMSPI_MODE_CHPA_MOSI_Msk) | (uint32_t)setup->clockPhaseMOSI)
                                                 | ((QMSPI1_REGS->QMSPI_MODE & ~QMSPI_MODE_CHPA_MISO_Msk) | (uint32_t)setup->clockPhaseMISO);
        setupStatus = true;
    }
    return setupStatus;
}

void QMSPI1_TapControlSet(uint16_t tap_vl, uint32_t tap_ctrl)
{
    QMSPI1_REGS->QMSPI_TAPS = tap_vl;
    QMSPI1_REGS->QMSPI_TAP_CTRL = (tap_ctrl & QMSPI_TAP_CTRL_Msk);
}

/* Manual mode - command, register and memory write */
bool QMSPI1_Write(QMSPI_XFER_T *qmspiXfer, void* pTransmitData, size_t txSize)
{
    size_t xferLength = 0U;
    size_t count = 0U;
    bool   status = false;

    if (qmspiXfer != NULL)
    {
        if ((QMSPI1_REGS->QMSPI_STS & QMSPI_STS_TRANS_ACTIV_Msk) == 0U)
        {
            QMSPI1_REGS->QMSPI_EXE = QMSPI_EXE_CLR_DAT_BUFF_Msk;
            QMSPI1_REGS->QMSPI_STS = QMSPI_STS_Msk;

            *(volatile uint8_t *)(&QMSPI1_REGS->QMSPI_TX_FIFO[0]) = qmspiXfer->command;
            QMSPI1_REGS->QMSPI_CTRL = QMSPI_CTRL_TX_MODE(qmspiIoMode[qmspiXfer->qmspi_ifc_mode][0]) | QMSPI_CTRL_TX_TRANS_EN(1U) | QMSPI_CTRL_TRANS_UNITS(1U) | QMSPI_CTRL_TRANS_LEN(1U);
            QMSPI1_REGS->QMSPI_EXE = QMSPI_EXE_START_Msk;
            while ((QMSPI1_REGS->QMSPI_STS & QMSPI_STS_TX_BUFF_EMP_Msk) == 0U)
            {
                // wait if transmit buffer is not empty
            }

            if (qmspiXfer->address_enable)
            {
                /* Check if 32-bit address enable */
                if (qmspiXfer->address_32_bit_en)
                {
                    QMSPI1_REGS->QMSPI_TX_FIFO[0] = SWAP32(qmspiXfer->address);
                    xferLength = 4U;
                } else
                {
                    uint32_t shift = 24U;
                    xferLength = 3U;
                    count = 0;
                    while(count < xferLength)
                    {
                        shift -= 8U;
                        *(volatile uint8_t *)(&QMSPI1_REGS->QMSPI_TX_FIFO[0]) = (uint8_t)((qmspiXfer->address >> shift) & 0xFFU);
                        count++;
                    }
                }

                QMSPI1_REGS->QMSPI_CTRL = QMSPI_CTRL_TX_MODE(qmspiIoMode[qmspiXfer->qmspi_ifc_mode][1]) | QMSPI_CTRL_TX_TRANS_EN(1U) | QMSPI_CTRL_TRANS_UNITS(1U) | QMSPI_CTRL_TRANS_LEN(xferLength);
                QMSPI1_REGS->QMSPI_EXE = QMSPI_EXE_START_Msk;
                while ((QMSPI1_REGS->QMSPI_STS & QMSPI_STS_TX_BUFF_EMP_Msk) == 0U)
                {
                    // wait if transmit buffer is not empty
                }
            }

            while (txSize > 0U)
            {
                if (txSize >= (QMSPI_CTRL_TRANS_LEN_Msk >> QMSPI_CTRL_TRANS_LEN_Pos))
                {
                    xferLength = (QMSPI_CTRL_TRANS_LEN_Msk >> QMSPI_CTRL_TRANS_LEN_Pos);
                }
                else
                {
                    xferLength = txSize;
                }
                QMSPI1_REGS->QMSPI_CTRL = QMSPI_CTRL_TX_MODE(qmspiIoMode[qmspiXfer->qmspi_ifc_mode][2]) | QMSPI_CTRL_TX_TRANS_EN(1U) | QMSPI_CTRL_TRANS_UNITS(1U) | QMSPI_CTRL_TRANS_LEN(xferLength);
                QMSPI1_REGS->QMSPI_EXE = QMSPI_EXE_START_Msk;

                count = 0U;
                while (count < xferLength)
                {
                    while ((QMSPI1_REGS->QMSPI_STS & QMSPI_STS_TX_BUFF_FULL_Msk) != 0U)
                    {
                        // wait if transmit buffer is full
                    }
                    *(volatile uint8_t *)(&QMSPI1_REGS->QMSPI_TX_FIFO[0]) = ((uint8_t *)pTransmitData)[count];
                    count++;
                }
                while ((QMSPI1_REGS->QMSPI_STS & QMSPI_STS_TX_BUFF_EMP_Msk) == 0U)
                {
                    // wait if transmit buffer is not empty
                }
                txSize -= xferLength;
            }

            QMSPI1_REGS->QMSPI_CTRL |= QMSPI_CTRL_CLOSE_TRANS_EN_Msk;
            QMSPI1_REGS->QMSPI_EXE = QMSPI_EXE_STOP_Msk;
            status = true;
        }
    }
    return status;
}

/* Manual mode - register and memory read */
bool QMSPI1_Read(QMSPI_XFER_T *qmspiXfer, void* pReceiveData, size_t rxSize)
{
    size_t xferLength = 0U;
    size_t count = 0U;
    bool   status = false;

    if ((qmspiXfer != NULL) && (rxSize > 0U) && (pReceiveData != NULL))
    {
        if ((QMSPI1_REGS->QMSPI_STS & QMSPI_STS_TRANS_ACTIV_Msk) == 0U)
        {
            QMSPI1_REGS->QMSPI_EXE = QMSPI_EXE_CLR_DAT_BUFF_Msk;
            QMSPI1_REGS->QMSPI_STS = QMSPI_STS_Msk;

            *(volatile uint8_t *)(&QMSPI1_REGS->QMSPI_TX_FIFO[0]) = qmspiXfer->command;
            QMSPI1_REGS->QMSPI_CTRL = QMSPI_CTRL_TX_MODE(qmspiIoMode[qmspiXfer->qmspi_ifc_mode][0]) | QMSPI_CTRL_TX_TRANS_EN(1U) | QMSPI_CTRL_TRANS_UNITS(1U) | QMSPI_CTRL_TRANS_LEN(1U);
            QMSPI1_REGS->QMSPI_EXE = QMSPI_EXE_START_Msk;
            while ((QMSPI1_REGS->QMSPI_STS & QMSPI_STS_TX_BUFF_EMP_Msk) == 0U)
            {
                // wait if transmit buffer is not empty
            }

            if (qmspiXfer->address_enable)
            {
                /* Check if 32-bit address enable */
                if (qmspiXfer->address_32_bit_en)
                {
                    QMSPI1_REGS->QMSPI_TX_FIFO[0] = SWAP32(qmspiXfer->address);
                    xferLength = 4U;
                } else
                {
                    uint32_t shift = 24U;
                    xferLength = 3U;
                    count = 0;
                    while(count < xferLength)
                    {
                        shift -= 8U;
                        *(volatile uint8_t *)(&QMSPI1_REGS->QMSPI_TX_FIFO[0]) = (uint8_t)((qmspiXfer->address >> shift) & 0xFFU);
                        count++;
                    }
                }

                QMSPI1_REGS->QMSPI_CTRL = QMSPI_CTRL_TX_MODE(qmspiIoMode[qmspiXfer->qmspi_ifc_mode][1]) | QMSPI_CTRL_TX_TRANS_EN(1U) | QMSPI_CTRL_TRANS_UNITS(1U) | QMSPI_CTRL_TRANS_LEN(xferLength);
                QMSPI1_REGS->QMSPI_EXE = QMSPI_EXE_START_Msk;
                while ((QMSPI1_REGS->QMSPI_STS & QMSPI_STS_TX_BUFF_EMP_Msk) == 0U)
                {
                    // wait if transmit buffer is not empty
                }
            }

            if (qmspiXfer->num_of_dummy_byte > 0U)
            {
                QMSPI1_REGS->QMSPI_CTRL = QMSPI_CTRL_TX_MODE(qmspiIoMode[qmspiXfer->qmspi_ifc_mode][2]) | QMSPI_CTRL_TRANS_UNITS(1U) | QMSPI_CTRL_TRANS_LEN(qmspiXfer->num_of_dummy_byte);
                QMSPI1_REGS->QMSPI_EXE = QMSPI_EXE_START_Msk;
                while ((QMSPI1_REGS->QMSPI_STS & QMSPI_STS_TRANS_COMPL_Msk) == 0U)
                {
                    // wait for transmit complete
                }
            }

            while (rxSize > 0U)
            {
                if (rxSize >= (QMSPI_CTRL_TRANS_LEN_Msk >> QMSPI_CTRL_TRANS_LEN_Pos))
                {
                    xferLength = (QMSPI_CTRL_TRANS_LEN_Msk >> QMSPI_CTRL_TRANS_LEN_Pos);
                }
                else
                {
                    xferLength = rxSize;
                }
                QMSPI1_REGS->QMSPI_CTRL = QMSPI_CTRL_TX_MODE(qmspiIoMode[qmspiXfer->qmspi_ifc_mode][2]) | QMSPI_CTRL_RX_TRANS_EN(1U) | QMSPI_CTRL_TRANS_UNITS(1U) | QMSPI_CTRL_TRANS_LEN(xferLength);
                QMSPI1_REGS->QMSPI_EXE = QMSPI_EXE_START_Msk;

                count = 0U;
                while (count < xferLength)
                {
                    while ((QMSPI1_REGS->QMSPI_STS & QMSPI_STS_RX_BUFF_EMP_Msk) != 0U)
                    {
                        // wait if receive buffer is empty
                    }
                    ((uint8_t *)pReceiveData)[count] = *(volatile uint8_t *)(&QMSPI1_REGS->QMSPI_RX_FIFO[0]);
                    count++;
                }
                rxSize -= xferLength;
            }

            QMSPI1_REGS->QMSPI_CTRL |= QMSPI_CTRL_CLOSE_TRANS_EN_Msk;
            QMSPI1_REGS->QMSPI_EXE = QMSPI_EXE_STOP_Msk;
            status = true;
        }
    }
    return status;
}

bool QMSPI1_IsTransmitterBusy(void)
{
    return ((QMSPI1_REGS->QMSPI_STS & QMSPI_STS_TX_BUFF_EMP_Msk) == 0U);
}

/* Memory write with Local/Central DMA */
uint32_t QMSPI1_DMATransferWrite(QMSPI_DESCRIPTOR_XFER_T *qmspiDescXfer, void* pTransmitData, size_t txSize)
{
    uint32_t desc_id, size, xferLength;
    uint32_t txBytes = 0U;
    uint8_t len, xferUnitLen, xferUnitShift;

    if ((qmspiDescXfer != NULL) && (txSize > 0U))
    {

        if ((QMSPI1_REGS->QMSPI_STS & QMSPI_STS_TRANS_ACTIV_Msk) == 0U)
        {
            QMSPI1_REGS->QMSPI_CTRL = QMSPI_CTRL_DESCR_BUFF_EN_Msk;
            QMSPI1_REGS->QMSPI_EXE = QMSPI_EXE_CLR_DAT_BUFF_Msk;
            QMSPI1_REGS->QMSPI_STS = QMSPI_STS_Msk;

            *(volatile uint8_t *)(&QMSPI1_REGS->QMSPI_TX_FIFO[0]) = qmspiDescXfer->command;
            desc_id = 0;
            /* Descriptor 0 - Command */
            QMSPI1_REGS->QMSPI_DESCR[desc_id] = QMSPI_DESCR_INFACE_MOD(qmspiIoMode[qmspiDescXfer->qmspi_ifc_mode][0])
                                                              | QMSPI_DESCR_TX_TRANS_EN(1U)
                                                              | QMSPI_DESCR_TRANS_LEN(1U)
                                                              | QMSPI_DESCR_DESCR_BUF_NXT_PTR((desc_id + 1U))
                                                              | QMSPI_DESCR_TX_LEN(1U);
            desc_id++;

            /* Check if 32-bit address enable */
            if (qmspiDescXfer->address_32_bit_en)
            {
                QMSPI1_REGS->QMSPI_TX_FIFO[0] = SWAP32(qmspiDescXfer->address);
                xferLength = 4U;
            } else
            {
                uint32_t shift = 24U;
                xferLength = 3U;
                txBytes = 0;
                while(txBytes < xferLength)
                {
                    shift -= 8U;
                    *(volatile uint8_t *)(&QMSPI1_REGS->QMSPI_TX_FIFO[0]) = (uint8_t)((qmspiDescXfer->address >> shift) & 0xFFU);
                    txBytes++;
                }
            }

            /* Descriptor 1 - Address */
            QMSPI1_REGS->QMSPI_DESCR[desc_id] = QMSPI_DESCR_INFACE_MOD(qmspiIoMode[qmspiDescXfer->qmspi_ifc_mode][1])
                                                              | QMSPI_DESCR_TX_TRANS_EN(1U)
                                                              | QMSPI_DESCR_TRANS_LEN(1U)
                                                              | QMSPI_DESCR_DESCR_BUF_NXT_PTR((desc_id + 1U))
                                                              | QMSPI_DESCR_TX_LEN(xferLength);
            desc_id++;

            len = QMSPI1_DMALenGet((uint32_t)((uint32_t*)pTransmitData), txSize);

            if (qmspiDescXfer->ldma_enable)
            {
                QMSPI1_REGS->QMSPI_MODE |= QMSPI_MODE_LDMA_TXEN_Msk;

                QMSPI1_REGS->QMSPI_DESCR[desc_id] = QMSPI_DESCR_INFACE_MOD(qmspiIoMode[qmspiDescXfer->qmspi_ifc_mode][2])
                                                              | QMSPI_DESCR_TX_DMA_EN((qmspiDescXfer->ldma_channel_num + 1U))
                                                              | QMSPI_DESCR_TX_TRANS_EN(1U)
                                                              | QMSPI_DESCR_DESCR_BUF_NXT_PTR((desc_id + 1U))
                                                              | QMSPI_DESCR_DESCR_BUF_LAST_Msk
                                                              | QMSPI_DESCR_CLOSE_TRANS_EN_Msk;

                QMSPI1_REGS->QMSPI_DESC_LDMA_TXEN = QMSPI_DESC_LDMA_TXEN_DESC_LDMA_TXEN_Msk;

                QMSPI1_REGS->LDMA_TX[qmspiDescXfer->ldma_channel_num].QMSPI_LDMA_TXCTRL = QMSPI_LDMA_TXCTRL_CH_EN_Msk
                                                              | QMSPI_LDMA_TXCTRL_OVRD_LEN_Msk
                                                              | QMSPI_LDMA_TXCTRL_ACS_SZ(len);
                if (qmspiDescXfer->ldma_incr_addr_disable == false)
                {
                    QMSPI1_REGS->LDMA_TX[qmspiDescXfer->ldma_channel_num].QMSPI_LDMA_TXCTRL |= QMSPI_LDMA_TXCTRL_INC_ADDR_EN_Msk;
                }
                QMSPI1_REGS->LDMA_TX[qmspiDescXfer->ldma_channel_num].QMSPI_LDMA_TXSTRT_ADDR = (uint32_t)((uint32_t*)pTransmitData);
                QMSPI1_REGS->LDMA_TX[qmspiDescXfer->ldma_channel_num].QMSPI_LDMA_TX_LEN = txSize;
                txBytes = txSize;
            }
            else
            {
                QMSPI1_REGS->QMSPI_MODE &= ~QMSPI_MODE_LDMA_TXEN_Msk;
                xferUnitLen = QMSPI1_XferUnitLenGet(txSize);
                xferUnitShift = QMSPI1_XferUnitShiftGet(txSize);
                size = txSize >> xferUnitShift;
                while (size > 0U)
                {
                    xferLength = size;
                    if (size > (QMSPI_DESCR_TX_LEN_Msk >> QMSPI_DESCR_TX_LEN_Pos))
                    {
                        xferLength = (QMSPI_DESCR_TX_LEN_Msk >> QMSPI_DESCR_TX_LEN_Pos);
                    }

                    QMSPI1_REGS->QMSPI_DESCR[desc_id] = QMSPI_DESCR_INFACE_MOD(qmspiIoMode[qmspiDescXfer->qmspi_ifc_mode][2]) | QMSPI_DESCR_TX_TRANS_EN(1U)
                                                                | QMSPI_DESCR_TRANS_LEN(xferUnitLen) | QMSPI_DESCR_TX_DMA_EN(((uint32_t)len + 1U))
                                                                | QMSPI_DESCR_TX_LEN(xferLength) | QMSPI_DESCR_DESCR_BUF_NXT_PTR((desc_id + 1U));
                    size -= xferLength;

                    desc_id++;
                    if (desc_id >= QMSPI_MAX_DESCR)
                    {
                        break;
                    }
                }
                if (desc_id != 0U)
                {
                    desc_id--;
                }
                QMSPI1_REGS->QMSPI_DESCR[desc_id] |= QMSPI_DESCR_DESCR_BUF_LAST_Msk | QMSPI_DESCR_CLOSE_TRANS_EN_Msk;
                txBytes = (txSize - (size << xferUnitShift));
            }
            QMSPI1_REGS->QMSPI_IEN |= QMSPI_IEN_TRANS_COMPL_EN_Msk;
            QMSPI1_REGS->QMSPI_EXE = QMSPI_EXE_START_Msk;
        }
    }
    return txBytes;
}

/* Memory read with Local/Central DMA */
uint32_t QMSPI1_DMATransferRead(QMSPI_DESCRIPTOR_XFER_T *qmspiDescXfer, void* pReceiveData, size_t rxSize)
{
    uint32_t desc_id, count, xferLength;
    uint32_t size = 0U;
    uint8_t len, xferUnitLen, xferUnitShift;

    if ((qmspiDescXfer != NULL) && (rxSize > 0U) && (pReceiveData != NULL))
    {
        QMSPI1_REGS->QMSPI_CTRL = QMSPI_CTRL_DESCR_BUFF_EN_Msk;
        QMSPI1_REGS->QMSPI_EXE = QMSPI_EXE_CLR_DAT_BUFF_Msk;
        QMSPI1_REGS->QMSPI_STS = QMSPI_STS_Msk;

        *(volatile uint8_t *)(&QMSPI1_REGS->QMSPI_TX_FIFO[0]) = qmspiDescXfer->command;
        desc_id = 0;
        /* Descriptor 0 - Command */
        QMSPI1_REGS->QMSPI_DESCR[desc_id] = QMSPI_DESCR_INFACE_MOD(qmspiIoMode[qmspiDescXfer->qmspi_ifc_mode][0])
                                                          | QMSPI_DESCR_TX_TRANS_EN(1U)
                                                          | QMSPI_DESCR_TRANS_LEN(1U)
                                                          | QMSPI_DESCR_DESCR_BUF_NXT_PTR((desc_id + 1U))
                                                          | QMSPI_DESCR_TX_LEN(1U);
        desc_id++;

        /* Check if 32-bit address enable */
        if (qmspiDescXfer->address_32_bit_en)
        {
            QMSPI1_REGS->QMSPI_TX_FIFO[0] = SWAP32(qmspiDescXfer->address);
            xferLength = 4U;
        } else
        {
            uint32_t shift = 24U;
            xferLength = 3U;
            count = 0;
            while(count < xferLength)
            {
                shift -= 8U;
                *(volatile uint8_t *)(&QMSPI1_REGS->QMSPI_TX_FIFO[0]) = (uint8_t)((qmspiDescXfer->address >> shift) & 0xFFU);
                count++;
            }
        }

        /* Descriptor 1 - Address */
        QMSPI1_REGS->QMSPI_DESCR[desc_id] = QMSPI_DESCR_INFACE_MOD(qmspiIoMode[qmspiDescXfer->qmspi_ifc_mode][1])
                                                          | QMSPI_DESCR_TX_TRANS_EN(1U)
                                                          | QMSPI_DESCR_TRANS_LEN(1U)
                                                          | QMSPI_DESCR_DESCR_BUF_NXT_PTR((desc_id + 1U))
                                                          | QMSPI_DESCR_TX_LEN(xferLength);
        desc_id++;

        /* Dummy Bytes */
        if (qmspiDescXfer->num_of_dummy_byte > 0U)
        {
            QMSPI1_REGS->QMSPI_DESCR[desc_id] = QMSPI_DESCR_INFACE_MOD(qmspiIoMode[qmspiDescXfer->qmspi_ifc_mode][2])
                                                          | QMSPI_DESCR_TRANS_LEN(1U)
                                                          | QMSPI_DESCR_DESCR_BUF_NXT_PTR((desc_id + 1U))
                                                          | QMSPI_DESCR_TX_LEN(qmspiDescXfer->num_of_dummy_byte);
            desc_id++;
        }

        len = QMSPI1_DMALenGet((uint32_t)((uint32_t*)pReceiveData), rxSize);

        if (qmspiDescXfer->ldma_enable)
        {
            QMSPI1_REGS->QMSPI_MODE |= QMSPI_MODE_LDMA_RXEN_Msk;

            QMSPI1_REGS->QMSPI_DESCR[desc_id] = QMSPI_DESCR_INFACE_MOD(qmspiIoMode[qmspiDescXfer->qmspi_ifc_mode][2])
                                                          | QMSPI_DESCR_RX_DMA_EN((qmspiDescXfer->ldma_channel_num + 1U))
                                                          | QMSPI_DESCR_RX_TRANS_EN_Msk
                                                          | QMSPI_DESCR_DESCR_BUF_NXT_PTR((desc_id + 1U))
                                                          | QMSPI_DESCR_DESCR_BUF_LAST_Msk
                                                          | QMSPI_DESCR_CLOSE_TRANS_EN_Msk;

            QMSPI1_REGS->QMSPI_DESC_LDMA_RXEN = QMSPI_DESC_LDMA_RXEN_DESC_LDMA_RXEN_Msk;

            QMSPI1_REGS->LDMA_RX[qmspiDescXfer->ldma_channel_num].QMSPI_LDMA_RXCTRL = QMSPI_LDMA_RXCTRL_CH_EN_Msk
                                                          | QMSPI_LDMA_RXCTRL_OVRD_LEN_Msk
                                                          | QMSPI_LDMA_RXCTRL_ACS_SZ(len);
            if (qmspiDescXfer->ldma_incr_addr_disable == false)
            {
                QMSPI1_REGS->LDMA_RX[qmspiDescXfer->ldma_channel_num].QMSPI_LDMA_RXCTRL |= QMSPI_LDMA_RXCTRL_INC_ADDR_EN_Msk;
            }
            QMSPI1_REGS->LDMA_RX[qmspiDescXfer->ldma_channel_num].QMSPI_LDMA_RXSTRT_ADDR = (uint32_t)((uint32_t*)pReceiveData);
            QMSPI1_REGS->LDMA_RX[qmspiDescXfer->ldma_channel_num].QMSPI_LDMA_RX_LEN = rxSize;
            size = rxSize;
        }
        else
        {
            QMSPI1_REGS->QMSPI_MODE &= ~QMSPI_MODE_LDMA_RXEN_Msk;
            xferUnitLen = QMSPI1_XferUnitLenGet(rxSize);
            xferUnitShift = QMSPI1_XferUnitShiftGet(rxSize);
            size = rxSize >> xferUnitShift;

            count = 0U;
            while (count < size)
            {
                xferLength = size - count;
                if (xferLength > (QMSPI_DESCR_TX_LEN_Msk >> QMSPI_DESCR_TX_LEN_Pos))
                {
                    xferLength = (QMSPI_DESCR_TX_LEN_Msk >> QMSPI_DESCR_TX_LEN_Pos);
                }
                count += xferLength;

                QMSPI1_REGS->QMSPI_DESCR[desc_id] = QMSPI_DESCR_INFACE_MOD(qmspiIoMode[qmspiDescXfer->qmspi_ifc_mode][2]) | QMSPI_DESCR_RX_TRANS_EN_Msk
                                                                | QMSPI_DESCR_TRANS_LEN(xferUnitLen) | QMSPI_DESCR_RX_DMA_EN(((uint32_t)len + 1U))
                                                                | QMSPI_DESCR_TX_LEN(xferLength) | QMSPI_DESCR_DESCR_BUF_NXT_PTR((desc_id + 1U));

                desc_id++;
                if (desc_id >= QMSPI_MAX_DESCR)
                {
                    break;
                }
            }

            if (desc_id != 0U)
            {
                desc_id--;
            }
            QMSPI1_REGS->QMSPI_DESCR[desc_id] |= QMSPI_DESCR_DESCR_BUF_LAST_Msk | QMSPI_DESCR_CLOSE_TRANS_EN_Msk;

            size = (count << xferUnitShift);
        }
        QMSPI1_REGS->QMSPI_IEN |= QMSPI_IEN_TRANS_COMPL_EN_Msk;
        QMSPI1_REGS->QMSPI_EXE = QMSPI_EXE_START_Msk;
    }
    return size;
}

void QMSPI1_CallbackRegister(QMSPI_CALLBACK callback, uintptr_t context)
{
    qmspi1Obj.callback = callback;
    qmspi1Obj.context = context;
}

void QMSPI1_GRP_InterruptHandler(void)
{
    ECIA_GIRQSourceClear(ECIA_AGG_INT_SRC_QMSPI1);

    if ((QMSPI1_REGS->QMSPI_STS & QMSPI_STS_TRANS_COMPL_Msk) != 0U)
    {
        QMSPI1_REGS->QMSPI_STS |= QMSPI_STS_TRANS_COMPL_Msk;
        QMSPI1_REGS->QMSPI_IEN &= ~QMSPI_IEN_TRANS_COMPL_EN_Msk;
        if(qmspi1Obj.callback != NULL)
        {
            qmspi1Obj.callback(qmspi1Obj.context);
        }
    }
}

/*******************************************************************************
 End of File
*/
