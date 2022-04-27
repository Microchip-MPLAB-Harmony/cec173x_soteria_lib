/*******************************************************************************
  UART0 PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_uart0.c

  Summary:
    UART0 PLIB Implementation File

  Description:
    None

*******************************************************************************/

/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
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

#include "interrupts.h"
#include "plib_uart0.h"
#include "../ecia/plib_ecia.h"

// *****************************************************************************
// *****************************************************************************
// Section: UART0 Implementation
// *****************************************************************************
// *****************************************************************************

static UART_ERROR errorStatus = 0;

void UART0_Initialize( void )
{
    UART0_REGS->DATA.UART_LCR |= UART_DATA_LCR_DLAB_Msk;
    UART0_REGS->DLAB.UART_BAUDRT_MSB = 0x0;
    UART0_REGS->DLAB.UART_BAUDRT_LSB = 0x1;
    UART0_REGS->DATA.UART_LCR &= ~UART_DATA_LCR_DLAB_Msk;
    UART0_REGS->DATA.UART_LCR = 0x3;

    /* Turn ON UART0 */
    UART0_REGS->DATA.UART_ACTIVATE = 0x01;
}

bool UART0_SerialSetup(UART_SERIAL_SETUP* setup, uint32_t srcClkFreq )
{
    bool status = false;
    uint32_t baud;
    uint32_t baud_clk_src = 1843200;
    uint32_t baud_div;

    if (setup != NULL)
    {
        baud = setup->baudRate;

        /* Set DLAB = 1 */
        UART0_REGS->DATA.UART_LCR |= UART_DATA_LCR_DLAB_Msk;

        if ((UART0_REGS->DLAB.UART_BAUDRT_MSB & 0x80U) != 0U)
        {
            baud_clk_src = 48000000;
        }

        baud_div = (baud_clk_src >> 4)/baud;

        if ((baud_div < 1U) || (baud_div > 32767U))
        {
            /* Set DLAB = 0 */
            UART0_REGS->DATA.UART_LCR &= ~UART_DATA_LCR_DLAB_Msk;

            return status;
        }

        UART0_REGS->DLAB.UART_BAUDRT_LSB = (uint8_t)baud_div;
        UART0_REGS->DLAB.UART_BAUDRT_MSB |= (uint8_t)((baud_div & 0x7F00U) >> 8U);

        /* Set DLAB = 0 */
        UART0_REGS->DATA.UART_LCR &= ~UART_DATA_LCR_DLAB_Msk;

        UART0_REGS->DATA.UART_LCR = (UART0_REGS->DATA.UART_LCR & ~(UART_DATA_LCR_PAR_SEL_Msk | UART_DATA_LCR_STOP_BITS_Msk | UART_DATA_LCR_WORD_LEN_Msk)) | ((uint8_t)setup->parity | (uint8_t)setup->stopBits | (uint8_t)setup->dataWidth);

        if (setup->parity == UART_PARITY_NONE)
        {
            UART0_REGS->DATA.UART_LCR &= ~UART_DATA_LCR_EN_PAR_Msk;
        }
        else
        {
            UART0_REGS->DATA.UART_LCR |= UART_DATA_LCR_EN_PAR_Msk;
        }

        status = true;
    }

    return status;
}

bool UART0_Read(void* buffer, const size_t size )
{
    bool status = false;
    uint8_t lsr;
    uint8_t* pRxBuffer = (uint8_t* )buffer;
    size_t processedSize = 0;

    if(pRxBuffer != NULL)
    {
        /* Clear error flags that may have been received when no active request was pending */
        lsr = UART0_REGS->DATA.UART_LSR;

        while( size > processedSize )
        {
            /* Wait for data ready flag */
            do
            {
                lsr = UART0_REGS->DATA.UART_LSR;
            }while ((lsr & (UART_DATA_LSR_DATA_READY_Msk | UART_DATA_LSR_OVERRUN_Msk | UART_DATA_LSR_PE_Msk | UART_DATA_LSR_FRAME_ERR_Msk)) == 0U);

            /* Check for overrun, parity and framing errors */
            lsr = (lsr & (UART_DATA_LSR_OVERRUN_Msk | UART_DATA_LSR_PE_Msk | UART_DATA_LSR_FRAME_ERR_Msk));
            errorStatus = lsr;

            if ((uint32_t)errorStatus != 0U)
            {
                break;
            }
            else
            {
                pRxBuffer[processedSize] = UART0_REGS->DATA.UART_RX_DAT;
                processedSize++;
            }
        }

        if(size == processedSize)
        {
            status = true;
        }
    }

    return status;
}

bool UART0_Write( void* buffer, const size_t size )
{
    bool status = false;
    uint8_t* pTxBuffer = (uint8_t*)buffer;
    size_t processedSize = 0;

    if(pTxBuffer != NULL)
    {

        while( size > processedSize )
        {
            /* Wait for transmitter to become ready */
            while ((UART0_REGS->DATA.UART_LSR & UART_DATA_LSR_TRANS_EMPTY_Msk) == 0U)
            {
            }
            UART0_REGS->DATA.UART_TX_DAT = pTxBuffer[processedSize];
            processedSize++;
        }

        status = true;
    }

    return status;
}

UART_ERROR UART0_ErrorGet( void )
{
    UART_ERROR errors = errorStatus;

    errorStatus = UART_ERROR_NONE;

    /* All errors are cleared, but send the previous error state */
    return errors;
}


bool UART0_TransmitComplete( void )
{
    bool transmitComplete = false;

    if ((UART0_REGS->DATA.UART_LSR & UART_DATA_LSR_TRANS_ERR_Msk) != 0U)
    {
        transmitComplete = true;
    }

    return transmitComplete;
}

void UART0_WriteByte(int data)
{
    while ((UART0_REGS->DATA.UART_LSR & UART_DATA_LSR_TRANS_EMPTY_Msk) == 0U)
    {
        /* Do nothing */
    }

    UART0_REGS->DATA.UART_TX_DAT = (uint8_t)data;
}

bool UART0_TransmitterIsReady( void )
{
    bool transmitterReady = false;

    if ((UART0_REGS->DATA.UART_LSR & UART_DATA_LSR_TRANS_EMPTY_Msk) != 0U)
    {
        transmitterReady = true;
    }

    return transmitterReady;
}

int UART0_ReadByte( void )
{
    return (int)UART0_REGS->DATA.UART_RX_DAT;
}

bool UART0_ReceiverIsReady( void )
{
    bool receiverReady = false;

    if ((UART0_REGS->DATA.UART_LSR & UART_DATA_LSR_DATA_READY_Msk) != 0U)
    {
        receiverReady = true;
    }

    return receiverReady;
}
