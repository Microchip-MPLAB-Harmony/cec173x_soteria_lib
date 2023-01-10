/*****************************************************************************
* © 2018 Microchip Technology Inc. and its subsidiaries.
* You may use this software and any derivatives exclusively with
* Microchip products.
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".
* NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
* INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
* AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.
* TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
* CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF
* FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
* MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE
* OF THESE TERMS.
******************************************************************************

Version Control Information (Perforce)
******************************************************************************
$Revision: #3 $ 
$DateTime: 2023/01/10 04:01:19 $ 
$Author: i64652 $
Last Change:    None
******************************************************************************/
/** @file uart_api.c
* \brief UART API Source file
* \author pramans
* 
* This file implements the UART API functions  
******************************************************************************/

/** @defgroup UART
 *  @{
 */

#include "common.h"
#include "uart_api.h"
#include "peripheral/uart/plib_uart_common.h"
#include "peripheral/uart/plib_uart0.h"

/* ------------------------------------------------------------------------------ */
/*  Local Helper Function - Do not use                                            */

/* ------------------------------------------------------------------------------ */
static uint8_t uart_id_is_valid( uint8_t uart_id )
{
    uint8_t ret_valid;
    if ( uart_id == UART0_ID ) {
        ret_valid = 1u;
    } else {
        ret_valid = 0u;
    }

    return ret_valid;
}


/* --------------------------------------------------------------------------------------------- */
/*                  Function to initialize the UART pins                                         */
/* --------------------------------------------------------------------------------------------- */

/**
 * uart_pins_init - Initializes the gpio pin that are associated
 * with the specified UART instance for UART functionality.
 * @param uint8_t 0-based UART ID
 * @return None.
 */
void uart_pins_init( uint8_t uart_id )
{
	if ( uart_id_is_valid(uart_id) ) {
		switch ( uart_id )
		{
			case UART0_ID:
				//Tx
				AHB_API_gpio_init( GPIO_PIN_GPIO104, GPIO_INP_DISABLE, GPIO_FUNCTION_FUNC1, \
									GPIO_POLARITY_NON_INVERTED, GPIO_DIR_OUTPUT, GPIO_OUTPUT_BUFFER_TYPE_PUSH_PULL, \
									GPIO_INTDET_TYPE_DISABLED, GPIO_PWR_VTR, GPIO_PULL_TYPE_NONE );
				//Rx
				AHB_API_gpio_init( GPIO_PIN_GPIO105, GPIO_INP_ENABLE, GPIO_FUNCTION_FUNC1, \
									GPIO_POLARITY_INVERTED, GPIO_DIR_INPUT, GPIO_OUTPUT_BUFFER_TYPE_PUSH_PULL, \
									GPIO_INTDET_TYPE_DISABLED, GPIO_PWR_VTR, GPIO_PULL_TYPE_NONE );
				break;

			default:
				break;
		}
	}
}

/* --------------------------------------------------------------------------------------------- */
/*                  Function to initialize and enable the UART block                             */
/* --------------------------------------------------------------------------------------------- */

/**
 * uart_hw_init - Initializes the uart block hardware and enables it.
 * @param uint8_t 0-based UART ID
 * @return None
 * @note While using the non - fifo mode; keep the fifo trigger level setting as
 *       UART_FIFO_INT_LVL_1.
 */
void uart_hw_init( uint8_t uart_id )
{
	if ( uart_id_is_valid(uart_id) ) {
		switch ( uart_id )
		{
			case UART0_ID:
				UART0_Initialize();
				break;
			default:
				break;
		}
	}
}

/* --------------------------------------------------------------------------------------------- */
/*                  Function to configure the protocol & interrupt parameters                    */
/* --------------------------------------------------------------------------------------------- */

/**
 * uart_protocol_init - Initializes the serial protocol and interrupt parameters.
 * @param uint8_t 0-based UART ID
 * @param uint32_t desired baud rate value
 * @param uint8_t word length setting
 * @param uint8_t stop bit setting
 * @param uint8_t parity bit setting
 * @param uint8_t interrupt source type that is to be enabled
 * @return None
 * @note In case interrupts are not being used; keep the interrupt source
 *       type parameter value as UART_INT_DISABLED.
 */
void uart_protocol_init( uint8_t uart_id, uint32_t baud, uint8_t wrd_len, uint8_t stp_bit, uint8_t parity_type)
{
	UART_SERIAL_SETUP uart_config = {
		.baudRate = baud,
		.dataWidth = wrd_len,
		.parity = parity_type,
		.stopBits = stp_bit
	};
	
    if ( uart_id_is_valid(uart_id) )
	{
        switch ( uart_id )
		{
			case UART0_ID:
				UART0_SerialSetup(&uart_config, 1843200);
				break;
			default:
				break;
		}
    }
}

/* --------------------------------------------------------------------------------------------- */
/*                  Function to transmit serial data                                             */
/* --------------------------------------------------------------------------------------------- */

/**
 * uart_transmit - Transmits serial data using the specified uart instance.
 * @param uint8_t 0-based UART ID
 * @param uint8_t data to be transmitted
 * @return None.
 */
void uart_transmit( uint8_t uart_id, uint8_t data )
{
    if ( uart_id_is_valid(uart_id) )
	{
        switch ( uart_id )
		{
			case UART0_ID:
				while(false == UART0_TransmitterIsReady());
				UART0_WriteByte((int)data);
				break;
			default:
				break;
		}
    }
}

/* --------------------------------------------------------------------------------------------- */
/*                  Function to receive serial data                                              */
/* --------------------------------------------------------------------------------------------- */

/**
 * uart_transmit - Receives serial data using the specified uart instance.
 * @param uint8_t 0-based UART ID
 * @return uint8_t data received through the uart.
 */
uint8_t uart_receive( uint8_t uart_id )
{
    if ( uart_id_is_valid(uart_id) )
	{
        switch ( uart_id )
		{
			case UART0_ID:
				while(false == UART0_ReceiverIsReady());
				return UART0_ReadByte();
				break;
			default:
				break;
		}
    }
}

/* end of uart_api.c */
/**   @}
 */

