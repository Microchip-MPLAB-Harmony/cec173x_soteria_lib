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
$Revision: #2 $ 
$DateTime: 2023/01/02 04:42:23 $ 
$Author: i64652 $
Last Change:    None
******************************************************************************/
/** @file uart_api.h
* \brief UART Peripheral Header File
* \author pramans
* 
* This file is the header file for UART Peripheral 
******************************************************************************/

/** @defgroup UART
 *  @{
 */
 
#ifndef UART_H
#define UART_H


#ifdef __cplusplus
extern "C" {
#endif
    
#define UART0_ID            0u
#define UART1_ID            1u

#define BAUD_4800           4800U
#define BAUD_7200           7200U
#define BAUD_9600           9600U
#define BAUD_19200          19200U
#define BAUD_38400          38400U
#define BAUD_57600          57600U
#define BAUD_115200         115200U

/* --------------------------------------------------------------------------------------------- */
/*                  Function to initialize the UART pins                                         */
/* --------------------------------------------------------------------------------------------- */

/**
 * uart_pins_init - Initializes the gpio pin that are associated
 * with the specified UART instance for UART functionality.
 * @param uint8_t 0-based UART ID
 * @return None.
 */
void uart_pins_init( uint8_t uart_id );

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
void uart_hw_init( uint8_t uart_id );

/* --------------------------------------------------------------------------------------------- */
/*                  Function to configure the protocol & interrupt parameters                    */
/* --------------------------------------------------------------------------------------------- */

/**
 * uart_protocol_init - Initializes the serial protocol and interrupt parameters.
 * @param uint8_t 0-based UART ID
 * @param uint16_t desired baud rate value
 * @param uint8_t word length setting
 * @param uint8_t stop bit setting
 * @param uint8_t parity bit setting
 * @param uint8_t interrupt source type that is to be enabled
 * @return None
 * @note In case interrupts are not being used; keep the interrupt source
 *       type parameter value as UART_INT_DISABLED.
 */
void uart_protocol_init( uint8_t uart_id, uint32_t baud, uint8_t wrd_len, uint8_t stp_bit, uint8_t parity_type);

/* --------------------------------------------------------------------------------------------- */
/*                  Function to transmit serial data                                             */
/* --------------------------------------------------------------------------------------------- */

/**
 * uart_transmit - Transmits serial data using the specified uart instance.
 * @param uint8_t 0-based UART ID
 * @param uint8_t data to be transmitted
 * @return None.
 */
void uart_transmit( uint8_t uart_id, uint8_t data );

/* --------------------------------------------------------------------------------------------- */
/*                  Function to receive serial data                                              */
/* --------------------------------------------------------------------------------------------- */

/**
 * uart_transmit - Receives serial data using the specified uart instance.
 * @param uint8_t 0-based UART ID
 * @return uint8_t data received through the uart.
 */
uint8_t uart_receive( uint8_t uart_id );

#ifdef __cplusplus
}
#endif

#endif

/* end of uart_api.h */
/**   @}
 */
