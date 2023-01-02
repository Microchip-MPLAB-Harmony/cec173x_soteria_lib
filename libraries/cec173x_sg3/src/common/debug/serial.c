/*----------------------------------------------------------------------------
 * Name:    Serial.c
 * Purpose: Low level serial routines
 * Note(s):
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2010-2013 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/
#include "common.h"
#include "hal/uart/uart_api.h"
#include "peripheral/uart/plib_uart_common.h"

/*----------------------------------------------------------------------------
  Initialize UART pins, Baudrate
  - MMCR_8b(UART_LINE_CONTROL)  = 0x83;             // N 8 1
  - MMCR_8b(UART_DIVISOR_LATCH_1)=BaudRate[idx];    // BaudRateDiv.
  - MMCR_8b(UART_LINE_CONTROL)  = 0x03;             // Clr DLAB
 *----------------------------------------------------------------------------*/
void SER_init(void)
{
	uart_pins_init(DEBUG_UART_NUMBER);
	uart_hw_init(DEBUG_UART_NUMBER);
}

/*----------------------------------------------------------------------------
  Write character to Serial Port
 *----------------------------------------------------------------------------*/
int sendchar(char c)
{
    uart_transmit(DEBUG_UART_NUMBER, (uint8_t)c);
    return (1u);
}

/*----------------------------------------------------------------------------
  Read character from Sergpio.hial Port   (blocking read)
 *----------------------------------------------------------------------------*/
int getkey(void)
{
    return (int) (uart_receive(DEBUG_UART_NUMBER));
}