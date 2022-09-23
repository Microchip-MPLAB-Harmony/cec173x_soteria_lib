/***************************************************************************** 
* Copyright 2019 Microchip Technology Inc. and its subsidiaries.                       
* You may use this software and any derivatives exclusively with               
* Microchip products.                                                          
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP 'AS IS'.                              
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
*****************************************************************************/ 
                                                                               
/** @file platform_serial_flash.h
 * flash_program 
 */
/** @defgroup flash_program 
 */
  
#ifndef PLATFORM_SERIAL_FLASH_H                                                          
#define PLATFORM_SERIAL_FLASH_H 

#define SPI_DONE_OK 0

#define SPI_4BYTE_ADDR_REQUIRED(addr, len) ((addr + len - 1) & 0xFF000000ul)
#define SPI_GET_PORT(cs) ((cs >> 2) & 0x3)

/** CHIP SELECT */
typedef enum {
    SELECT,
    DESELECT
}CS_SPI;

/******************************************************************************/
/** platform_serial_flash_read_dma_nonblocking_buf_not_fill();
* read the content from the SPI interface without using DMA interface. 
* Read to same buffer Location
*  Supported SPI Read commands are 
*       0 =  Read data Single Output(0x03)
*       1 =  Fast Read Single Output(0x0B)
*       2 =  Fast Read Dual Output(0x3B)
*       3 =  Fast Read Quad Output(0x6B)
* @param spi_addr - SPI address to start reading from
* @param mem_addr - Memory address to copy data
* @param data_len - Number of bytes to read
* @param mode - Read mode
* @param *qmspi_status - Pointer to store qmspi status value
* @param spi_select - spi select
* @param channel - channel 0 or 1
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t platform_serial_flash_read_dma_nonblocking_no_fill(uint32_t spi_addr, uint32_t mem_addr, uint32_t data_len, uint8_t mode, uint32_t *qmspi_status,  uint8_t spi_select, uint8_t channel);

extern void chipSelectSPI(uint8_t cs, CS_SPI state);

#endif
/* end platform_serial_flash.h */                                                         
/**   @}                                                                       
 */ 
