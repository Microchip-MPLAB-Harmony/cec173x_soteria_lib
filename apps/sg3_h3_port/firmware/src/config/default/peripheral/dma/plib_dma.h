/*******************************************************************************
  DMAC Peripheral Library Interface Header File

  Company:
    Microchip Technology Inc.

  File Name:
    plib_dma.h

  Summary:
    DMA peripheral library interface.

  Description:
    This file defines the interface to the DMAC peripheral library. This
    library provides access to and control of the DMAC controller.

  Remarks:
    None.

*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2021 Microchip Technology Inc. and its subsidiaries.
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

#ifndef PLIB_DMA_H    // Guards against multiple inclusion
#define PLIB_DMA_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

/*  This section lists the other files that are included in this file.
*/
#include <device.h>
#include <string.h>
#include <stdbool.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************

typedef enum
{
    /* DMA Channel 0 */
    DMA_CHANNEL_0 = 0,
    /* DMA Channel 1 */
    DMA_CHANNEL_1 = 1,
    /* DMA Channel 2 */
    DMA_CHANNEL_2 = 2,
    /* DMA Channel 3 */
    DMA_CHANNEL_3 = 3,
    /* DMA Channel 4 */
    DMA_CHANNEL_4 = 4,
    /* DMA Channel 5 */
    DMA_CHANNEL_5 = 5,
    /* DMA Channel 6 */
    DMA_CHANNEL_6 = 6,
    /* DMA Channel 7 */
    DMA_CHANNEL_7 = 7,
    /* DMA Channel 8 */
    DMA_CHANNEL_8 = 8,
    /* DMA Channel 9 */
    DMA_CHANNEL_9 = 9,
} DMA_CHANNEL;

#define BUSY_TIMEOUT_COUNTER    8000U

void DMA_Initialize( void );
void DMA_MemIncrCfg(DMA_CHANNEL channel, uint8_t mem_incr_sts);
void DMA_ChannelAbort(DMA_CHANNEL channel);
void DMA_Activate(uint8_t activate);
void DMA_SoftReset(void);
void DMA_Enable(void);
void DMA_Disable(void);
void DMA_ChannelActivate(DMA_CHANNEL channel);
void DMA_ChannelDeactivate(DMA_CHANNEL channel);
void DMA_Stop(DMA_CHANNEL channel);
void DMA_Start(DMA_CHANNEL channel);
uint8_t DMA_WaitTillFree(DMA_CHANNEL channel);
void DMA_SetupTx(DMA_CHANNEL channel, const uint8_t device, const uint32_t dataAHBAddress, uint32_t dma_buffer_tx, const uint8_t transfer_bytes_count);
void DMA_SetupRx(DMA_CHANNEL channel, const uint8_t device, const uint32_t dataAHBAddress, uint32_t dma_buffer_rx, const uint8_t transfer_bytes_count, const bool incrMemAddrFlag);
void DMA_SwitchTxToRx(DMA_CHANNEL channel, const uint8_t device, const uint32_t dataAHBAddress);
uint8_t DMA_GetDeviceId(const uint8_t device_name, const uint8_t device_instance);
void DMA_EnableInterrupt(DMA_CHANNEL channel);
void DMA_DisableInterrupt(DMA_CHANNEL channel);



// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
// DOM-IGNORE-END

#endif //PLIB_DMA_H
