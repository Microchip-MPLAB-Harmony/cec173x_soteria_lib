/*******************************************************************************
Interface definition of QMSPI0 PLIB.

 Company:
    Microchip Technology Inc.

 File Name:
    plib_qmspi0.h

 Summary:
    Interface definition of the Quad Serial Peripheral Interface Plib (QMSPI0).

 Description:
    This file defines the interface for the QMSPI0 Plib.
    It allows user to setup QMSPI0 and transfer data to and from slave devices
    attached.
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

#ifndef PLIB_QMSPI0_H // Guards against multiple inclusion
#define PLIB_QMSPI0_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

/* This section lists the other files that are included in this file.
*/

#include "device.h"
#include "plib_qmspi_common.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus // Provide C++ Compatibility
    extern "C" {
#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************

void QMSPI0_Initialize(void);
void QMSPI0_ChipSelectSetup(QMSPI_CHIP_SELECT chipSelect);
bool QMSPI0_TransferSetup (QMSPI_TRANSFER_SETUP *setup);
void QMSPI0_TapControlSet(uint16_t tap_vl, uint32_t tap_ctrl);
bool QMSPI0_Write(QMSPI_XFER_T *qmspiXfer, void* pTransmitData, size_t txSize);
bool QMSPI0_Read(QMSPI_XFER_T *qmspiXfer, void* pReceiveData, size_t rxSize);
bool QMSPI0_IsTransmitterBusy(void);
uint32_t QMSPI0_DMATransferWrite(QMSPI_DESCRIPTOR_XFER_T *qmspiDescXfer, void* pTransmitData, size_t txSize);
uint32_t QMSPI0_DMATransferRead(QMSPI_DESCRIPTOR_XFER_T *qmspiDescXfer, void* pReceiveData, size_t rxSize);
void QMSPI0_CallbackRegister(QMSPI_CALLBACK callback, uintptr_t context);
// DOM-IGNORE-BEGIN
#ifdef __cplusplus // Provide C++ Compatibility
}
#endif
// DOM-IGNORE-END

#endif /* PLIB_QMSPI0_H */
