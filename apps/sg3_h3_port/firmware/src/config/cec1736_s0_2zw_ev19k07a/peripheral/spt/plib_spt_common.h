/*******************************************************************************
  SPI Peripheral Target Peripheral Library Interface Header File

  Company
    Microchip Technology Inc.

  File Name
    plib_spt_common.h

  Summary
    SPT Peripheral Target peripheral library interface.

  Description
    This file defines the interface to the SPT Peripheral Target peripheral
    library. This library provides access to and control of the associated peripheral
    instance.

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

#ifndef PLIB_SPT_COMMON_H    // Guards against multiple inclusion
#define PLIB_SPT_COMMON_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

/*  This section lists the other files that are included in this file.
*/

#include <stddef.h>
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
/*  The following data type definitions are used by the functions in this
    interface and should be considered part it.
*/

typedef enum
{
    SPT_EC_INT_MEM_WR_DONE = 0x1,
	
SPT_EC_INT_TXF_UNFLW = 0x2000000,
	
SPT_EC_INT_OBF_FLG = 0x8000,
	
SPT_EC_INT_RXF_SIZE_ERR = 0x1000000,
	
SPT_EC_INT_TXF_FUL = 0x800,
	
SPT_EC_INT_OOL0_ERR = 0x80000,
	
SPT_EC_INT_POLL_HI = 0x40,
	
SPT_EC_INT_TXF_RST_DN = 0x40000,
	
SPT_EC_INT_DV_BUSY = 0x800000,
	
SPT_EC_INT_MEM_WR_BUSY = 0x8,
	
SPT_EC_INT_RXF_FUL = 0x200,
	
SPT_EC_INT_RXF_UNFLW = 0x8000000,
	
SPT_EC_INT_SREG_TRANS = 0x20,
	
SPT_EC_INT_RXF_RST_DN = 0x20000,
	
SPT_EC_INT_MEM_RD_BUSY = 0x10,
	
SPT_EC_INT_RXF_EMP = 0x100,
	
SPT_EC_INT_ARMBUS_ERR = 0x200000,
	
SPT_EC_INT_IBF_FLG = 0x4000,
	
SPT_EC_INT_MEM_RD_DONE = 0x2,
	
SPT_EC_INT_UNDEF_CMD_ERR = 0x400000,
	
SPT_EC_INT_RXF_OVRFLW = 0x10000000,
	
SPT_EC_INT_TXF_OVRFLW = 0x4000000,
	
SPT_EC_INT_TMCLK_CNT_ERR = 0x2000,
	
SPT_EC_INT_TXF_EMP = 0x400,
	
SPT_EC_INT_OOL1_ERR = 0x100000,
	
SPT_EC_INT_SPIM_RST_REQ = 0x10000,
	

}SPT_EC_INT;

typedef void (*SPT_CALLBACK)(uint32_t status, uintptr_t context);

typedef struct
{
    SPT_CALLBACK callback;
    uintptr_t context;
}SPT_CALLBACK_OBJ;


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

}

#endif
// DOM-IGNORE-END

#endif //PLIB_SPT_COMMON_H

/**
 End of File
*/
