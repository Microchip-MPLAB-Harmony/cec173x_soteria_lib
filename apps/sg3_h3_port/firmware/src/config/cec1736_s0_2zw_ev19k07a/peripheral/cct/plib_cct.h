/*******************************************************************************
  Data Type definition of CCT Timer PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_cct_tmr.h

  Summary:
    Data Type definition of the CCT Timer Peripheral Interface Plib.

  Description:
    This file defines the Data Types for the CCT Timer Plib.

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

#ifndef PLIB_CCT_H
#define PLIB_CCT_H

#include <stddef.h>
#include <stdint.h>
#include "device.h"
#include "plib_cct_common.h"

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
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
void CCT_Initialize(void);

void CCT_TimerActivate( void );

void CCT_TimerDeActivate( void );

void CCT_FreeRunningTimerStart( void );

void CCT_FreeRunningTimerStop( void );

void CCT_FreeRunningTimerReset( void );

uint32_t CCT_FreeRunningTimerGet( void );

void CCT_FreeRunningTimerSet( uint32_t count);

void CCT_FreqDivSet( uint32_t div );

uint32_t CCT_FrequencyGet(void);




uint32_t CCT_CompareChannel0PeriodGet( void );

void CCT_CompareChannel0PeriodSet( uint32_t period );

void CCT_CompareChannel0Enable( void );

void CCT_CompareChannel0Disable( void );

void CCT_CompareChannel0OutputSet( void );

void CCT_CompareChannel0OutputClear( void );


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    }
#endif
// DOM-IGNORE-END

#endif /* PLIB_CCT_H */
