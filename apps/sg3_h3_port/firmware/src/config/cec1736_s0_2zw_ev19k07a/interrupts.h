/*******************************************************************************
 System Interrupts File

  Company:
    Microchip Technology Inc.

  File Name:
    interrupt.h

  Summary:
    Interrupt vectors mapping

  Description:
    This file contains declarations of device vectors used by Harmony 3
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
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

#ifndef INTERRUPTS_H
#define INTERRUPTS_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>



// *****************************************************************************
// *****************************************************************************
// Section: Handler Routines
// *****************************************************************************
// *****************************************************************************

void Reset_Handler (void);
void NonMaskableInt_Handler (void);
void HardFault_Handler (void);
void MemoryManagement_Handler (void);
void BusFault_Handler (void);
void UsageFault_Handler (void);
void DebugMonitor_Handler (void);
void GPIO140_GRP_InterruptHandler (void);
void GPIO107_GRP_InterruptHandler (void);
void GPIO113_GRP_InterruptHandler (void);
void GPIO127_GRP_InterruptHandler (void);
void GPIO046_GRP_InterruptHandler (void);
void GPIO047_GRP_InterruptHandler (void);
void GPIO050_GRP_InterruptHandler (void);
void GPIO063_GRP_InterruptHandler (void);
void GPIO013_GRP_InterruptHandler (void);
void GPIO015_GRP_InterruptHandler (void);
void GPIO201_GRP_InterruptHandler (void);
void I2CSMB0_GRP_InterruptHandler (void);
void I2CSMB1_GRP_InterruptHandler (void);
void I2CSMB2_GRP_InterruptHandler (void);
void I2CSMB3_GRP_InterruptHandler (void);
void I2CSMB4_GRP_InterruptHandler (void);
void DMA_CH00_GRP_InterruptHandler (void);
void DMA_CH01_GRP_InterruptHandler (void);
void DMA_CH03_GRP_InterruptHandler (void);
void SPT0_GRP_InterruptHandler (void);
void QMSPI0_GRP_InterruptHandler (void);
void QMSPI1_GRP_InterruptHandler (void);
void SPT1_GRP_InterruptHandler (void);
void VTR1_PAD_MON_GRP_InterruptHandler (void);
void VTR2_PAD_MON_GRP_InterruptHandler (void);
void WDT_GRP_InterruptHandler (void);
void EMC_GRP_InterruptHandler (void);



#endif // INTERRUPTS_H
