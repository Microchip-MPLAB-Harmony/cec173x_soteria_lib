/*******************************************************************************
 Cortex-M device vectors file

  Company:
    Microchip Technology Inc.

  File Name:
    device_vectors.h

  Summary:
    Harmony3 device handler structure for cortex-M devices

  Description:
    This file contains Harmony3 device handler structure for cortex-M devices
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

#ifndef DEVICE_VECTORS_H
#define DEVICE_VECTORS_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

/* Function pointer type for vector handlers */
typedef void (*pfn_handler_t)(void);

/* Structure defining device vector types */
typedef struct H3DeviceVectorsTag
{
  /* Stack pointer */
  uint32_t* pvStack;

  /* CORTEX-M4 handlers */ 
  pfn_handler_t pfnReset_Handler;                   /* -15 Reset Vector, invoked on Power up and warm reset */
  pfn_handler_t pfnNonMaskableInt_Handler;          /* -14 Non maskable Interrupt, cannot be stopped or preempted */
  pfn_handler_t pfnHardFault_Handler;               /* -13 Hard Fault, all classes of Fault */
  pfn_handler_t pfnMemoryManagement_Handler;        /* -12 Memory Management, MPU mismatch, including Access Violation and No Match */
  pfn_handler_t pfnBusFault_Handler;                /* -11 Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory related Fault */
  pfn_handler_t pfnUsageFault_Handler;              /* -10 Usage Fault, i.e. Undef Instruction, Illegal State Transition */
  pfn_handler_t pfnReservedC9;
  pfn_handler_t pfnReservedC8;
  pfn_handler_t pfnReservedC7;
  pfn_handler_t pfnReservedC6;
  pfn_handler_t pfnSVCall_Handler;                  /* -5 System Service Call via SVC instruction */
  pfn_handler_t pfnDebugMonitor_Handler;            /* -4 Debug Monitor */
  pfn_handler_t pfnReservedC3;
  pfn_handler_t pfnPendSV_Handler;                  /* -2 Pendable request for system service */
  pfn_handler_t pfnSysTick_Handler;                 /* -1 System Tick Timer */

  /* Peripheral handlers */
  pfn_handler_t pfnGIRQ08_Handler;                  /* 0 GIRQ08 */
  pfn_handler_t pfnGIRQ09_Handler;                  /* 1 GIRQ09 */
  pfn_handler_t pfnGIRQ10_Handler;                  /* 2 GIRQ10 */
  pfn_handler_t pfnGIRQ11_Handler;                  /* 3 GIRQ11 */
  pfn_handler_t pfnGIRQ12_Handler;                  /* 4 GIRQ12 */
  pfn_handler_t pfnGIRQ13_Handler;                  /* 5 GIRQ13 */
  pfn_handler_t pfnGIRQ14_Handler;                  /* 6 GIRQ14 */
  pfn_handler_t pfnGIRQ15_Handler;                  /* 7 GIRQ15 */
  pfn_handler_t pfnGIRQ16_Handler;                  /* 8 GIRQ16 */
  pfn_handler_t pfnGIRQ17_Handler;                  /* 9 GIRQ17 */
  pfn_handler_t pfnGIRQ18_Handler;                  /* 10 GIRQ18 */
  pfn_handler_t pfnReserved11;
  pfn_handler_t pfnGIRQ20_Handler;                  /* 12 GIRQ20 */
  pfn_handler_t pfnGIRQ21_Handler;                  /* 13 GIRQ21 */
  pfn_handler_t pfnGIRQ23_Handler;                  /* 14 GIRQ23 */
  pfn_handler_t pfnGIRQ24_Handler;                  /* 15 GIRQ24 */
  pfn_handler_t pfnReserved16;
  pfn_handler_t pfnGIRQ26_Handler;                  /* 17 GIRQ26 */
  pfn_handler_t pfnReserved18;
  pfn_handler_t pfnReserved19;
  pfn_handler_t pfnI2CSMB0_Handler;                 /* 20 I2CSMB0 */
  pfn_handler_t pfnI2CSMB1_Handler;                 /* 21 I2CSMB1 */
  pfn_handler_t pfnI2CSMB2_Handler;                 /* 22 I2CSMB2 */
  pfn_handler_t pfnI2CSMB3_Handler;                 /* 23 I2CSMB3 */
  pfn_handler_t pfnDMA_CH00_Handler;                /* 24 DMA_CH00 */
  pfn_handler_t pfnDMA_CH01_Handler;                /* 25 DMA_CH01 */
  pfn_handler_t pfnDMA_CH02_Handler;                /* 26 DMA_CH02 */
  pfn_handler_t pfnDMA_CH03_Handler;                /* 27 DMA_CH03 */
  pfn_handler_t pfnDMA_CH04_Handler;                /* 28 DMA_CH04 */
  pfn_handler_t pfnDMA_CH05_Handler;                /* 29 DMA_CH05 */
  pfn_handler_t pfnDMA_CH06_Handler;                /* 30 DMA_CH06 */
  pfn_handler_t pfnDMA_CH07_Handler;                /* 31 DMA_CH07 */
  pfn_handler_t pfnDMA_CH08_Handler;                /* 32 DMA_CH08 */
  pfn_handler_t pfnDMA_CH09_Handler;                /* 33 DMA_CH09 */
  pfn_handler_t pfnReserved34;
  pfn_handler_t pfnReserved35;
  pfn_handler_t pfnReserved36;
  pfn_handler_t pfnReserved37;
  pfn_handler_t pfnReserved38;
  pfn_handler_t pfnReserved39;
  pfn_handler_t pfnUART0_Handler;                   /* 40 UART0 */
  pfn_handler_t pfnReserved41;
  pfn_handler_t pfnReserved42;
  pfn_handler_t pfnReserved43;
  pfn_handler_t pfnReserved44;
  pfn_handler_t pfnReserved45;
  pfn_handler_t pfnReserved46;
  pfn_handler_t pfnReserved47;
  pfn_handler_t pfnReserved48;
  pfn_handler_t pfnReserved49;
  pfn_handler_t pfnReserved50;
  pfn_handler_t pfnReserved51;
  pfn_handler_t pfnReserved52;
  pfn_handler_t pfnReserved53;
  pfn_handler_t pfnReserved54;
  pfn_handler_t pfnReserved55;
  pfn_handler_t pfnReserved56;
  pfn_handler_t pfnReserved57;
  pfn_handler_t pfnReserved58;
  pfn_handler_t pfnReserved59;
  pfn_handler_t pfnReserved60;
  pfn_handler_t pfnReserved61;
  pfn_handler_t pfnReserved62;
  pfn_handler_t pfnReserved63;
  pfn_handler_t pfnReserved64;
  pfn_handler_t pfnPKE_ERR_Handler;                 /* 65 PKE_ERR */
  pfn_handler_t pfnPKE_END_Handler;                 /* 66 PKE_END */
  pfn_handler_t pfnRNG_Handler;                     /* 67 RNG */
  pfn_handler_t pfnAES_Handler;                     /* 68 AES */
  pfn_handler_t pfnHASH_Handler;                    /* 69 HASH */
  pfn_handler_t pfnReserved70;
  pfn_handler_t pfnReserved71;
  pfn_handler_t pfnReserved72;
  pfn_handler_t pfnReserved73;
  pfn_handler_t pfnReserved74;
  pfn_handler_t pfnReserved75;
  pfn_handler_t pfnReserved76;
  pfn_handler_t pfnReserved77;
  pfn_handler_t pfnReserved78;
  pfn_handler_t pfnReserved79;
  pfn_handler_t pfnReserved80;
  pfn_handler_t pfnReserved81;
  pfn_handler_t pfnReserved82;
  pfn_handler_t pfnLED0_Handler;                    /* 83 LED0 */
  pfn_handler_t pfnLED1_Handler;                    /* 84 LED1 */
  pfn_handler_t pfnReserved85;
  pfn_handler_t pfnReserved86;
  pfn_handler_t pfnReserved87;
  pfn_handler_t pfnReserved88;
  pfn_handler_t pfnReserved89;
  pfn_handler_t pfnSPT0_Handler;                    /* 90 SPT0 */
  pfn_handler_t pfnQMSPI0_Handler;                  /* 91 QMSPI0 */
  pfn_handler_t pfnQMSPI1_Handler;                  /* 92 QMSPI1 */
  pfn_handler_t pfnReserved93;
  pfn_handler_t pfnReserved94;
  pfn_handler_t pfnReserved95;
  pfn_handler_t pfnReserved96;
  pfn_handler_t pfnReserved97;
  pfn_handler_t pfnReserved98;
  pfn_handler_t pfnReserved99;
  pfn_handler_t pfnReserved100;
  pfn_handler_t pfnReserved101;
  pfn_handler_t pfnReserved102;
  pfn_handler_t pfnReserved103;
  pfn_handler_t pfnReserved104;
  pfn_handler_t pfnReserved105;
  pfn_handler_t pfnReserved106;
  pfn_handler_t pfnReserved107;
  pfn_handler_t pfnReserved108;
  pfn_handler_t pfnReserved109;
  pfn_handler_t pfnReserved110;
  pfn_handler_t pfnRTMR_Handler;                    /* 111 RTMR */
  pfn_handler_t pfnHTMR0_Handler;                   /* 112 HTMR0 */
  pfn_handler_t pfnHTMR1_Handler;                   /* 113 HTMR1 */
  pfn_handler_t pfnReserved114;
  pfn_handler_t pfnReserved115;
  pfn_handler_t pfnReserved116;
  pfn_handler_t pfnReserved117;
  pfn_handler_t pfnReserved118;
  pfn_handler_t pfnReserved119;
  pfn_handler_t pfnReserved120;
  pfn_handler_t pfnReserved121;
  pfn_handler_t pfnReserved122;
  pfn_handler_t pfnReserved123;
  pfn_handler_t pfnReserved124;
  pfn_handler_t pfnReserved125;
  pfn_handler_t pfnReserved126;
  pfn_handler_t pfnReserved127;
  pfn_handler_t pfnReserved128;
  pfn_handler_t pfnReserved129;
  pfn_handler_t pfnReserved130;
  pfn_handler_t pfnReserved131;
  pfn_handler_t pfnReserved132;
  pfn_handler_t pfnReserved133;
  pfn_handler_t pfnEMC_Handler;                     /* 134 EMC */
  pfn_handler_t pfnReserved135;
  pfn_handler_t pfnReserved136;
  pfn_handler_t pfnReserved137;
  pfn_handler_t pfnReserved138;
  pfn_handler_t pfnReserved139;
  pfn_handler_t pfnTIMER32_0_Handler;               /* 140 TIMER32_0 */
  pfn_handler_t pfnTIMER32_1_Handler;               /* 141 TIMER32_1 */
  pfn_handler_t pfnReserved142;
  pfn_handler_t pfnReserved143;
  pfn_handler_t pfnReserved144;
  pfn_handler_t pfnReserved145;
  pfn_handler_t pfnCCT_Handler;                     /* 146 CCT */
  pfn_handler_t pfnCCT_CAP0_Handler;                /* 147 CCT_CAP0 */
  pfn_handler_t pfnCCT_CAP1_Handler;                /* 148 CCT_CAP1 */
  pfn_handler_t pfnCCT_CAP2_Handler;                /* 149 CCT_CAP2 */
  pfn_handler_t pfnCCT_CAP3_Handler;                /* 150 CCT_CAP3 */
  pfn_handler_t pfnCCT_CAP4_Handler;                /* 151 CCT_CAP4 */
  pfn_handler_t pfnCCT_CAP5_Handler;                /* 152 CCT_CAP5 */
  pfn_handler_t pfnCCT_CMP0_Handler;                /* 153 CCT_CMP0 */
  pfn_handler_t pfnCCT_CMP1_Handler;                /* 154 CCT_CMP1 */
  pfn_handler_t pfnReserved155;
  pfn_handler_t pfnReserved156;
  pfn_handler_t pfnReserved157;
  pfn_handler_t pfnI2CSMB4_Handler;                 /* 158 I2CSMB4 */
  pfn_handler_t pfnReserved159;
  pfn_handler_t pfnReserved160;
  pfn_handler_t pfnReserved161;
  pfn_handler_t pfnReserved162;
  pfn_handler_t pfnReserved163;
  pfn_handler_t pfnReserved164;
  pfn_handler_t pfnReserved165;
  pfn_handler_t pfnReserved166;
  pfn_handler_t pfnReserved167;
  pfn_handler_t pfnReserved168;
  pfn_handler_t pfnReserved169;
  pfn_handler_t pfnReserved170;
  pfn_handler_t pfnWDT_Handler;                     /* 171 WDT */
  pfn_handler_t pfnReserved172;
  pfn_handler_t pfnReserved173;
  pfn_handler_t pfnCLK_MON_Handler;                 /* 174 CLK_MON */
  pfn_handler_t pfnReserved175;
  pfn_handler_t pfnReserved176;
  pfn_handler_t pfnReserved177;
  pfn_handler_t pfnReserved178;
  pfn_handler_t pfnReserved179;
  pfn_handler_t pfnReserved180;
  pfn_handler_t pfnSWI0_Handler;                    /* 181 SWI0 */
  pfn_handler_t pfnSWI1_Handler;                    /* 182 SWI1 */
  pfn_handler_t pfnSWI2_Handler;                    /* 183 SWI2 */
  pfn_handler_t pfnSWI3_Handler;                    /* 184 SWI3 */
  pfn_handler_t pfnIMSPI_Handler;                   /* 185 IMSPI */
  pfn_handler_t pfnReserved186;
  pfn_handler_t pfnSPT1_Handler;                    /* 187 SPT1 */
  pfn_handler_t pfnSPIMON0_VLTN_Handler;            /* 188 SPIMON0_VLTN */
  pfn_handler_t pfnSPIMON0_MTMON_Handler;           /* 189 SPIMON0_MTMON */
  pfn_handler_t pfnSPIMON0_LTMON_Handler;           /* 190 SPIMON0_LTMON */
  pfn_handler_t pfnSPIMON1_VLTN_Handler;            /* 191 SPIMON1_VLTN */
  pfn_handler_t pfnSPIMON1_MTMON_Handler;           /* 192 SPIMON1_MTMON */
  pfn_handler_t pfnSPIMON1_LTMON_Handler;           /* 193 SPIMON1_LTMON */
  pfn_handler_t pfnVTR1_PAD_MON_Handler;            /* 194 VTR1_PAD_MON */
  pfn_handler_t pfnVTR2_PAD_MON_Handler;            /* 195 VTR2_PAD_MON */
}H3DeviceVectors;

#endif //DEVICE_VECTORS_H
