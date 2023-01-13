/*******************************************************************************
 System Interrupts File

  Company:
    Microchip Technology Inc.

  File Name:
    interrupt.c

  Summary:
    Interrupt vectors mapping

  Description:
    This file maps all the interrupt vectors to their corresponding
    implementations. If a particular module interrupt is used, then its ISR
    definition can be found in corresponding PLIB source file. If a module
    interrupt is not used, then its ISR implementation is mapped to dummy
    handler.
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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "device_vectors.h"
#include "interrupts.h"
#include "definitions.h"


// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

/* MISRA C-2012 Rule 8.6 deviated below. Deviation record ID -  H3_MISRAC_2012_R_8_6_DR_1 */
extern uint32_t _stack;
extern const H3DeviceVectors exception_table;

extern void Dummy_Handler(void);

/* Brief default interrupt handler for unused IRQs.*/
void __attribute__((optimize("-O1"),section(".text.Dummy_Handler"),long_call, noreturn))Dummy_Handler(void)
{
#if defined(__DEBUG) || defined(__DEBUG_D) && defined(__XC32)
    __builtin_software_breakpoint();
#endif
    while (true)
    {
    }
}

/* MISRAC 2012 deviation block start */
/* MISRA C-2012 Rule 8.6 deviated 67 times.  Deviation record ID -  H3_MISRAC_2012_R_8_6_DR_1 */
/* Device vectors list dummy definition*/
extern void SVCall_Handler             ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void PendSV_Handler             ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void SysTick_Handler            ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void GIRQ15_Handler             ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void GIRQ16_Handler             ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void GIRQ17_Handler             ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void GIRQ20_Handler             ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void GIRQ23_Handler             ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void GIRQ24_Handler             ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void GIRQ26_Handler             ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void I2CSMB0_Handler            ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void I2CSMB1_Handler            ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void I2CSMB2_Handler            ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void I2CSMB3_Handler            ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void DMA_CH00_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void DMA_CH01_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void DMA_CH02_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void DMA_CH03_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void DMA_CH04_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void DMA_CH05_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void DMA_CH06_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void DMA_CH07_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void DMA_CH08_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void DMA_CH09_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void UART0_Handler              ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void PKE_ERR_Handler            ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void PKE_END_Handler            ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void RNG_Handler                ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void AES_Handler                ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void HASH_Handler               ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void LED0_Handler               ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void LED1_Handler               ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void SPT0_Handler               ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void QMSPI0_Handler             ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void QMSPI1_Handler             ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void RTMR_Handler               ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void HTMR0_Handler              ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void HTMR1_Handler              ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void EMC_Handler                ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void TIMER32_0_Handler          ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void TIMER32_1_Handler          ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void CCT_Handler                ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void CCT_CAP0_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void CCT_CAP1_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void CCT_CAP2_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void CCT_CAP3_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void CCT_CAP4_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void CCT_CAP5_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void CCT_CMP0_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void CCT_CMP1_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void I2CSMB4_Handler            ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void WDT_Handler                ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void CLK_MON_Handler            ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void SWI0_Handler               ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void SWI1_Handler               ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void SWI2_Handler               ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void SWI3_Handler               ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void IMSPI_Handler              ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void SPT1_Handler               ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void SPIMON0_VLTN_Handler       ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void SPIMON0_MTMON_Handler      ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void SPIMON0_LTMON_Handler      ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void SPIMON1_VLTN_Handler       ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void SPIMON1_MTMON_Handler      ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void SPIMON1_LTMON_Handler      ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void VTR1_PAD_MON_Handler       ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void VTR2_PAD_MON_Handler       ( void ) __attribute__((weak, alias("Dummy_Handler")));


/* MISRAC 2012 deviation block end */

/* Multiple handlers for vector */
static void GIRQ08_Handler( void )
{
    GPIO140_GRP_InterruptHandler();
}

static void GIRQ09_Handler( void )
{
    GPIO107_GRP_InterruptHandler();
    GPIO113_GRP_InterruptHandler();
    GPIO127_GRP_InterruptHandler();
}

static void GIRQ10_Handler( void )
{
    GPIO046_GRP_InterruptHandler();
    GPIO047_GRP_InterruptHandler();
    GPIO050_GRP_InterruptHandler();
    GPIO063_GRP_InterruptHandler();
}

static void GIRQ11_Handler( void )
{
    GPIO013_GRP_InterruptHandler();
    GPIO015_GRP_InterruptHandler();
}

static void GIRQ12_Handler( void )
{
    GPIO201_GRP_InterruptHandler();
}

static void GIRQ13_Handler( void )
{
    I2CSMB0_GRP_InterruptHandler();
    I2CSMB1_GRP_InterruptHandler();
    I2CSMB2_GRP_InterruptHandler();
    I2CSMB3_GRP_InterruptHandler();
    I2CSMB4_GRP_InterruptHandler();
}

static void GIRQ14_Handler( void )
{
    DMA_CH00_GRP_InterruptHandler();
    DMA_CH01_GRP_InterruptHandler();
    DMA_CH02_GRP_InterruptHandler();
    DMA_CH03_GRP_InterruptHandler();
    DMA_CH04_GRP_InterruptHandler();
    DMA_CH05_GRP_InterruptHandler();
    DMA_CH06_GRP_InterruptHandler();
    DMA_CH07_GRP_InterruptHandler();
    DMA_CH08_GRP_InterruptHandler();
    DMA_CH09_GRP_InterruptHandler();
}

static void GIRQ18_Handler( void )
{
    QMSPI0_GRP_InterruptHandler();
    QMSPI1_GRP_InterruptHandler();
}

static void GIRQ21_Handler( void )
{
    WDT_GRP_InterruptHandler();
}




__attribute__ ((section(".vectors")))
const H3DeviceVectors exception_table=
{
    /* Configure Initial Stack Pointer, using linker-generated symbols */
    .pvStack = &_stack,

    .pfnReset_Handler              = Reset_Handler,
    .pfnNonMaskableInt_Handler     = NonMaskableInt_Handler,
    .pfnHardFault_Handler          = HardFault_Handler,
    .pfnMemoryManagement_Handler   = MemoryManagement_Handler,
    .pfnBusFault_Handler           = BusFault_Handler,
    .pfnUsageFault_Handler         = UsageFault_Handler,
    .pfnSVCall_Handler             = SVCall_Handler,
    .pfnDebugMonitor_Handler       = DebugMonitor_Handler,
    .pfnPendSV_Handler             = PendSV_Handler,
    .pfnSysTick_Handler            = SysTick_Handler,
    .pfnGIRQ08_Handler             = GIRQ08_Handler,
    .pfnGIRQ09_Handler             = GIRQ09_Handler,
    .pfnGIRQ10_Handler             = GIRQ10_Handler,
    .pfnGIRQ11_Handler             = GIRQ11_Handler,
    .pfnGIRQ12_Handler             = GIRQ12_Handler,
    .pfnGIRQ13_Handler             = GIRQ13_Handler,
    .pfnGIRQ14_Handler             = GIRQ14_Handler,
    .pfnGIRQ15_Handler             = GIRQ15_Handler,
    .pfnGIRQ16_Handler             = GIRQ16_Handler,
    .pfnGIRQ17_Handler             = GIRQ17_Handler,
    .pfnGIRQ18_Handler             = GIRQ18_Handler,
    .pfnGIRQ20_Handler             = GIRQ20_Handler,
    .pfnGIRQ21_Handler             = GIRQ21_Handler,
    .pfnGIRQ23_Handler             = GIRQ23_Handler,
    .pfnGIRQ24_Handler             = GIRQ24_Handler,
    .pfnGIRQ26_Handler             = GIRQ26_Handler,
    .pfnI2CSMB0_Handler            = I2CSMB0_Handler,
    .pfnI2CSMB1_Handler            = I2CSMB1_Handler,
    .pfnI2CSMB2_Handler            = I2CSMB2_Handler,
    .pfnI2CSMB3_Handler            = I2CSMB3_Handler,
    .pfnDMA_CH00_Handler           = DMA_CH00_Handler,
    .pfnDMA_CH01_Handler           = DMA_CH01_Handler,
    .pfnDMA_CH02_Handler           = DMA_CH02_Handler,
    .pfnDMA_CH03_Handler           = DMA_CH03_Handler,
    .pfnDMA_CH04_Handler           = DMA_CH04_Handler,
    .pfnDMA_CH05_Handler           = DMA_CH05_Handler,
    .pfnDMA_CH06_Handler           = DMA_CH06_Handler,
    .pfnDMA_CH07_Handler           = DMA_CH07_Handler,
    .pfnDMA_CH08_Handler           = DMA_CH08_Handler,
    .pfnDMA_CH09_Handler           = DMA_CH09_Handler,
    .pfnUART0_Handler              = UART0_Handler,
    .pfnPKE_ERR_Handler            = PKE_ERR_Handler,
    .pfnPKE_END_Handler            = PKE_END_Handler,
    .pfnRNG_Handler                = RNG_Handler,
    .pfnAES_Handler                = AES_Handler,
    .pfnHASH_Handler               = HASH_Handler,
    .pfnLED0_Handler               = LED0_Handler,
    .pfnLED1_Handler               = LED1_Handler,
    .pfnSPT0_Handler               = SPT0_Handler,
    .pfnQMSPI0_Handler             = QMSPI0_Handler,
    .pfnQMSPI1_Handler             = QMSPI1_Handler,
    .pfnRTMR_Handler               = RTMR_Handler,
    .pfnHTMR0_Handler              = HTMR0_Handler,
    .pfnHTMR1_Handler              = HTMR1_Handler,
    .pfnEMC_Handler                = EMC_Handler,
    .pfnTIMER32_0_Handler          = TIMER32_0_Handler,
    .pfnTIMER32_1_Handler          = TIMER32_1_Handler,
    .pfnCCT_Handler                = CCT_Handler,
    .pfnCCT_CAP0_Handler           = CCT_CAP0_Handler,
    .pfnCCT_CAP1_Handler           = CCT_CAP1_Handler,
    .pfnCCT_CAP2_Handler           = CCT_CAP2_Handler,
    .pfnCCT_CAP3_Handler           = CCT_CAP3_Handler,
    .pfnCCT_CAP4_Handler           = CCT_CAP4_Handler,
    .pfnCCT_CAP5_Handler           = CCT_CAP5_Handler,
    .pfnCCT_CMP0_Handler           = CCT_CMP0_Handler,
    .pfnCCT_CMP1_Handler           = CCT_CMP1_Handler,
    .pfnI2CSMB4_Handler            = I2CSMB4_Handler,
    .pfnWDT_Handler                = WDT_Handler,
    .pfnCLK_MON_Handler            = CLK_MON_Handler,
    .pfnSWI0_Handler               = SWI0_Handler,
    .pfnSWI1_Handler               = SWI1_Handler,
    .pfnSWI2_Handler               = SWI2_Handler,
    .pfnSWI3_Handler               = SWI3_Handler,
    .pfnIMSPI_Handler              = IMSPI_Handler,
    .pfnSPT1_Handler               = SPT1_Handler,
    .pfnSPIMON0_VLTN_Handler       = SPIMON0_VLTN_Handler,
    .pfnSPIMON0_MTMON_Handler      = SPIMON0_MTMON_Handler,
    .pfnSPIMON0_LTMON_Handler      = SPIMON0_LTMON_Handler,
    .pfnSPIMON1_VLTN_Handler       = SPIMON1_VLTN_Handler,
    .pfnSPIMON1_MTMON_Handler      = SPIMON1_MTMON_Handler,
    .pfnSPIMON1_LTMON_Handler      = SPIMON1_LTMON_Handler,
    .pfnVTR1_PAD_MON_Handler       = VTR1_PAD_MON_Handler,
    .pfnVTR2_PAD_MON_Handler       = VTR2_PAD_MON_Handler,


};

/*******************************************************************************
 End of File
*/
