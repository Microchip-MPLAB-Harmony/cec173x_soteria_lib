/*****************************************************************************
* � 2019 Microchip Technology Inc. and its subsidiaries.
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
*****************************************************************************/

/** @file platform.h
 *MEC1322 BootROM platform/cpu abstractions
 */
/** @defgroup bootrom include
 */

#ifndef _PLATFORM_H
#define _PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG_UART_NUMBER 0
#define DEBUG_UART_BAUD BAUD_115200

/* macro for defining MMCR register */
/* add MMCRARRAY() & EXTERNMMCRARRAY() */
/* General Constants */

#define BIT_n_MASK(n)	(1ul << (n))
#define BIT_0_MASK    (1<<0)
#define BIT_1_MASK    (1<<1)
#define BIT_2_MASK    (1<<2)
#define BIT_3_MASK    (1<<3)
#define BIT_4_MASK    (1<<4)
#define BIT_5_MASK    (1<<5)
#define BIT_6_MASK    (1<<6)
#define BIT_7_MASK    (1<<7)
#define BIT_8_MASK    ((uint32_t)1<<8)
#define BIT_9_MASK    ((uint32_t)1<<9)
#define BIT_10_MASK   ((uint32_t)1<<10)
#define BIT_11_MASK   ((uint32_t)1<<11)
#define BIT_12_MASK   ((uint32_t)1<<12)
#define BIT_13_MASK   ((uint32_t)1<<13)
#define BIT_14_MASK   ((uint32_t)1<<14)
#define BIT_15_MASK   ((uint32_t)1<<15)
#define BIT_16_MASK     ((uint32_t)1<<16)
#define BIT_17_MASK     ((uint32_t)1<<17)
#define BIT_18_MASK     ((uint32_t)1<<18)
#define BIT_19_MASK     ((uint32_t)1<<19)
#define BIT_20_MASK     ((uint32_t)1<<20)
#define BIT_21_MASK     ((uint32_t)1<<21)
#define BIT_22_MASK     ((uint32_t)1<<22)
#define BIT_23_MASK     ((uint32_t)1<<23)
#define BIT_24_MASK     ((uint32_t)1<<24)
#define BIT_25_MASK     ((uint32_t)1<<25)
#define BIT_26_MASK     ((uint32_t)1<<26)
#define BIT_27_MASK     ((uint32_t)1<<27)
#define BIT_28_MASK     ((uint32_t)1<<28)
#define BIT_29_MASK     ((uint32_t)1<<29)
#define BIT_30_MASK     ((uint32_t)1<<30)
#define BIT_31_MASK     ((uint32_t)1<<31)

#define MAX_IRQn (PERIPH_MAX_IRQn + 1)

void gpio_register_isr_handlers(void);
void qmspi_register_isr_handlers(void);

#ifdef __cplusplus
}
#endif

#endif // #ifndef _PLATFORM_H
/**   @}
 */
