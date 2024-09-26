/****************************************************************************
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
****************************************************************************/

#include "common.h"

extern void GPIO013_ISR(void);
extern void GPIO015_ISR(void);
extern void GPIO046_ISR(void);
extern void GPIO047_ISR(void);
extern void GPIO050_ISR(void);
extern void GPIO063_ISR(void);
extern void GPIO107_ISR(void);
extern void GPIO113_ISR(void);
extern void GPIO127_ISR(void);
extern void GPIO140_ISR(void);
extern void GPIO201_ISR(void);
extern void WDT_ISR(void);
extern void sb_vtr1_pad_mon_isr(void);
extern void sb_vtr2_pad_mon_isr(void);
extern void qmspi_isr(uint8_t channel);
extern void smb_isr(void);
extern void smb_dma_isr(void);
extern void sb_monitor_envmon_isr();
static void I2CSMB_GRP_InterruptHandler (void);
static void DMA_GRP_InterruptHandler (void);
typedef void (*EMC_CALLBACK)(void);
extern void EMC_CallbackRegister( EMC_CALLBACK callback);

void I2CSMB0_GRP_InterruptHandler       ( void ) __attribute__((alias("I2CSMB_GRP_InterruptHandler")));
void I2CSMB1_GRP_InterruptHandler       ( void ) __attribute__((alias("I2CSMB_GRP_InterruptHandler")));
void I2CSMB2_GRP_InterruptHandler       ( void ) __attribute__((alias("I2CSMB_GRP_InterruptHandler")));
void I2CSMB3_GRP_InterruptHandler       ( void ) __attribute__((alias("I2CSMB_GRP_InterruptHandler")));
void I2CSMB4_GRP_InterruptHandler       ( void ) __attribute__((alias("I2CSMB_GRP_InterruptHandler")));
 
void DMA_CH00_GRP_InterruptHandler       ( void ) __attribute__((alias("DMA_GRP_InterruptHandler")));
void DMA_CH01_GRP_InterruptHandler       ( void ) __attribute__((alias("DMA_GRP_InterruptHandler")));
void DMA_CH02_GRP_InterruptHandler       ( void ) __attribute__((alias("DMA_GRP_InterruptHandler")));
void DMA_CH03_GRP_InterruptHandler       ( void ) __attribute__((alias("DMA_GRP_InterruptHandler")));
void DMA_CH04_GRP_InterruptHandler       ( void ) __attribute__((alias("DMA_GRP_InterruptHandler")));
void DMA_CH05_GRP_InterruptHandler       ( void ) __attribute__((alias("DMA_GRP_InterruptHandler")));
void DMA_CH06_GRP_InterruptHandler       ( void ) __attribute__((alias("DMA_GRP_InterruptHandler")));
void DMA_CH07_GRP_InterruptHandler       ( void ) __attribute__((alias("DMA_GRP_InterruptHandler")));
void DMA_CH08_GRP_InterruptHandler       ( void ) __attribute__((alias("DMA_GRP_InterruptHandler")));
void DMA_CH09_GRP_InterruptHandler       ( void ) __attribute__((alias("DMA_GRP_InterruptHandler")));

static void gpio_isr(GPIO_PIN pin, uintptr_t context)
{
    switch(pin)
    {
        case GPIO_PIN_GPIO013:
            GPIO013_ISR();
        break;
        case GPIO_PIN_GPIO015:
            GPIO015_ISR();
        break;        
        case GPIO_PIN_GPIO046:
            GPIO046_ISR();
        break;
        case GPIO_PIN_GPIO047:
            GPIO047_ISR();
        break;
        case GPIO_PIN_GPIO050:
            GPIO050_ISR();
        break;
        case GPIO_PIN_GPIO063:
            GPIO063_ISR();
        break;
        case GPIO_PIN_GPIO107:
            GPIO107_ISR();
        break;
        case GPIO_PIN_GPIO113:
            GPIO113_ISR();
        break;
        case GPIO_PIN_GPIO127:
            GPIO127_ISR();
        break;
        case GPIO_PIN_GPIO140:
            GPIO140_ISR();
        break;
        case GPIO_PIN_GPIO201:
            GPIO201_ISR();
        break;
        default:
        break;
    }
}

/******************************************************************************/
/**  wdt_isr_callback
 *  
 *  @return None
 *  
 *  @details wdt_isr_callback is used to handle WDT monitor ISRs
 ******************************************************************************/
static void wdt_isr_callback(uintptr_t context)
{
    WDT_ISR();
}

/******************************************************************************/
/**  vtr_mon_register_isr_handlers
 *  
 *  @return None
 *  
 *  @details vtr_mon_register_isr_handlers is used to register VTR
 *  monitor handlers with Harmony 3 Interrupt handling framework
 ******************************************************************************/
void wdt_register_isr_handler(void)
{
    WDT_CallbackRegister(wdt_isr_callback, 0);
}

/******************************************************************************/
/**  vtr_mon_isr_callback
 *  
 *  @return None
 *  
 *  @details vtr_mon_isr_callback is used to handle all VTR monitor ISRs
 ******************************************************************************/
static void vtr_mon_isr_callback(uintptr_t context)
{
    if(VTR1 == context)
    {
        sb_vtr1_pad_mon_isr();
    }

    if(VTR2 == context)
    {
        sb_vtr2_pad_mon_isr();
    }
}

/******************************************************************************/
/**  vtr_mon_register_isr_handlers
 *  
 *  @return None
 *  
 *  @details vtr_mon_register_isr_handlers is used to register VTR
 *  monitor handlers with Harmony 3 Interrupt handling framework
 ******************************************************************************/
void vtr_mon_register_isr_handlers(void)
{
    EC_REG_BANK_VTR1_CallbackRegister(vtr_mon_isr_callback, VTR1);
    EC_REG_BANK_VTR2_CallbackRegister(vtr_mon_isr_callback, VTR2);
}

/******************************************************************************/
/**  qmspi_isr_callback
 *  
 *  @return None
 *  
 *  @details qmspi_isr_callback is used to handle all QMSPI ISRs
 ******************************************************************************/
static void qmspi_isr_callback(uintptr_t context)
{
    if(SPI_CHANNEL_0 == context)
    {
        qmspi_isr( SPI_CHANNEL_0);
    }

    if(SPI_CHANNEL_1 == context)
    {
        qmspi_isr( SPI_CHANNEL_1);
    }
}

/******************************************************************************/
/**  qmspi_register_isr_handlers
 *  
 *  @return None
 *  
 *  @details qmspi_register_isr_handlers is used to register QMSPI handlers
 *  with Harmony 3 Interrupt handling framework
 ******************************************************************************/
void qmspi_register_isr_handlers(void)
{
    QMSPI0_CallbackRegister(qmspi_isr_callback, SPI_CHANNEL_0);
    QMSPI1_CallbackRegister(qmspi_isr_callback, SPI_CHANNEL_1);
}

/******************************************************************************/
/**  gpio_register_isr_handlers
 *  
 *  @return None
 *  
 *  @details gpio_register_isr_handlers is used to register GPIO handlers
 *  with Harmony 3 Interrupt handling framework
 ******************************************************************************/
void gpio_register_isr_handlers(void)
{
    GPIO_PinInterruptCallbackRegister(GPIO_PIN_GPIO013, gpio_isr, 0);
    GPIO_PinInterruptCallbackRegister(GPIO_PIN_GPIO015, gpio_isr, 0);
    GPIO_PinInterruptCallbackRegister(GPIO_PIN_GPIO046, gpio_isr, 0);
    GPIO_PinInterruptCallbackRegister(GPIO_PIN_GPIO047, gpio_isr, 0);
    GPIO_PinInterruptCallbackRegister(GPIO_PIN_GPIO050, gpio_isr, 0);
    GPIO_PinInterruptCallbackRegister(GPIO_PIN_GPIO063, gpio_isr, 0);
    GPIO_PinInterruptCallbackRegister(GPIO_PIN_GPIO107, gpio_isr, 0);
    GPIO_PinInterruptCallbackRegister(GPIO_PIN_GPIO113, gpio_isr, 0);
    GPIO_PinInterruptCallbackRegister(GPIO_PIN_GPIO127, gpio_isr, 0);
    GPIO_PinInterruptCallbackRegister(GPIO_PIN_GPIO140, gpio_isr, 0);
    GPIO_PinInterruptCallbackRegister(GPIO_PIN_GPIO201, gpio_isr, 0);
}

/******************************************************************************/
/**  I2CSMB_GRP_InterruptHandler
 *  
 *  @return None
 *  
 *  @details I2CSMB_GRP_InterruptHandler handles I2C interrupts
 ******************************************************************************/
void I2CSMB_GRP_InterruptHandler ( void )
{
    smb_isr();
}

/******************************************************************************/
/**  DMA_GRP_InterruptHandler
 *  
 *  @return None
 *  
 *  @details DMA_GRP_InterruptHandler handles DMA interrupts
 ******************************************************************************/
void DMA_GRP_InterruptHandler (void)
{
    smb_dma_isr();
}

/******************************************************************************/
/**  spt_register_isr_handlers
 *
 *  @return None
 *
 *  @details spt_register_isr_handlers is used to register SPT handlers
 *  with Harmony 3 Interrupt handling framework
 ******************************************************************************/
void spt_register_isr_handlers(void)
{
    SPT1_CallbackRegister(spt_isr, SPT1);
    SPT0_CallbackRegister(spt_isr, SPT0);
}

/******************************************************************************/
/**  emc_isr_handlers
 *
 *  @return None
 *
 *  @details emc_isr_handlers is used to register EMC handlers
 *  with Harmony 3 Interrupt handling framework
 ******************************************************************************/
void emc_register_isr_handlers(void)
{
    EMC_CallbackRegister(sb_monitor_envmon_isr);
}

/**   @}
 */
