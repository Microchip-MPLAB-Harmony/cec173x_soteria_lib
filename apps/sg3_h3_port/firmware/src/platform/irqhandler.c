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
extern void qmspi_isr(uint8_t channel);

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

/**   @}
 */
