/*****************************************************************************
* © 2020 Microchip Technology Inc. and its subsidiaries.
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
******************************************************************************

Version Control Information (Perforce)
******************************************************************************
$Revision: #1 $
$DateTime: 2023/01/02 04:27:58 $
$Author: i64652 $
Last Change:    None
******************************************************************************/
/** @file gpio_api.c
* \brief GPIO API Source file
* \author pramans
*
* This file implements the GPIO API functions
******************************************************************************/

/** @defgroup GPIO
 *  @{
 */

#include "common.h"
#include "peripheral/gpio/plib_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_GPIO_PORTS      6u
#define MAX_NUM_GPIO        (NUM_GPIO_PORTS * 32)

//
// Logical bit map representation of each ports
//
#define GPIO_BANK0_BITMAP (0x7FFFFFFFul) /* 0000 - 0037 */
#define GPIO_BANK1_BITMAP (0x7FFFFFFFul) /* 0040 - 0077 */
#define GPIO_BANK2_BITMAP (0x7FFFFFFFul) /* 0100 - 0137 */
#define GPIO_BANK3_BITMAP (0x7FFFFFFFul) /* 0140 - 0177 */
#define GPIO_BANK4_BITMAP (0x7FFFFFFFul) /* 0200 - 0237 */
#define GPIO_BANK5_BITMAP (0x00FFFFFFul) /* 0240 - 0277 */

//
// Logical bit map representation of each ports
//
#define GPIO_BANK0_BITMAP (0x7FFFFFFFul) /* 0000 - 0037 */
#define GPIO_BANK1_BITMAP (0x7FFFFFFFul) /* 0040 - 0077 */
#define GPIO_BANK2_BITMAP (0x7FFFFFFFul) /* 0100 - 0137 */
#define GPIO_BANK3_BITMAP (0x7FFFFFFFul) /* 0140 - 0177 */
#define GPIO_BANK4_BITMAP (0x7FFFFFFFul) /* 0200 - 0237 */
#define GPIO_BANK5_BITMAP (0x00FFFFFFul) /* 0240 - 0277 */

// scaled for 3.06 MHz (3MHz + 2%) operation
#define TIMER_CNT_50US          (153ul)
// 1 millisecond sample 50 micro second x 20  = 1 millisecond
#define SAMPLE_1MS              (20ul)
#define MAX_PIN					(GPIO_PIN_GPIO253 + 1)

extern void timer_delay_us(uint32_t num_us);
static uint8_t gpio_is_valid( GPIO_PIN pin );

// GPIO Port bitmap
static const uint32_t valid_ctrl_masks[NUM_GPIO_PORTS] = {
    (GPIO_BANK0_BITMAP),
    (GPIO_BANK1_BITMAP),
    (GPIO_BANK2_BITMAP),
    (GPIO_BANK3_BITMAP),
    (GPIO_BANK4_BITMAP),
    (GPIO_BANK5_BITMAP)
};

/* ------------------------------------------------------------------------------ */
/*                      Function to set gpio property                             */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_property_set - updates the configuration of the specified gpio pin
 * at run time.
 * @param 0-based GPIO ID
 * @param property type that is to be changed
 * @param new value
 * @return uint8_t 0(success), 1(fail).
 */
uint8_t gpio_property_set( GPIO_PIN pin, GPIO_PROPERTY gpio_prop, const uint32_t new_prop_val )
{
    uint8_t retCode = 0u;
    if ( gpio_is_valid(pin) )
    {
        GPIO_PropertySet(pin, gpio_prop, new_prop_val);
        retCode = 0u;
    }
    else
    {
        retCode = 1u;
    }
    return retCode;
}

/* ------------------------------------------------------------------------------ */
/*                      Function to initialize gpio pin                           */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_init - inititializes the specified gpio pin.
 * @param 0-based GPIO ID
 * @param input disable
 * @param control mux mode
 * @param pin polarity
 * @param direction of the pin
 * @param output buffer type
 * @param interrupt detection mode
 * @param power gate source
 * @param internal resistor mode
 * @return uint8_t 0(success), 1(fail).
 */
uint8_t gpio_init( GPIO_PIN pin, GPIO_INP_READ new_val, GPIO_FUNCTION new_mux, GPIO_POLARITY new_pol, GPIO_DIR new_dir, \
                   GPIO_OUTPUT_BUFFER_TYPE new_obuf, GPIO_INTDET_TYPE new_idet, GPIO_PWRGATE new_pwrg, GPIO_PULL_TYPE new_pud )
{
    uint8_t ret_init_sts = 0u;
    if ( gpio_is_valid(pin) )
    {
		gpio_property_set(pin, GPIO_PROP_INP_EN_DIS, new_val);
		gpio_property_set(pin, GPIO_PROP_MUX_SEL, new_mux);
		gpio_property_set(pin, GPIO_PROP_POLARITY, new_pol);
		gpio_property_set(pin, GPIO_PROP_DIR, new_dir);
		gpio_property_set(pin, GPIO_PROP_OBUFF_TYPE, new_obuf);
		gpio_property_set(pin, GPIO_PROP_INT_DET, new_idet);
		gpio_property_set(pin, GPIO_PROP_PWR_GATE, new_pwrg);
		gpio_property_set(pin, GPIO_PROP_PU_PD, new_pud);
		
        ret_init_sts = 0u;
    }
    else
    {
        ret_init_sts = 1u;
    }
	
    return ret_init_sts;
}

/* ------------------------------------------------------------------------------ */
/*                          Function to write to the pad                          */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_output_set - writes the output value to the specified
 * pin depending upon the output source register.
 * @param 0-based GPIO ID
 * @param output source register
 * @param new output value
 * @return uint8_t 0(success), 1(fail).
 *
 */

uint8_t gpio_output_set( GPIO_PIN pin, GPIO_ALT_OUT out_src, const uint32_t gpio_state )
{
    uint8_t ret_op_set_sts = 0ul;

    if ( gpio_is_valid(pin) )
    {
//        p_gpio_output_write_enable( pin, out_src );
		GPIO_PinGroupOutputConfig( pin, out_src );

        if ( out_src )
        {
			if(gpio_state) {
				GPIO_GroupPinSet(pin);
			} else {
				GPIO_GroupPinClear(pin);
			}
        }
        else
        {
			if(gpio_state) {
				GPIO_PinSet(pin);
			} else {
				GPIO_PinClear(pin);
			}
        }

        ret_op_set_sts = 0u;
    }
    else
    {
        ret_op_set_sts = 1u;
    }
    return ret_op_set_sts;
}

/* ------------------------------------------------------------------------------ */
/*               Function to read the pad input using the input register          */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_input_get - reads the input from the specified gpio pin
 * using the input register.
 * @param 0-based GPIO ID
 * @return uint8_t current input value, 0xFF(fail).
 */
uint8_t gpio_input_get( GPIO_PIN pin )
{
    uint8_t ret_ip_get_sts = 0xFFu;
    if ( gpio_is_valid(pin) )
    {
        ret_ip_get_sts = GPIO_PinRead( pin );
    }
    else
    {
        ret_ip_get_sts = 0xFFu;
    }
    return ret_ip_get_sts;
}


/* ------------------------------------------------------------------------------ */
/*                      Function to configure gpio slew rate value                */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_slewRate_set - sets the slew rate setting of the
 * specified gpio pin.
 * @param 0-based GPIO ID
 * @param new slew value
 * @return uint8_t 0(success), 1(fail).
 */
uint8_t gpio_slewRate_set( GPIO_PIN pin, GPIO_SLEW_RATE new_slew )
{
    uint8_t ret_slw_set_sts = 0u;
    if ( gpio_is_valid(pin) )
    {
		GPIO_PinSlewRateConfig(pin, new_slew);

        ret_slw_set_sts = 0u;
    }
    else
    {
        ret_slw_set_sts = 1u;
    }

    return ret_slw_set_sts;
}


/* ------------------------------------------------------------------------------ */
/*                  Function to configure gpio drive strength value               */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_drvStr_set -sets the drive strength setting of the
 * specified gpio pin.
 * @param 0-based GPIO ID
 * @param new drive strength setting
 * @return uint8_t 0(success), 1(fail).
 */
uint8_t gpio_drvStr_set( GPIO_PIN pin, GPIO_DRV drv_str )
{
    uint8_t ret_drvstr_set_sts = 0u;
    if ( gpio_is_valid(pin) )
    {
		GPIO_DrvStrConfig(pin, drv_str);
		
        ret_drvstr_set_sts = 0u;
    }
    else
    {
        ret_drvstr_set_sts = 1u;
    }
    return ret_drvstr_set_sts;
}

/* ------------------------------------------------------------------------------ */
/*                  Function to configure a default output high gpio              */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_init_default_output_high - For default output high gpio pin, it should be
 * configured as per below steps to avoid glitches
 * specified gpio pin.
 * @param GPIO PIN number
 * @return void.
 */
void gpio_init_default_output_high( GPIO_PIN pin)
{
    gpio_init( pin, GPIO_INP_ENABLE, GPIO_FUNCTION_GPIO, GPIO_POLARITY_NON_INVERTED, \
               GPIO_DIR_INPUT, GPIO_OUTPUT_BUFFER_TYPE_PUSH_PULL, GPIO_INTDET_TYPE_DISABLED, GPIO_PWR_VTR,\
               GPIO_PULL_TYPE_NONE );

    gpio_output_set( pin, GPIO_ALT_OUT_DIS, 1u );

    gpio_property_set( pin, GPIO_PROP_DIR, GPIO_DIR_OUTPUT );

}

/* ------------------------------------------------------------------------------ */
/*                  Function to read gpio pin state at 50us interval              */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_sample_pin_state - This function shall read the gpio state at 50 microsecond
 * interval for 1millisecond time and return the value. If the sample value meet the pin
 * state for 3 count then return the state. otherwise sample the pin till the 
 * num_50us_samples times and return the value.
 * @param GPIO_PIN pin_id gpio pin number
 * @param uint32_t num_50us_samples number of samples to be taken
 * @return 0 or 1 based on pin state after sample
 */
uint8_t gpio_sample_pin_state(GPIO_PIN pin_id, uint32_t num_50us_samples)
{
    uint8_t pin1 = 0xFFu;  // indeterminate
    uint32_t nsamples = 0u;
    uint8_t pin_cnt = 0u;
    uint8_t pin_state = 0x00;
    pin_state = gpio_input_get(pin_id);
    
    while (nsamples < num_50us_samples)
    {
        timer_delay_us(TIMER_CNT_50US); //check this 50 actually provide 50 micro second
        pin1 = gpio_input_get(pin_id);
        if (pin1 == pin_state){
            pin_cnt++;
        } 
        else{
            pin_cnt = 0u;
        }
        
        if (pin_cnt > 3){
            break;
        }
        
        nsamples++;
    }
    
    return pin1;
}

/* ------------------------------------------------------------------------------ */
/*                  Function to check if the gpio pin is a valid one              */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_is_valid - Checks if GPIO pin is implemented in this hardware.
 * @param 0-based GPIO ID
 * @return uint8_t 1(GPIO Pin implemented), 0(not implemented).
 */
static uint8_t gpio_is_valid( GPIO_PIN pin )
{
    uint8_t ret_valid_sts = 0u;
    uint16_t gp_bank = 0u;

    if ( pin < MAX_PIN )
    {
        gp_bank = (uint16_t)pin >> 5;

        if ( valid_ctrl_masks[gp_bank] & (1 << (pin & 0x001Fu)) )
        {
            ret_valid_sts = 1u;
        }
    }
    else
    {
        ret_valid_sts = 0u;
    }

    return ret_valid_sts;
}

#ifdef __cplusplus
}
#endif

/* end of gpio_api.c */
/**   @}
 */
