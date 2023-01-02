/*****************************************************************************
* © 2015 Microchip Technology Inc. and its subsidiaries.
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
$Revision: #2 $
$DateTime: 2023/01/02 04:42:23 $
$Author: i64652 $
Last Change:    None
******************************************************************************/
/** @file gpio.h
* \brief GPIO Peripheral Header File
* \author pramans
*
* This file is the header file for GPIO Peripheral
******************************************************************************/

/** @defgroup GPIO
 *  @{
 */

#ifndef GPIO_H
#define GPIO_H


#ifdef __cplusplus
 extern "C" {
#endif

/* ------------------------------------------------------------------------------ */
/*  API Function - Function to set gpio property                                  */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_property_set - updates the configuration of the specified gpio pin
 * at run time.
 * @param 0-based GPIO ID
 * @param property type that is to be changed
 * @param new value
 * @return uint8_t 1(success), 0(fail).
 */
uint8_t gpio_property_set( GPIO_PIN pin, GPIO_PROPERTY gpio_prop, const uint32_t new_prop_val );
	 
/* ------------------------------------------------------------------------------ */
/*  API Function - Function to initialize gpio pin                                */
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
 * @return uint8_t 1(success), 0(fail).
 */
uint8_t gpio_init( GPIO_PIN pin, GPIO_INP_READ new_val, GPIO_FUNCTION new_mux, GPIO_POLARITY new_pol, GPIO_DIR new_dir, \
                   GPIO_OUTPUT_BUFFER_TYPE new_obuf, GPIO_INTDET_TYPE new_idet, GPIO_PWRGATE new_pwrg, GPIO_PULL_TYPE new_pud );

/* ------------------------------------------------------------------------------ */
/*  API Function - Function to write to the pad                                   */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_output_set - writes the output value to the specified
 * pin depending upon the output source register.
 * @param 0-based GPIO ID
 * @param output source register
 * @param new output value
 * @return uint8_t 1(success), 0(fail).
 */
uint8_t gpio_output_set( GPIO_PIN pin, GPIO_ALT_OUT out_src, const uint32_t gpio_state );


/* ------------------------------------------------------------------------------ */
/*  API Function - Function to read the pad input using the input register        */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_input_get - reads the input from the specified gpio pin
 * using the input register.
 * @param 0-based GPIO ID
 * @return uint8_t current input value, 0xFF(fail).
 */
uint8_t gpio_input_get( GPIO_PIN pin );

/* ------------------------------------------------------------------------------ */
/*  API Function - Function to set the gpio slew rate value                       */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_slewRate_set - sets the slew rate setting of the
 * specified gpio pin.
 * @param 0-based GPIO ID
 * @param new slew value
 * @return uint8_t 1(success), 0(fail).
 */
uint8_t gpio_slewRate_set( GPIO_PIN pin, GPIO_SLEW_RATE new_slew );

/* ------------------------------------------------------------------------------ */
/*  API Function - Function to set the gpio drive strength value                  */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_drvStr_set -sets the drive strength setting of the
 * specified gpio pin.
 * @param 0-based GPIO ID
 * @param new drive strength setting
 * @return uint8_t 1(success), 0(fail).
 */
uint8_t gpio_drvStr_set( GPIO_PIN pin, GPIO_DRV drv_str );

/* ------------------------------------------------------------------------------ */
/*                  Function to configure a default output high gpio              */
/* ------------------------------------------------------------------------------ */
/**
 * gpio_init_default_output_high - For default output high gpio pin, it should be
 * configured as per below steps to avoid glitches
 * specified gpio pin.
 * @param GPIO PIN number
 * @return None.
 */
void gpio_init_default_output_high( GPIO_PIN pin);

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
uint8_t gpio_sample_pin_state(GPIO_PIN pin_id, uint32_t num_50us_samples);

#ifdef __cplusplus
}
#endif

#endif

/* end of gpio.h */
/**   @}
 */
