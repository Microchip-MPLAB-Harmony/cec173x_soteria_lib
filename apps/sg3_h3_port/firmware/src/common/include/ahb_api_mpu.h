/*****************************************************************************
* © 2021 Microchip Technology Inc. and its subsidiaries.
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

#ifndef INCLUDE_AHB_API_MPU_H_
#define INCLUDE_AHB_API_MPU_H_

uint8_t AHB_API_gpio_init( GPIO_PIN pin, GPIO_INP_READ new_val, GPIO_FUNCTION new_mux, GPIO_POLARITY new_pol, GPIO_DIR new_dir, \
                   GPIO_OUTPUT_BUFFER_TYPE new_obuf, GPIO_INTDET_TYPE new_idet, GPIO_PWRGATE new_pwrg, GPIO_PULL_TYPE new_pud);
uint8_t AHB_API_gpio_init_ISR( GPIO_PIN pin, GPIO_INP_READ new_val, GPIO_FUNCTION new_mux, GPIO_POLARITY new_pol, GPIO_DIR new_dir, \
                   GPIO_OUTPUT_BUFFER_TYPE new_obuf, GPIO_INTDET_TYPE new_idet, GPIO_PWRGATE new_pwrg, GPIO_PULL_TYPE new_pud);
uint8_t AHB_API_gpio_property_set(GPIO_PIN pin, GPIO_PROPERTY gpio_prop, const uint32_t new_prop_val);
uint8_t AHB_API_gpio_property_set_ISR(GPIO_PIN pin, GPIO_PROPERTY gpio_prop, const uint32_t new_prop_val);
uint8_t AHB_API_gpio_output_set( GPIO_PIN pin, GPIO_ALT_OUT out_src, const uint32_t gpio_state );
uint8_t AHB_API_gpio_output_set_ISR( GPIO_PIN pin, GPIO_ALT_OUT out_src, const uint32_t gpio_state );
uint8_t AHB_API_gpio_input_get(GPIO_PIN pin);
uint8_t AHB_API_gpio_input_get_ISR(GPIO_PIN pin);
uint8_t AHB_API_gpio_sample_pin_state(GPIO_PIN pin, uint32_t num_50us_samples);
uint8_t AHB_API_gpio_slewRate_set( GPIO_PIN pin, GPIO_SLEW_RATE new_slew);
uint8_t AHB_API_gpio_drvStr_set( GPIO_PIN pin, GPIO_DRV drv_str);
uint8_t AHB_API_gpio_init_default_output_high(GPIO_PIN pin);

uint8_t AHB_API_mchp_privileged_ecia_init(uint32_t direct_bitmap, uint8_t dflt_priority);
uint32_t AHB_API_interrupt_device_ecia_source_get(const uint32_t dev_iroute);
uint8_t AHB_API_interrupt_device_ecia_enable_clear(const uint32_t dev_iroute);
uint8_t AHB_API_interrupt_device_ecia_enable_set(const uint32_t dev_iroute);
uint8_t AHB_API_interrupt_device_ecia_source_clear(const uint32_t dev_iroute);
uint8_t AHB_API_interrupt_device_enable(uint32_t dev_iroute, uint8_t is_aggregated);
uint8_t AHB_API_interrupt_device_disable(uint32_t dev_iroute, uint8_t is_aggregated);
uint8_t AHB_API_interrupt_device_girqs_source_reset(const uint32_t dev_iroute);
uint32_t AHB_API_interrupt_device_ecia_result_get_ISR(const uint32_t dev_iroute);
uint8_t AHB_API_interrupt_device_ecia_enable_clear_ISR(const uint32_t dev_iroute);
uint8_t AHB_API_interrupt_device_ecia_enable_set_ISR(const uint32_t dev_iroute);
uint8_t AHB_API_interrupt_device_ecia_source_clear_ISR(const uint32_t dev_iroute);
uint8_t SRAM_RLOG_API_tag0_buildnum_read(uint8_t *buffer);
uint8_t SRAM_MBOX_API_read_container_crisis_interface_en(uint8_t *buffer);

#endif /* INCLUDE_AHB_API_MPU_H_ */
