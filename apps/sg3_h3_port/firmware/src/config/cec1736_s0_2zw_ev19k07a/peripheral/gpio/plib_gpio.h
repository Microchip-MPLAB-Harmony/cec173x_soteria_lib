/*******************************************************************************
  GPIO PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_gpio.h

  Summary:
    GPIO PLIB Header File

  Description:
    This file provides an interface to control and interact with GPIO-I/O
    Pin controller module.

*******************************************************************************/

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

#ifndef PLIB_GPIO_H
#define PLIB_GPIO_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "device.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility
    extern "C" {
#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data types and constants
// *****************************************************************************
// *****************************************************************************
/*** Macros for GPIO_GPIO063 pin ***/
#define GPIO_GPIO063_Set()               (GPIO_REGS->GPIO_CTRL6[3] |= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO063_Clear()             (GPIO_REGS->GPIO_CTRL6[3] &= ~GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO063_Toggle()            (GPIO_REGS->GPIO_CTRL6[3] ^= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO063_OutputEnable()      (GPIO_REGS->GPIO_CTRL6[3] |= GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO063_InputEnable()       (GPIO_REGS->GPIO_CTRL6[3] &= ~GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO063_Get()               ((GPIO_REGS->GPIO_CTRL6[3] & GPIO_CTRL0_GPIO_INP_Msk)? 1 : 0)
#define GPIO_GPIO063_PIN                  GPIO063
/*** Macros for GPIO_GPIO113 pin ***/
#define GPIO_GPIO113_Set()               (GPIO_REGS->GPIO_CTRL11[3] |= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO113_Clear()             (GPIO_REGS->GPIO_CTRL11[3] &= ~GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO113_Toggle()            (GPIO_REGS->GPIO_CTRL11[3] ^= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO113_OutputEnable()      (GPIO_REGS->GPIO_CTRL11[3] |= GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO113_InputEnable()       (GPIO_REGS->GPIO_CTRL11[3] &= ~GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO113_Get()               ((GPIO_REGS->GPIO_CTRL11[3] & GPIO_CTRL0_GPIO_INP_Msk)? 1 : 0)
#define GPIO_GPIO113_PIN                  GPIO113
/*** Macros for GPIO_GPIO107 pin ***/
#define GPIO_GPIO107_Set()               (GPIO_REGS->GPIO_CTRL10[7] |= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO107_Clear()             (GPIO_REGS->GPIO_CTRL10[7] &= ~GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO107_Toggle()            (GPIO_REGS->GPIO_CTRL10[7] ^= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO107_OutputEnable()      (GPIO_REGS->GPIO_CTRL10[7] |= GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO107_InputEnable()       (GPIO_REGS->GPIO_CTRL10[7] &= ~GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO107_Get()               ((GPIO_REGS->GPIO_CTRL10[7] & GPIO_CTRL0_GPIO_INP_Msk)? 1 : 0)
#define GPIO_GPIO107_PIN                  GPIO107
/*** Macros for GPIO_GPIO046 pin ***/
#define GPIO_GPIO046_Set()               (GPIO_REGS->GPIO_CTRL4[6] |= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO046_Clear()             (GPIO_REGS->GPIO_CTRL4[6] &= ~GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO046_Toggle()            (GPIO_REGS->GPIO_CTRL4[6] ^= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO046_OutputEnable()      (GPIO_REGS->GPIO_CTRL4[6] |= GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO046_InputEnable()       (GPIO_REGS->GPIO_CTRL4[6] &= ~GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO046_Get()               ((GPIO_REGS->GPIO_CTRL4[6] & GPIO_CTRL0_GPIO_INP_Msk)? 1 : 0)
#define GPIO_GPIO046_PIN                  GPIO046
/*** Macros for GPIO_GPIO050 pin ***/
#define GPIO_GPIO050_Set()               (GPIO_REGS->GPIO_CTRL5[0] |= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO050_Clear()             (GPIO_REGS->GPIO_CTRL5[0] &= ~GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO050_Toggle()            (GPIO_REGS->GPIO_CTRL5[0] ^= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO050_OutputEnable()      (GPIO_REGS->GPIO_CTRL5[0] |= GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO050_InputEnable()       (GPIO_REGS->GPIO_CTRL5[0] &= ~GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO050_Get()               ((GPIO_REGS->GPIO_CTRL5[0] & GPIO_CTRL0_GPIO_INP_Msk)? 1 : 0)
#define GPIO_GPIO050_PIN                  GPIO050
/*** Macros for GPIO_GPIO015 pin ***/
#define GPIO_GPIO015_Set()               (GPIO_REGS->GPIO_CTRL1[5] |= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO015_Clear()             (GPIO_REGS->GPIO_CTRL1[5] &= ~GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO015_Toggle()            (GPIO_REGS->GPIO_CTRL1[5] ^= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO015_OutputEnable()      (GPIO_REGS->GPIO_CTRL1[5] |= GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO015_InputEnable()       (GPIO_REGS->GPIO_CTRL1[5] &= ~GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO015_Get()               ((GPIO_REGS->GPIO_CTRL1[5] & GPIO_CTRL0_GPIO_INP_Msk)? 1 : 0)
#define GPIO_GPIO015_PIN                  GPIO015
/*** Macros for GPIO_GPIO140 pin ***/
#define GPIO_GPIO140_Set()               (GPIO_REGS->GPIO_CTRL14[0] |= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO140_Clear()             (GPIO_REGS->GPIO_CTRL14[0] &= ~GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO140_Toggle()            (GPIO_REGS->GPIO_CTRL14[0] ^= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO140_OutputEnable()      (GPIO_REGS->GPIO_CTRL14[0] |= GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO140_InputEnable()       (GPIO_REGS->GPIO_CTRL14[0] &= ~GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO140_Get()               ((GPIO_REGS->GPIO_CTRL14[0] & GPIO_CTRL0_GPIO_INP_Msk)? 1 : 0)
#define GPIO_GPIO140_PIN                  GPIO140
/*** Macros for GPIO_GPIO047 pin ***/
#define GPIO_GPIO047_Set()               (GPIO_REGS->GPIO_CTRL4[7] |= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO047_Clear()             (GPIO_REGS->GPIO_CTRL4[7] &= ~GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO047_Toggle()            (GPIO_REGS->GPIO_CTRL4[7] ^= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO047_OutputEnable()      (GPIO_REGS->GPIO_CTRL4[7] |= GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO047_InputEnable()       (GPIO_REGS->GPIO_CTRL4[7] &= ~GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO047_Get()               ((GPIO_REGS->GPIO_CTRL4[7] & GPIO_CTRL0_GPIO_INP_Msk)? 1 : 0)
#define GPIO_GPIO047_PIN                  GPIO047
/*** Macros for GPIO_GPIO013 pin ***/
#define GPIO_GPIO013_Set()               (GPIO_REGS->GPIO_CTRL1[3] |= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO013_Clear()             (GPIO_REGS->GPIO_CTRL1[3] &= ~GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO013_Toggle()            (GPIO_REGS->GPIO_CTRL1[3] ^= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO013_OutputEnable()      (GPIO_REGS->GPIO_CTRL1[3] |= GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO013_InputEnable()       (GPIO_REGS->GPIO_CTRL1[3] &= ~GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO013_Get()               ((GPIO_REGS->GPIO_CTRL1[3] & GPIO_CTRL0_GPIO_INP_Msk)? 1 : 0)
#define GPIO_GPIO013_PIN                  GPIO013
/*** Macros for GPIO_GPIO127 pin ***/
#define GPIO_GPIO127_Set()               (GPIO_REGS->GPIO_CTRL12[7] |= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO127_Clear()             (GPIO_REGS->GPIO_CTRL12[7] &= ~GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO127_Toggle()            (GPIO_REGS->GPIO_CTRL12[7] ^= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO127_OutputEnable()      (GPIO_REGS->GPIO_CTRL12[7] |= GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO127_InputEnable()       (GPIO_REGS->GPIO_CTRL12[7] &= ~GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO127_Get()               ((GPIO_REGS->GPIO_CTRL12[7] & GPIO_CTRL0_GPIO_INP_Msk)? 1 : 0)
#define GPIO_GPIO127_PIN                  GPIO127
/*** Macros for GPIO_GPIO201 pin ***/
#define GPIO_GPIO201_Set()               (GPIO_REGS->GPIO_CTRL20[1] |= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO201_Clear()             (GPIO_REGS->GPIO_CTRL20[1] &= ~GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO201_Toggle()            (GPIO_REGS->GPIO_CTRL20[1] ^= GPIO_CTRL0_ALT_GPIO_DATA_Msk)
#define GPIO_GPIO201_OutputEnable()      (GPIO_REGS->GPIO_CTRL20[1] |= GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO201_InputEnable()       (GPIO_REGS->GPIO_CTRL20[1] &= ~GPIO_CTRL0_GPIO_DIR_Msk)
#define GPIO_GPIO201_Get()               ((GPIO_REGS->GPIO_CTRL20[1] & GPIO_CTRL0_GPIO_INP_Msk)? 1 : 0)
#define GPIO_GPIO201_PIN                  GPIO201

// *****************************************************************************
/* GPIO Pins

  Summary:
    Identifies the available GPIO pins.

  Description:
    This enumeration identifies all the GPIO pins that are available on this
    device.

  Remarks:
    The caller should not use the constant expressions assigned to any of
    the enumeration constants as these may vary between devices.

    GPIO pins shown here are the ones available on the selected device. Not
    all GPIO pins within a GPIO group are implemented. Refer to the device
    specific datasheet for more details.
*/

typedef enum
{
    /* GPIO000 pin */
    GPIO_PIN_GPIO000 = 0U,
    /* GPIO002 pin */
    GPIO_PIN_GPIO002 = 2U,
    /* GPIO003 pin */
    GPIO_PIN_GPIO003 = 3U,
    /* GPIO004 pin */
    GPIO_PIN_GPIO004 = 4U,
    /* GPIO012 pin */
    GPIO_PIN_GPIO012 = 10U,
    /* GPIO013 pin */
    GPIO_PIN_GPIO013 = 11U,
    /* GPIO015 pin */
    GPIO_PIN_GPIO015 = 13U,
    /* GPIO016 pin */
    GPIO_PIN_GPIO016 = 14U,
    /* GPIO020 pin */
    GPIO_PIN_GPIO020 = 16U,
    /* GPIO021 pin */
    GPIO_PIN_GPIO021 = 17U,
    /* GPIO022 pin */
    GPIO_PIN_GPIO022 = 18U,
    /* GPIO023 pin */
    GPIO_PIN_GPIO023 = 19U,
    /* GPIO024 pin */
    GPIO_PIN_GPIO024 = 20U,
    /* GPIO026 pin */
    GPIO_PIN_GPIO026 = 22U,
    /* GPIO027 pin */
    GPIO_PIN_GPIO027 = 23U,
    /* GPIO030 pin */
    GPIO_PIN_GPIO030 = 24U,
    /* GPIO031 pin */
    GPIO_PIN_GPIO031 = 25U,
    /* GPIO032 pin */
    GPIO_PIN_GPIO032 = 26U,
    /* GPIO033 pin */
    GPIO_PIN_GPIO033 = 27U,
    /* GPIO034 pin */
    GPIO_PIN_GPIO034 = 28U,
    /* GPIO045 pin */
    GPIO_PIN_GPIO045 = 37U,
    /* GPIO046 pin */
    GPIO_PIN_GPIO046 = 38U,
    /* GPIO047 pin */
    GPIO_PIN_GPIO047 = 39U,
    /* GPIO050 pin */
    GPIO_PIN_GPIO050 = 40U,
    /* GPIO053 pin */
    GPIO_PIN_GPIO053 = 43U,
    /* GPIO055 pin */
    GPIO_PIN_GPIO055 = 45U,
    /* GPIO056 pin */
    GPIO_PIN_GPIO056 = 46U,
    /* GPIO057 pin */
    GPIO_PIN_GPIO057 = 47U,
    /* GPIO063 pin */
    GPIO_PIN_GPIO063 = 51U,
    /* GPIO070 pin */
    GPIO_PIN_GPIO070 = 56U,
    /* GPIO071 pin */
    GPIO_PIN_GPIO071 = 57U,
    /* GPIO104 pin */
    GPIO_PIN_GPIO104 = 68U,
    /* GPIO105 pin */
    GPIO_PIN_GPIO105 = 69U,
    /* GPIO106 pin */
    GPIO_PIN_GPIO106 = 70U,
    /* GPIO107 pin */
    GPIO_PIN_GPIO107 = 71U,
    /* GPIO112 pin */
    GPIO_PIN_GPIO112 = 74U,
    /* GPIO113 pin */
    GPIO_PIN_GPIO113 = 75U,
    /* GPIO120 pin */
    GPIO_PIN_GPIO120 = 80U,
    /* GPIO121 pin */
    GPIO_PIN_GPIO121 = 81U,
    /* GPIO122 pin */
    GPIO_PIN_GPIO122 = 82U,
    /* GPIO123 pin */
    GPIO_PIN_GPIO123 = 83U,
    /* GPIO124 pin */
    GPIO_PIN_GPIO124 = 84U,
    /* GPIO125 pin */
    GPIO_PIN_GPIO125 = 85U,
    /* GPIO126 pin */
    GPIO_PIN_GPIO126 = 86U,
    /* GPIO127 pin */
    GPIO_PIN_GPIO127 = 87U,
    /* GPIO130 pin */
    GPIO_PIN_GPIO130 = 88U,
    /* GPIO131 pin */
    GPIO_PIN_GPIO131 = 89U,
    /* GPIO132 pin */
    GPIO_PIN_GPIO132 = 90U,
    /* GPIO140 pin */
    GPIO_PIN_GPIO140 = 96U,
    /* GPIO143 pin */
    GPIO_PIN_GPIO143 = 99U,
    /* GPIO144 pin */
    GPIO_PIN_GPIO144 = 100U,
    /* GPIO145 pin */
    GPIO_PIN_GPIO145 = 101U,
    /* GPIO146 pin */
    GPIO_PIN_GPIO146 = 102U,
    /* GPIO147 pin */
    GPIO_PIN_GPIO147 = 103U,
    /* GPIO150 pin */
    GPIO_PIN_GPIO150 = 104U,
    /* GPIO156 pin */
    GPIO_PIN_GPIO156 = 110U,
    /* GPIO157 pin */
    GPIO_PIN_GPIO157 = 111U,
    /* GPIO163 pin */
    GPIO_PIN_GPIO163 = 115U,
    /* GPIO165 pin */
    GPIO_PIN_GPIO165 = 117U,
    /* GPIO170 pin */
    GPIO_PIN_GPIO170 = 120U,
    /* GPIO171 pin */
    GPIO_PIN_GPIO171 = 121U,
    /* GPIO200 pin */
    GPIO_PIN_GPIO200 = 128U,
    /* GPIO201 pin */
    GPIO_PIN_GPIO201 = 129U,
    /* GPIO202 pin */
    GPIO_PIN_GPIO202 = 130U,
    /* GPIO203 pin */
    GPIO_PIN_GPIO203 = 131U,
    /* GPIO204 pin */
    GPIO_PIN_GPIO204 = 132U,
    /* GPIO223 pin */
    GPIO_PIN_GPIO223 = 147U,
    /* GPIO224 pin */
    GPIO_PIN_GPIO224 = 148U,
    /* GPIO227 pin */
    GPIO_PIN_GPIO227 = 151U,
    /* GPIO250 pin */
    GPIO_PIN_GPIO250 = 168U,
    /* GPIO253 pin */
    GPIO_PIN_GPIO253 = 171U,

/* This element should not be used in any of the GPIO APIs.
 * It will be used by other modules or application to denote that none of
 * the GPIO Pin is used */
    GPIO_PIN_NONE = 65535U,

} GPIO_PIN;

typedef enum
{
    GPIO_GROUP_0 = 0,
    GPIO_GROUP_1 = 1,
    GPIO_GROUP_2 = 2,
    GPIO_GROUP_3 = 3,
    GPIO_GROUP_4 = 4,
    GPIO_GROUP_5 = 5,
} GPIO_GROUP;

typedef enum
{
    GPIO_INP_ENABLE = 0,
    GPIO_INP_DISABLE = GPIO_CTRL0_INP_DIS(0x1U),
}GPIO_INP_READ;

typedef enum
{
    GPIO_FUNCTION_GPIO = GPIO_CTRL0_MUX_CTRL_GPIO,
	GPIO_FUNCTION_FUNC1 = GPIO_CTRL0_MUX_CTRL_FUNC1,
	GPIO_FUNCTION_FUNC2 = GPIO_CTRL0_MUX_CTRL_FUNC2,
	GPIO_FUNCTION_FUNC3 = GPIO_CTRL0_MUX_CTRL_FUNC3,
	

}GPIO_FUNCTION;

typedef enum
{
    GPIO_DIR_INPUT = 0,
    GPIO_DIR_OUTPUT = GPIO_CTRL0_GPIO_DIR(0x1U)
}GPIO_DIR;

typedef enum {
    GPIO_ALT_OUT_EN = 0,
    GPIO_ALT_OUT_DIS = GPIO_CTRL0_GPIO_OUT_SEL(0x1U),
} GPIO_ALT_OUT;

typedef enum
{
    GPIO_POLARITY_NON_INVERTED = 0,
    GPIO_POLARITY_INVERTED = GPIO_CTRL0_POL(0x1U),
}GPIO_POLARITY;

typedef enum
{
    GPIO_OUTPUT_BUFFER_TYPE_PUSH_PULL = 0,
    GPIO_OUTPUT_BUFFER_TYPE_OPEN_DRAIN = GPIO_CTRL0_OUT_BUFF_TYPE(0x1U),
}GPIO_OUTPUT_BUFFER_TYPE;

typedef enum
{
    GPIO_PULL_TYPE_NONE = 0x0U ,
	GPIO_PULL_TYPE_UP = 0x1U ,
	GPIO_PULL_TYPE_DOWN = 0x2U ,
	GPIO_PULL_TYPE_REPEATER = 0x3U ,
	

}GPIO_PULL_TYPE;

typedef enum
{
    GPIO_SLEW_RATE_SLOW = 0,
    GPIO_SLEW_RATE_FAST = 1
}GPIO_SLEW_RATE;

typedef enum {
    GPIO_DRV_STR_LEVEL0 = GPIO_CTRL2P0_DRIV_STREN_LEVEL0,
    GPIO_DRV_STR_LEVEL1 = GPIO_CTRL2P0_DRIV_STREN_LEVEL1,
    GPIO_DRV_STR_LEVEL2 = GPIO_CTRL2P0_DRIV_STREN_LEVEL2,
    GPIO_DRV_STR_LEVEL3 = GPIO_CTRL2P0_DRIV_STREN_LEVEL3,
} GPIO_DRV;

typedef enum
{
    GPIO_INTDET_TYPE_LOW_LEVEL = 0,
    GPIO_INTDET_TYPE_HIGH_LEVEL = ((uint32_t)0x01U << GPIO_CTRL0_INTR_DET_Pos),
    GPIO_INTDET_TYPE_DISABLED = ((uint32_t)0x04U << GPIO_CTRL0_INTR_DET_Pos),
    GPIO_INTDET_TYPE_RISING_EDGE = ((uint32_t)0x0DU << GPIO_CTRL0_INTR_DET_Pos),
    GPIO_INTDET_TYPE_FALLING_EDGE = ((uint32_t)0x0EU << GPIO_CTRL0_INTR_DET_Pos),
    GPIO_INTDET_TYPE_EITHER_EDGE = ((uint32_t)0x0FU << GPIO_CTRL0_INTR_DET_Pos),
}GPIO_INTDET_TYPE;

typedef enum {
    GPIO_PWR_VTR = 0,
    GPIO_PWR_VCC = GPIO_CTRL0_PWR_GATING(0x1U),
    GPIO_PWR_UNPWRD = GPIO_CTRL0_PWR_GATING(0x2U),
} GPIO_PWRGATE;

typedef enum {
    GPIO_PROP_PU_PD = 0UL,
    GPIO_PROP_PWR_GATE,
    GPIO_PROP_INT_DET,
    GPIO_PROP_OBUFF_TYPE,
    GPIO_PROP_DIR,
    GPIO_PROP_OUT_SRC,
    GPIO_PROP_POLARITY,
    GPIO_PROP_MUX_SEL,
    GPIO_PROP_INP_EN_DIS,
    GPIO_PROP_ALL,
} GPIO_PROPERTY;




typedef  void (*GPIO_PIN_CALLBACK) ( GPIO_PIN pin, uintptr_t context);

typedef struct {

    /* target pin */
    GPIO_PIN                 pin;

    /* Callback for event on target pin*/
    GPIO_PIN_CALLBACK        callback;

    /* Callback Context */
    uintptr_t               context;

} GPIO_PIN_CALLBACK_OBJ;


// *****************************************************************************
// *****************************************************************************
// Section: Generated API based on pin configurations done in Pin Manager
// *****************************************************************************
// *****************************************************************************

void GPIO_Initialize(void);
void GPIO_PinDirConfig(GPIO_PIN pin, GPIO_DIR dir);
void GPIO_PinOutputEnable(GPIO_PIN pin);
void GPIO_PinInputEnable(GPIO_PIN pin);
void GPIO_PinInputDisable(GPIO_PIN pin);
void GPIO_PinInputConfig(GPIO_PIN pin, GPIO_INP_READ inpEn);
void GPIO_PinGroupOutputEnable(GPIO_PIN pin);
void GPIO_PinGroupOutputDisable(GPIO_PIN pin);
void GPIO_PinGroupOutputConfig(GPIO_PIN pin, GPIO_ALT_OUT altOutputEn);
void GPIO_PinSet(GPIO_PIN pin);
void GPIO_PinClear(GPIO_PIN pin);
void GPIO_PinToggle(GPIO_PIN pin);
uint8_t GPIO_PinRead(GPIO_PIN pin);
void GPIO_PinWrite(GPIO_PIN pin, bool value);
uint8_t GPIO_PinLatchRead(GPIO_PIN pin);
void GPIO_GroupSet(GPIO_GROUP group, uint32_t mask);
void GPIO_GroupClear(GPIO_GROUP group, uint32_t mask);
void GPIO_GroupToggle(GPIO_GROUP group, uint32_t mask);
uint32_t GPIO_GroupRead(GPIO_GROUP group, uint32_t mask);
void GPIO_GroupPinSet(GPIO_PIN pin);
void GPIO_GroupPinClear(GPIO_PIN pin);
void GPIO_GroupPinToggle(GPIO_PIN pin);
uint32_t GPIO_GroupPinRead(GPIO_PIN pin);
void GPIO_PinMUXConfig(GPIO_PIN pin, GPIO_FUNCTION function);
void GPIO_PinPolarityConfig(GPIO_PIN pin, GPIO_POLARITY polarity);
void GPIO_PinOuputBufferTypeConfig(GPIO_PIN pin, GPIO_OUTPUT_BUFFER_TYPE bufferType);
void GPIO_PinPullUpPullDownConfig(GPIO_PIN pin, GPIO_PULL_TYPE pullType);
void GPIO_PinSlewRateConfig(GPIO_PIN pin, GPIO_SLEW_RATE slewRate);
void GPIO_PinIntDetectConfig(GPIO_PIN pin, GPIO_INTDET_TYPE intDet);
void GPIO_PinPwrGateConfig(GPIO_PIN pin, GPIO_PWRGATE pwrGate);
void GPIO_DrvStrConfig(GPIO_PIN pin, GPIO_DRV drvStrn);
void GPIO_PropertySet( GPIO_PIN pin, GPIO_PROPERTY gpioProp, const uint32_t propMask );

bool GPIO_PinInterruptCallbackRegister(
    GPIO_PIN pin,
    const GPIO_PIN_CALLBACK callback,
    uintptr_t context
);

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

}

#endif
// DOM-IGNORE-END
#endif // PLIB_GPIO_H
