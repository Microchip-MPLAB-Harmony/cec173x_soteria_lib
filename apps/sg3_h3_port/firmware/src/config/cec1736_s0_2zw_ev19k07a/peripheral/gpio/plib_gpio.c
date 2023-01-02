/*******************************************************************************
  GPIO PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_gpio.c

  Summary:
    Interface definition of GPIO PLIB

  Description:
    This file provides an interface to control and interact with GPIO
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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "plib_gpio.h"
#include "interrupts.h"

// *****************************************************************************
// *****************************************************************************
// Section: GPIO Implementation
// *****************************************************************************
// *****************************************************************************


#define TOTAL_NUM_OF_INT_USED 11U
static GPIO_PIN_CALLBACK_OBJ pinCallbackObj[TOTAL_NUM_OF_INT_USED];

#define GET_PINCTRL_REG_ADDR(pin)   (&GPIO_REGS->GPIO_CTRL0[0] + (uint32_t)(pin))
#define GET_PINCTRL2_REG_ADDR(pin)   (&GPIO_REGS->GPIO_CTRL2P0[0] + (uint32_t)(pin))


void GPIO_Initialize(void)
{
    GPIO_REGS->GPIO_CTRL6[3] = 0xE0;
    GPIO_REGS->GPIO_CTRL11[3] = 0xE0;
    GPIO_REGS->GPIO_CTRL10[7] = 0xE0;
    GPIO_REGS->GPIO_CTRL4[6] = 0xE0;
    GPIO_REGS->GPIO_CTRL5[0] = 0xE0;
    GPIO_REGS->GPIO_CTRL1[5] = 0xE0;
    GPIO_REGS->GPIO_CTRL14[0] = 0xE0;
    GPIO_REGS->GPIO_CTRL4[7] = 0xE0;
    GPIO_REGS->GPIO_CTRL1[3] = 0xE0;
    GPIO_REGS->GPIO_CTRL12[7] = 0xE0;
    GPIO_REGS->GPIO_CTRL20[1] = 0xE0;

    pinCallbackObj[0].pin = GPIO_PIN_GPIO063;
    pinCallbackObj[1].pin = GPIO_PIN_GPIO113;
    pinCallbackObj[2].pin = GPIO_PIN_GPIO107;
    pinCallbackObj[3].pin = GPIO_PIN_GPIO046;
    pinCallbackObj[4].pin = GPIO_PIN_GPIO050;
    pinCallbackObj[5].pin = GPIO_PIN_GPIO015;
    pinCallbackObj[6].pin = GPIO_PIN_GPIO140;
    pinCallbackObj[7].pin = GPIO_PIN_GPIO047;
    pinCallbackObj[8].pin = GPIO_PIN_GPIO013;
    pinCallbackObj[9].pin = GPIO_PIN_GPIO127;
    pinCallbackObj[10].pin = GPIO_PIN_GPIO201;
}

void GPIO_PinDirConfig(GPIO_PIN pin, GPIO_DIR dir)
{
    volatile uint32_t* pin_ctrl_reg = GET_PINCTRL_REG_ADDR(pin);
    *pin_ctrl_reg = ((*pin_ctrl_reg) & ~(GPIO_CTRL0_GPIO_DIR_Msk)) | ((uint32_t)dir);
}

void GPIO_PinInputEnable(GPIO_PIN pin)
{
    volatile uint32_t* pin_ctrl_reg = GET_PINCTRL_REG_ADDR(pin);
    *pin_ctrl_reg &= ~GPIO_CTRL0_INP_DIS_Msk;
}

void GPIO_PinInputDisable(GPIO_PIN pin)
{
    volatile uint32_t* pin_ctrl_reg = GET_PINCTRL_REG_ADDR(pin);
    *pin_ctrl_reg |= GPIO_CTRL0_INP_DIS_Msk;
}

void GPIO_PinInputConfig(GPIO_PIN pin, GPIO_INP_READ inpEn)
{
    volatile uint32_t* pin_ctrl_reg = GET_PINCTRL_REG_ADDR(pin);
    *pin_ctrl_reg = ((*pin_ctrl_reg) & ~(GPIO_CTRL0_INP_DIS_Msk)) | (uint32_t)inpEn;
}

void GPIO_PinGroupOutputEnable(GPIO_PIN pin)
{
    volatile uint32_t* pin_ctrl_reg = GET_PINCTRL_REG_ADDR(pin);
    *pin_ctrl_reg |= GPIO_CTRL0_GPIO_OUT_SEL_Msk;
}

void GPIO_PinGroupOutputDisable(GPIO_PIN pin)
{
    volatile uint32_t* pin_ctrl_reg = GET_PINCTRL_REG_ADDR(pin);
    *pin_ctrl_reg &= ~GPIO_CTRL0_GPIO_OUT_SEL_Msk;
}

void GPIO_PinGroupOutputConfig(GPIO_PIN pin, GPIO_ALT_OUT altOutputEn)
{
    volatile uint32_t* pin_ctrl_reg = GET_PINCTRL_REG_ADDR(pin);
    *pin_ctrl_reg = ((*pin_ctrl_reg) & ~(GPIO_CTRL0_GPIO_OUT_SEL_Msk)) | (uint32_t)altOutputEn;
}

void GPIO_PinSet(GPIO_PIN pin)
{
    volatile uint32_t* pin_ctrl_reg = GET_PINCTRL_REG_ADDR(pin);
    *pin_ctrl_reg |= GPIO_CTRL0_ALT_GPIO_DATA_Msk;
}

void GPIO_PinClear(GPIO_PIN pin)
{
    volatile uint32_t* pin_ctrl_reg = GET_PINCTRL_REG_ADDR(pin);
    *pin_ctrl_reg &= ~(GPIO_CTRL0_ALT_GPIO_DATA_Msk);
}

void GPIO_PinToggle(GPIO_PIN pin)
{
    volatile uint32_t* pin_ctrl_reg = GET_PINCTRL_REG_ADDR(pin);
    *pin_ctrl_reg ^= GPIO_CTRL0_ALT_GPIO_DATA_Msk;
}

uint8_t GPIO_PinRead(GPIO_PIN pin)
{
    volatile uint32_t* pin_ctrl_reg = GET_PINCTRL_REG_ADDR(pin);
    return (((*pin_ctrl_reg) & (GPIO_CTRL0_GPIO_INP_Msk)) > 0U)? 1U : 0U;
}

void GPIO_GroupSet(GPIO_GROUP group, uint32_t mask)
{
    GPIO_REGS->GPIO_PAROUT[group] |= mask;
}

void GPIO_GroupClear(GPIO_GROUP group, uint32_t mask)
{
    GPIO_REGS->GPIO_PAROUT[group] &= ~mask;
}

void GPIO_GroupToggle(GPIO_GROUP group, uint32_t mask)
{
    GPIO_REGS->GPIO_PAROUT[group] ^= mask;
}

uint32_t GPIO_GroupRead(GPIO_GROUP group, uint32_t mask)
{
    return GPIO_REGS->GPIO_PARIN[group] & mask;
}

void GPIO_GroupPinSet(GPIO_PIN pin)
{
    uint32_t index = ((uint32_t)pin >> 5U);
    uint32_t bit_pos = (uint32_t)pin - (index << 5U);

    GPIO_REGS->GPIO_PAROUT[index] |= (1UL << bit_pos);
}

void GPIO_GroupPinClear(GPIO_PIN pin)
{
    uint32_t index = ((uint32_t)pin >> 5U);
    uint32_t bit_pos = (uint32_t)pin - (index << 5U);

    GPIO_REGS->GPIO_PAROUT[index] &= ~(1UL << bit_pos);
}

void GPIO_GroupPinToggle(GPIO_PIN pin)
{
    uint32_t index = ((uint32_t)pin >> 5U);
    uint32_t bit_pos = (uint32_t)pin - (index << 5U);

    GPIO_REGS->GPIO_PAROUT[index] ^= (1UL << bit_pos);
}

uint32_t GPIO_GroupPinRead(GPIO_PIN pin)
{
    uint32_t index = ((uint32_t)pin >> 5U);
    uint32_t bit_pos = (uint32_t)pin - (index << 5U);

    return GPIO_REGS->GPIO_PARIN[index] & (1UL << bit_pos);
}

void GPIO_PinMUXConfig(GPIO_PIN pin, GPIO_FUNCTION function)
{
    volatile uint32_t* pin_ctrl_reg = GET_PINCTRL_REG_ADDR(pin);
    *pin_ctrl_reg = ((*pin_ctrl_reg) & ~(GPIO_CTRL0_MUX_CTRL_Msk)) | ((uint32_t)function);
}

void GPIO_PinPolarityConfig(GPIO_PIN pin, GPIO_POLARITY polarity)
{
    volatile uint32_t* pin_ctrl_reg = GET_PINCTRL_REG_ADDR(pin);
    *pin_ctrl_reg = ((*pin_ctrl_reg) & ~(GPIO_CTRL0_POL_Msk)) | (uint32_t)polarity;
}

void GPIO_PinOuputBufferTypeConfig(GPIO_PIN pin, GPIO_OUTPUT_BUFFER_TYPE bufferType)
{
    volatile uint32_t* pin_ctrl_reg = GET_PINCTRL_REG_ADDR(pin);
    *pin_ctrl_reg = ((*pin_ctrl_reg) & ~(GPIO_CTRL0_OUT_BUFF_TYPE_Msk)) | (uint32_t)bufferType;
}

void GPIO_PinPullUpPullDownConfig(GPIO_PIN pin, GPIO_PULL_TYPE pullType)
{
    volatile uint32_t* pin_ctrl_reg = GET_PINCTRL_REG_ADDR(pin);
    *pin_ctrl_reg = ((*pin_ctrl_reg) & ~(GPIO_CTRL0_PU_PD_Msk)) | (uint32_t)pullType;
}

void GPIO_PinSlewRateConfig(GPIO_PIN pin, GPIO_SLEW_RATE slewRate)
{
    volatile uint32_t* pin_ctrl2_reg = GET_PINCTRL2_REG_ADDR(pin);
    *pin_ctrl2_reg = ((*pin_ctrl2_reg) & ~(GPIO_CTRL2P0_SLEW_CTRL_Msk)) | (uint32_t)slewRate;
}

void GPIO_PinIntDetectConfig(GPIO_PIN pin, GPIO_INTDET_TYPE intDet)
{
    volatile uint32_t* pin_ctrl_reg = GET_PINCTRL_REG_ADDR(pin);
    *pin_ctrl_reg = ((*pin_ctrl_reg) & ~(GPIO_CTRL0_INTR_DET_Msk | GPIO_CTRL0_EDGE_EN_Msk)) | (uint32_t)intDet;
}

void GPIO_PinPwrGateConfig(GPIO_PIN pin, GPIO_PWRGATE pwrGate)
{
    volatile uint32_t* pin_ctrl_reg = GET_PINCTRL_REG_ADDR(pin);
    *pin_ctrl_reg = ((*pin_ctrl_reg) & ~(GPIO_CTRL0_PWR_GATING_Msk)) | (uint32_t)pwrGate;
}

void GPIO_DrvStrConfig(GPIO_PIN pin, GPIO_DRV drvStrn)
{
    volatile uint32_t* pin_ctrl2_reg = GET_PINCTRL2_REG_ADDR(pin);
    *pin_ctrl2_reg = ((*pin_ctrl2_reg) & ~(GPIO_CTRL2P0_DRIV_STREN_Msk)) | (uint32_t)drvStrn;
}

void GPIO_PropertySet( GPIO_PIN pin, GPIO_PROPERTY gpioProp, const uint32_t propMask )
{
    switch ( gpioProp )
    {
        case GPIO_PROP_PU_PD:
            GPIO_PinPullUpPullDownConfig( pin, (GPIO_PULL_TYPE)propMask );
            break;

        case GPIO_PROP_PWR_GATE:
            GPIO_PinPwrGateConfig(pin, (GPIO_PWRGATE)propMask);
            break;

        case GPIO_PROP_INT_DET:
            GPIO_PinIntDetectConfig( pin, (GPIO_INTDET_TYPE)propMask );
            break;

        case GPIO_PROP_OBUFF_TYPE:
            GPIO_PinOuputBufferTypeConfig( pin, (GPIO_OUTPUT_BUFFER_TYPE)propMask );
            break;

        case GPIO_PROP_DIR:
            GPIO_PinDirConfig(pin, (GPIO_DIR)propMask);
            break;

        case GPIO_PROP_OUT_SRC:
            GPIO_PinGroupOutputConfig(pin, (GPIO_ALT_OUT)propMask);
            break;

        case GPIO_PROP_POLARITY:
            GPIO_PinPolarityConfig( pin, (GPIO_POLARITY)propMask );
            break;

        case GPIO_PROP_MUX_SEL:
            GPIO_PinMUXConfig(pin, (GPIO_FUNCTION)propMask);
            break;

        case GPIO_PROP_INP_EN_DIS:
            GPIO_PinInputConfig(pin, (GPIO_INP_READ)propMask);
            break;

        case GPIO_PROP_ALL:
        {
            volatile uint32_t* pin_ctrl_reg = GET_PINCTRL_REG_ADDR(pin);
            *pin_ctrl_reg = propMask;
            break;
        }

        default:
            /* Do nothing */
            break;
    }
}

bool GPIO_PinInterruptCallbackRegister(
    GPIO_PIN pin,
    const GPIO_PIN_CALLBACK callback,
    uintptr_t context
)
{
    uint32_t i;

    for(i = 0; i < TOTAL_NUM_OF_INT_USED; i++)
    {
        if (pinCallbackObj[i].pin == pin)
        {
            pinCallbackObj[i].callback = callback;
            pinCallbackObj[i].context  = context;
            return true;
        }
    }
    return false;
}

void GPIO063_GRP_InterruptHandler(void)
{
    if ((ECIA_REGS->ECIA_RESULT10 & ((uint32_t)1U << 19)) != 0U)
    {
        ECIA_REGS->ECIA_SRC10 |= ((uint32_t)1U << 19);
        if (pinCallbackObj[0].callback != NULL)
        {
            pinCallbackObj[0].callback(GPIO_PIN_GPIO063, pinCallbackObj[0].context);
        }
    }
}
void GPIO113_GRP_InterruptHandler(void)
{
    if ((ECIA_REGS->ECIA_RESULT9 & ((uint32_t)1U << 11)) != 0U)
    {
        ECIA_REGS->ECIA_SRC9 |= ((uint32_t)1U << 11);
        if (pinCallbackObj[1].callback != NULL)
        {
            pinCallbackObj[1].callback(GPIO_PIN_GPIO113, pinCallbackObj[1].context);
        }
    }
}
void GPIO107_GRP_InterruptHandler(void)
{
    if ((ECIA_REGS->ECIA_RESULT9 & ((uint32_t)1U << 7)) != 0U)
    {
        ECIA_REGS->ECIA_SRC9 |= ((uint32_t)1U << 7);
        if (pinCallbackObj[2].callback != NULL)
        {
            pinCallbackObj[2].callback(GPIO_PIN_GPIO107, pinCallbackObj[2].context);
        }
    }
}
void GPIO046_GRP_InterruptHandler(void)
{
    if ((ECIA_REGS->ECIA_RESULT10 & ((uint32_t)1U << 6)) != 0U)
    {
        ECIA_REGS->ECIA_SRC10 |= ((uint32_t)1U << 6);
        if (pinCallbackObj[3].callback != NULL)
        {
            pinCallbackObj[3].callback(GPIO_PIN_GPIO046, pinCallbackObj[3].context);
        }
    }
}
void GPIO050_GRP_InterruptHandler(void)
{
    if ((ECIA_REGS->ECIA_RESULT10 & ((uint32_t)1U << 8)) != 0U)
    {
        ECIA_REGS->ECIA_SRC10 |= ((uint32_t)1U << 8);
        if (pinCallbackObj[4].callback != NULL)
        {
            pinCallbackObj[4].callback(GPIO_PIN_GPIO050, pinCallbackObj[4].context);
        }
    }
}
void GPIO015_GRP_InterruptHandler(void)
{
    if ((ECIA_REGS->ECIA_RESULT11 & ((uint32_t)1U << 13)) != 0U)
    {
        ECIA_REGS->ECIA_SRC11 |= ((uint32_t)1U << 13);
        if (pinCallbackObj[5].callback != NULL)
        {
            pinCallbackObj[5].callback(GPIO_PIN_GPIO015, pinCallbackObj[5].context);
        }
    }
}
void GPIO140_GRP_InterruptHandler(void)
{
    if ((ECIA_REGS->ECIA_RESULT8 & ((uint32_t)1U << 0)) != 0U)
    {
        ECIA_REGS->ECIA_SRC8 |= ((uint32_t)1U << 0);
        if (pinCallbackObj[6].callback != NULL)
        {
            pinCallbackObj[6].callback(GPIO_PIN_GPIO140, pinCallbackObj[6].context);
        }
    }
}
void GPIO047_GRP_InterruptHandler(void)
{
    if ((ECIA_REGS->ECIA_RESULT10 & ((uint32_t)1U << 7)) != 0U)
    {
        ECIA_REGS->ECIA_SRC10 |= ((uint32_t)1U << 7);
        if (pinCallbackObj[7].callback != NULL)
        {
            pinCallbackObj[7].callback(GPIO_PIN_GPIO047, pinCallbackObj[7].context);
        }
    }
}
void GPIO013_GRP_InterruptHandler(void)
{
    if ((ECIA_REGS->ECIA_RESULT11 & ((uint32_t)1U << 11)) != 0U)
    {
        ECIA_REGS->ECIA_SRC11 |= ((uint32_t)1U << 11);
        if (pinCallbackObj[8].callback != NULL)
        {
            pinCallbackObj[8].callback(GPIO_PIN_GPIO013, pinCallbackObj[8].context);
        }
    }
}
void GPIO127_GRP_InterruptHandler(void)
{
    if ((ECIA_REGS->ECIA_RESULT9 & ((uint32_t)1U << 23)) != 0U)
    {
        ECIA_REGS->ECIA_SRC9 |= ((uint32_t)1U << 23);
        if (pinCallbackObj[9].callback != NULL)
        {
            pinCallbackObj[9].callback(GPIO_PIN_GPIO127, pinCallbackObj[9].context);
        }
    }
}
void GPIO201_GRP_InterruptHandler(void)
{
    if ((ECIA_REGS->ECIA_RESULT12 & ((uint32_t)1U << 1)) != 0U)
    {
        ECIA_REGS->ECIA_SRC12 |= ((uint32_t)1U << 1);
        if (pinCallbackObj[10].callback != NULL)
        {
            pinCallbackObj[10].callback(GPIO_PIN_GPIO201, pinCallbackObj[10].context);
        }
    }
}
