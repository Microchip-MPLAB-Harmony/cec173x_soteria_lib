/*****************************************************************************
 * Copyright (c) 2022 Microchip Technology Inc. and its subsidiaries.
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

#include "common.h"
#include "vtr_mon_api.h"
#include "peripheral/ec_reg_bank/plib_ec_reg_bank.h"

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to convert Pad Monitor Control value to Milliseconds                                      */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_get_vtr_pad_ctrl_ms - This function converts the given pad control bits for
 * VTR1 to delay in millisaconds(ms)
 * @param VTR_PAD_CTRL_VALUE pad_ctrl_bits - Pad Monitor Control VTR1 bits.
 * Refer VTR_PAD_CTRL_VALUE
 * @return uint8_t -  Delay value in ms
 */
uint8_t vtr_mon_get_vtr_pad_ctrl_ms( VTR_PAD_CTRL_VALUE pad_ctrl_bits )
{
    uint8_t pad_ctrl_ms = DISABLED;
    if(VTR_PAD_CTRL_1 == pad_ctrl_bits)
    {
        pad_ctrl_ms = VTR_PAD_CTRL_1MS;
    }
    else if(VTR_PAD_CTRL_10 == pad_ctrl_bits)
    {
        pad_ctrl_ms = VTR_PAD_CTRL_10MS;
    }
    else if(VTR_PAD_CTRL_100 == pad_ctrl_bits)
    {
        pad_ctrl_ms = VTR_PAD_CTRL_100MS;
    }

    return pad_ctrl_ms;
}

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to get VTR1 power up status                                      */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_is_vtr1_powered_up - This function gets VTR1 power up status
 * @param None
 * @return bool - true - VTR1 powered up , false - powered down
 */
bool vtr_mon_is_vtr1_powered_up()
{
    if(EC_REG_BANK_VTR1PadMonStatusGet())
    {
        return true;
    }
    else
    {
        return false;
    }
}

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to get VTR2 power up status                                      */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_is_vtr2_powered_up - This function gets VTR2 power up status
 * @param None
 * @return bool - true - VTR2 powered up , false - powered down
 */
bool vtr_mon_is_vtr2_powered_up()
{
    if(EC_REG_BANK_VTR2PadMonStatusGet())
    {
        return true;
    }
    else
    {
        return false;
    }
}

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to get VTR1 power up status                                      */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_is_vtr1_powerup_detected - This function gets VTR1 power up ststus
 * @param None
 * @return bool - true - powerup event detected , false - powerup event not detected
 */
bool vtr_mon_is_vtr1_powerup_detected()
{
    if((EC_REG_BANK_VTR1PadMonStatusGet() 
            & EC_REG_BANK_PD_MON_STS_VTR1_PU_STS_Msk))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to get VTR2 power up status                                      */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_is_vtr2_powerup_detected - This function gets VTR2 power up ststus
 * @param None
 * @return bool - true - powerup event detected , false - powerup event not detected
 */
bool vtr_mon_is_vtr2_powerup_detected()
{
    if((EC_REG_BANK_VTR2PadMonStatusGet() 
            & EC_REG_BANK_PD_MON_STS_VTR2_PU_STS_Msk))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to get VTR1 power down status                                      */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_is_vtr1_power_down_detected - This function gets VTR1 power down ststus
 * @param None
 * @return bool - true  - power down event detected ,
 *                false - power down event not detected
 */
bool vtr_mon_is_vtr1_power_down_detected()
{
    if((EC_REG_BANK_VTR1PadMonStatusGet() 
            & EC_REG_BANK_PD_MON_STS_VTR1_PD_STS_Msk))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to get VTR2 power down status                                      */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_is_vtr2_power_down_detected - This function gets VTR2 power down ststus
 * @param None
 * @return bool - true  - power down event detected ,
 *                false - power down event not detected
 */
bool vtr_mon_is_vtr2_power_down_detected()
{
    if((EC_REG_BANK_VTR2PadMonStatusGet() 
            & EC_REG_BANK_PD_MON_STS_VTR2_PD_STS_Msk))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to Enable VTR1 power up IRQ                                           */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_vtr1_powerup_irq_en - This function enables VTR1 power up IRQ
 * @param None
 * @return None
 */
void vtr_mon_vtr1_powerup_irq_en()
{
    EC_REG_BANK_VTR1PadMonPUIntEn();
}

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to Enable VTR1 power down IRQ                                           */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_vtr1_power_down_irq_en - This function enables VTR1 power down IRQ
 * @param None
 * @return None
 */
void vtr_mon_vtr1_power_down_irq_en()
{
    EC_REG_BANK_VTR1PadMonPDIntEn();
}

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to Clear VTR1 power up Status                                           */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_vtr1_powerup_sts_clr - This function clear VTR1 power up status
 * @param None
 * @return None
 */
void vtr_mon_vtr1_powerup_sts_clr()
{
    EC_REG_BANK_VTR1PadMonStatusClr(EC_REG_BANK_PD_MON_STS_VTR1_PU_STS_Msk);
}

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to Clear VTR1 power down status                                           */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_vtr1_power_down_sts_clr - This function clears VTR1 power down status
 * @param None
 * @return None
 */
void vtr_mon_vtr1_power_down_sts_clr()
{
    EC_REG_BANK_VTR1PadMonStatusClr(EC_REG_BANK_PD_MON_STS_VTR1_PD_STS_Msk);
}

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to Enable VTR2 power up IRQ                                           */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_vtr2_powerup_irq_en - This function enables VTR2 power up IRQ
 * @param None
 * @return None
 */
void vtr_mon_vtr2_powerup_irq_en()
{
    EC_REG_BANK_VTR2PadMonPUIntEn();
}

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to Enable VTR2 power down IRQ                                           */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_vtr2_power_down_irq_en - This function enables VTR2 power down IRQ
 * @param None
 * @return None
 */
void vtr_mon_vtr2_power_down_irq_en()
{
    EC_REG_BANK_VTR2PadMonPDIntEn();
}

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to Clear VTR2 power up Status                                           */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_vtr2_powerup_sts_clr - This function clear VTR2 power up status
 * @param None
 * @return None
 */
void vtr_mon_vtr2_powerup_sts_clr()
{
    EC_REG_BANK_VTR2PadMonStatusClr(EC_REG_BANK_PD_MON_STS_VTR2_PU_STS_Msk);
}

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to Clear VTR2 power down status                                           */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_vtr2_power_down_sts_clr - This function clears VTR2 power down status
 * @param None
 * @return None
 */
void vtr_mon_vtr2_power_down_sts_clr()
{
    EC_REG_BANK_VTR2PadMonStatusClr(EC_REG_BANK_PD_MON_STS_VTR2_PD_STS_Msk);
}