/*****************************************************************************
* © 2022 Microchip Technology Inc. and its subsidiaries.
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

#ifndef VTR_MON_API_H
#define VTR_MON_API_H


#ifdef __cplusplus
extern "C" {
#endif

/* VTR PAD COntrol debounce counter delay in ms */
#define VTR_PAD_CTRL_1MS    (1u)
#define VTR_PAD_CTRL_10MS   (10u)
#define VTR_PAD_CTRL_100MS  (100u)

/* VTR PAD COntrol value */
typedef enum
{
    VTR_PAD_CTRL_OFF,
    VTR_PAD_CTRL_1,
    VTR_PAD_CTRL_10,
    VTR_PAD_CTRL_100,
    VTR_PAD_CTRL_MAX,
} VTR_PAD_CTRL_VALUE;

/* Number of VTR's   */
typedef enum
{
    VTR1,
    VTR2,
    VTR_MAX
} VTR;


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
uint8_t vtr_mon_get_vtr_pad_ctrl_ms( VTR_PAD_CTRL_VALUE pad_ctrl_bits );

/* --------------------------------------------------------------------------------------------- */
/*  API Function - Function to write the PAD_CTRL_VALUE for VTR1                                 */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_vtr1_wr_pad_ctrl - Writes the given Pad Ctrl value
 * for VTR1 to PAD Ctrl register.
 * @param pad_ctrl - Refen enum  VTR_PAD_CTRL_VALUE
 * @return None
 */
void vtr_mon_vtr1_wr_pad_ctrl(VTR_PAD_CTRL_VALUE pad_ctrl);

/* --------------------------------------------------------------------------------------------- */
/*  API Function - Function to write the PAD_CTRL_VALUE for VTR2                                 */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_vtr2_wr_pad_ctrl - Writes the given Pad Ctrl value
 * for VTR2 to PAD Ctrl register.
 * @param pad_ctrl - Refen enum  VTR_PAD_CTRL_VALUE
 * @return None
 */
void vtr_mon_vtr2_wr_pad_ctrl(VTR_PAD_CTRL_VALUE pad_ctrl);

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to get VTR1 power up status                                      */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_is_vtr1_powered_up - This function gets VTR1 power up status
 * @param None
 * @return bool - true - VTR1 powered up , false - powered down
 */
bool vtr_mon_is_vtr1_powered_up();

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to get VTR2 power up status                                      */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_is_vtr2_powered_up - This function gets VTR2 power up status
 * @param None
 * @return bool - true - VTR2 powered up , false - powered down
 */
bool vtr_mon_is_vtr2_powered_up();

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to get VTR1 power up status                                      */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_is_vtr1_powerup_detected - This function gets VTR1 power up ststus
 * @param None
 * @return bool - true - powerup event detected , false - powerup event not detected
 */
bool vtr_mon_is_vtr1_powerup_detected();

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to get VTR2 power up status                                      */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_is_vtr2_powerup_detected - This function gets VTR2 power up ststus
 * @param None
 * @return bool - true - powerup event detected , false - powerup event not detected
 */
bool vtr_mon_is_vtr2_powerup_detected();

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to get VTR1 power down status                                      */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_is_vtr1_power_down_detected - This function gets VTR1 power down ststus
 * @param None
 * @return bool - true  - power down event detected ,
 *                false - power down event not detected
 */
bool vtr_mon_is_vtr1_power_down_detected();

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to get VTR2 power down status                                      */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_is_vtr2_power_down_detected - This function gets VTR2 power down ststus
 * @param None
 * @return bool - true  - power down event detected ,
 *                false - power down event not detected
 */
bool vtr_mon_is_vtr2_power_down_detected();

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to Enable VTR1 power up IRQ                                           */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_vtr1_powerup_irq_en - This function enables VTR1 power up IRQ
 * @param None
 * @return None
 */
void vtr_mon_vtr1_powerup_irq_en();

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to Enable VTR1 power down IRQ                                           */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_vtr1_power_down_irq_en - This function enables VTR1 power down IRQ
 * @param None
 * @return None
 */
void vtr_mon_vtr1_power_down_irq_en();

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to Clear VTR1 power up Status                                           */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_vtr1_powerup_sts_clr - This function clear VTR1 power up status
 * @param None
 * @return None
 */
void vtr_mon_vtr1_powerup_sts_clr();

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to Clear VTR1 power down status                                           */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_vtr1_power_down_sts_clr - This function clears VTR1 power down status
 * @param None
 * @return None
 */
void vtr_mon_vtr1_power_down_sts_clr();

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to Enable VTR2 power up IRQ                                           */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_vtr2_powerup_irq_en - This function enables VTR2 power up IRQ
 * @param None
 * @return None
 */
void vtr_mon_vtr2_powerup_irq_en();

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to Enable VTR2 power down IRQ                                           */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_vtr2_power_down_irq_en - This function enables VTR2 power down IRQ
 * @param None
 * @return None
 */
void vtr_mon_vtr2_power_down_irq_en();

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to Clear VTR2 power up Status                                           */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_vtr2_powerup_sts_clr - This function clear VTR2 power up status
 * @param None
 * @return None
 */
void vtr_mon_vtr2_powerup_sts_clr();

/* --------------------------------------------------------------------------------------------- */
/* API Function - Function to Clear VTR2 power down status                                           */
/* --------------------------------------------------------------------------------------------- */
/**
 * vtr_mon_vtr2_power_down_sts_clr - This function clears VTR2 power down status
 * @param None
 * @return None
 */
void vtr_mon_vtr2_power_down_sts_clr();

/* --------------------------------------------------------------------------------------------- */
/*  Peripheral Function - Function to write the PAD_CTRL_VALUE for VTR1/VTR2 */
/* --------------------------------------------------------------------------------------------- */
/**
 * p_vtr_mon_wr_pad_ctrl_vtr1 - Writes the given Pad Ctrl value
 * for given VTR to PAD Ctrl register.
 * @param vtr - Refen enum  VTR
 * @param pad_ctrl - Refen enum  VTR_PAD_CTRL_VALUE
 * @return None
 */
void p_vtr_mon_wr_pad_ctrl(VTR vtr, VTR_PAD_CTRL_VALUE pad_ctrl);

/* --------------------------------------------------------------------------------------------- */
/*  Peripheral Function - Function to get the VRT1/VTR2 current status  */
/* --------------------------------------------------------------------------------------------- */
/**
 * p_vtr_mon_get_vtr_status - gets current state of  the Power up status for given VTR
 * from VTR_MON_STATUS register.
 * @param vtr - Refer enum VTR
 * @return uint32_t - VTRCurrent status bit mask
 */
uint32_t p_vtr_mon_get_vtr_status(VTR vtr);

/* --------------------------------------------------------------------------------------------- */
/*  Peripheral Function - Function to get the VRT1/VTR2 power up event status  */
/* --------------------------------------------------------------------------------------------- */
/**
 * p_vtr_mon_get_powerup_event - gets the Power up status for given VTR
 * from VTR_MON_STATUS register.
 * @param vtr - Refer enum VTR
 * @return uint32_t - Power up status bit mask
 */
uint32_t p_vtr_mon_get_powerup_event(VTR vtr);

/* --------------------------------------------------------------------------------------------- */
/*  Peripheral Function - Function to get the VRT1/VTR2 power down event status  */
/* --------------------------------------------------------------------------------------------- */
/**
 * p_vtr_mon_get_power_down_event - gets the Power down status for given VTR
 * from VTR_MON_STATUS register.
 * @param vtr - Refer enum VTR
 * @return uint32_t - Power down status bit mask
 */
uint32_t p_vtr_mon_get_power_down_event(VTR vtr);

/* --------------------------------------------------------------------------------------------- */
/*  Peripheral Function - Function Enable VTR1/VTR2 power up IRQ  */
/* --------------------------------------------------------------------------------------------- */
/**
 * p_vtr_mon_powerup_irq_en - Enables the Power up IRQ for given VTR
 * @param vtr - Refer enum VTR
 * @return None
 */
void p_vtr_mon_powerup_irq_en(VTR vtr);

/* --------------------------------------------------------------------------------------------- */
/*  Peripheral Function - Function Disables VTR1/VTR2 power up IRQ  */
/* --------------------------------------------------------------------------------------------- */
/**
 * p_vtr_mon_powerup_irq_dis - Disable the Power up IRQ for given VTR
 * @param vtr - Refer enum VTR
 * @return None
 */
void p_vtr_mon_powerup_irq_dis(VTR vtr);

/* --------------------------------------------------------------------------------------------- */
/*  Peripheral Function - Function Enable VTR1/VTR2 power down IRQ  */
/* --------------------------------------------------------------------------------------------- */
/**
 * p_vtr_mon_power_down_irq_en - Enables the Power down IRQ for given VTR
 * @param vtr - Refer enum VTR
 * @return None
 */
void p_vtr_mon_power_down_irq_en(VTR vtr);

/* --------------------------------------------------------------------------------------------- */
/*  Peripheral Function - Function Disables VTR1/VTR2 power down IRQ  */
/* --------------------------------------------------------------------------------------------- */
/**
 * p_vtr_mon_power_down_irq_dis - Disable the Power down IRQ for given VTR
 * @param vtr - Refer enum VTR
 * @return None
 */
void p_vtr_mon_power_down_irq_dis(VTR vtr);

/* --------------------------------------------------------------------------------------------- */
/*  Peripheral Function - Function Clear VTR1/VTR2 power-up status */
/* --------------------------------------------------------------------------------------------- */
/**
 * p_vtr_mon_powerup_sts_clr - Clears the power-up status for the given VTR
 * @param vtr - Refer enum VTR
 * @return None
 */
void p_vtr_mon_powerup_sts_clr(VTR vtr);

/* --------------------------------------------------------------------------------------------- */
/*  Peripheral Function - Function Clear VTR1/VTR2 power down status */
/* --------------------------------------------------------------------------------------------- */
/**
 * p_vtr_mon_power_down_sts_clr - Clears the power down status for the given VTR
 * @param vtr - Refer enum VTR
 * @return None
 */
void p_vtr_mon_power_down_sts_clr(VTR vtr);

#ifdef __cplusplus
}
#endif

#endif