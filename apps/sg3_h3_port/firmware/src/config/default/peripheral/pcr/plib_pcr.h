/*******************************************************************************
  PCR PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_pcr.h

  Summary:
    PCR PLIB Header File.

  Description:
    The PCR PLIB initializes all the oscillators based on the
    requirements.

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

#ifndef PLIB_PCR_H
#define PLIB_PCR_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/* This section lists the other files that are included in this file.
*/
#include <device.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus // Provide C++ Compatibility
extern "C" {
#endif

// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************


typedef enum
{
    PCR_SLEEP_EN0_STAP_SLP_EN = PCR_SLP_EN_0_STAP_SLP_EN_Msk,
	PCR_SLEEP_EN0_OTP_SLP_EN = PCR_SLP_EN_0_OTP_SLP_EN_Msk,
	PCR_SLEEP_EN0_IMSPI_SLP_EN = PCR_SLP_EN_0_IMSPI_SLP_EN_Msk,
	PCR_SLEEP_EN0_CHPTST_SLP_EN = PCR_SLP_EN_0_CHPTST_SLP_EN_Msk,
	PCR_SLEEP_EN0_HRBNK_SLP_EN = PCR_SLP_EN_0_HRBNK_SLP_EN_Msk,
	PCR_SLEEP_EN0_TSTSPI_SLP_EN = PCR_SLP_EN_0_TSTSPI_SLP_EN_Msk,
	PCR_SLEEP_EN0_GPIO_SLP_EN = PCR_SLP_EN_0_GPIO_SLP_EN_Msk,
	PCR_SLEEP_EN0_PCR_SLP_EN = PCR_SLP_EN_0_PCR_SLP_EN_Msk,
	

}PCR_SLEEP_EN0;

typedef enum
{
    PCR_SLEEP_EN1_INT_SLP_EN = PCR_SLP_EN_1_INT_SLP_EN_Msk,
	PCR_SLEEP_EN1_PWM0_SLP_EN = PCR_SLP_EN_1_PWM0_SLP_EN_Msk,
	PCR_SLEEP_EN1_PMC_SLP_EN = PCR_SLP_EN_1_PMC_SLP_EN_Msk,
	PCR_SLEEP_EN1_DMA_SLP_EN = PCR_SLP_EN_1_DMA_SLP_EN_Msk,
	PCR_SLEEP_EN1_TFDP_SLP_EN = PCR_SLP_EN_1_TFDP_SLP_EN_Msk,
	PCR_SLEEP_EN1_PROC_SLP_EN = PCR_SLP_EN_1_PROC_SLP_EN_Msk,
	PCR_SLEEP_EN1_WDT_SLP_EN = PCR_SLP_EN_1_WDT_SLP_EN_Msk,
	PCR_SLEEP_EN1_SMB0_SLP_EN = PCR_SLP_EN_1_SMB0_SLP_EN_Msk,
	PCR_SLEEP_EN1_EC_REG_BANK_SLP_EN = PCR_SLP_EN_1_EC_REG_BANK_SLP_EN_Msk,
	PCR_SLEEP_EN1_TMR32_0_SLP_EN = PCR_SLP_EN_1_TMR32_0_SLP_EN_Msk,
	PCR_SLEEP_EN1_TMR32_1_SLP_EN = PCR_SLP_EN_1_TMR32_1_SLP_EN_Msk,
	

}PCR_SLEEP_EN1;

typedef enum
{
    PCR_SLEEP_EN3_HTM_0_SLP_EN = PCR_SLP_EN_3_HTM_0_SLP_EN_Msk,
	PCR_SLEEP_EN3_SMB1_SLP_EN = PCR_SLP_EN_3_SMB1_SLP_EN_Msk,
	PCR_SLEEP_EN3_SMB2_SLP_EN = PCR_SLP_EN_3_SMB2_SLP_EN_Msk,
	PCR_SLEEP_EN3_SMB3_SLP_EN = PCR_SLP_EN_3_SMB3_SLP_EN_Msk,
	PCR_SLEEP_EN3_LED0_SLP_EN = PCR_SLP_EN_3_LED0_SLP_EN_Msk,
	PCR_SLEEP_EN3_LED1_SLP_EN = PCR_SLP_EN_3_LED1_SLP_EN_Msk,
	PCR_SLEEP_EN3_SMB4_SLP_EN = PCR_SLP_EN_3_SMB4_SLP_EN_Msk,
	PCR_SLEEP_EN3_CRYPTO_SLP_EN = PCR_SLP_EN_3_CRYPTO_SLP_EN_Msk,
	PCR_SLEEP_EN3_HTM_1_SLP_EN = PCR_SLP_EN_3_HTM_1_SLP_EN_Msk,
	PCR_SLEEP_EN3_CCT_SLP_EN = PCR_SLP_EN_3_CCT_SLP_EN_Msk,
	

}PCR_SLEEP_EN3;

typedef enum
{
    PCR_SLEEP_EN4_SECMON0_SLP_EN = PCR_SLP_EN_4_SECMON0_SLP_EN_Msk,
	PCR_SLEEP_EN4_SECMON1_SLP_EN = PCR_SLP_EN_4_SECMON1_SLP_EN_Msk,
	PCR_SLEEP_EN4_RTOS_SLP_EN = PCR_SLP_EN_4_RTOS_SLP_EN_Msk,
	PCR_SLEEP_EN4_QMSPI0_SLP_EN = PCR_SLP_EN_4_QMSPI0_SLP_EN_Msk,
	PCR_SLEEP_EN4_UART0_SLP_EN = PCR_SLP_EN_4_UART0_SLP_EN_Msk,
	PCR_SLEEP_EN4_SPIPER0_SLP_EN = PCR_SLP_EN_4_SPIPER0_SLP_EN_Msk,
	PCR_SLEEP_EN4_SPIPER1_SLP_EN = PCR_SLP_EN_4_SPIPER1_SLP_EN_Msk,
	PCR_SLEEP_EN4_QMSPI_1_SLP_EN = PCR_SLP_EN_4_QMSPI_1_SLP_EN_Msk,
	PCR_SLEEP_EN4_VBAT_REG_SLP_EN = PCR_SLP_EN_4_VBAT_REG_SLP_EN_Msk,
	

}PCR_SLEEP_EN4;


typedef enum
{
    PCR_PRIV_EN0_OTP = PCR_EC_PRIV_EN0_OTP_Msk,
	PCR_PRIV_EN0_HOST_REG = PCR_EC_PRIV_EN0_HOST_REG_Msk,
	PCR_PRIV_EN0_TST_SPI = PCR_EC_PRIV_EN0_TST_SPI_Msk,
	PCR_PRIV_EN0_GPIO = PCR_EC_PRIV_EN0_GPIO_Msk,
	PCR_PRIV_EN0_PCR = PCR_EC_PRIV_EN0_PCR_Msk,
	

}PCR_PRIV_EN0;

typedef enum
{
    PCR_PRIV_EN1_INTR = PCR_EC_PRIV_EN1_INTR_Msk,
	PCR_PRIV_EN1_PWM0 = PCR_EC_PRIV_EN1_PWM0_Msk,
	PCR_PRIV_EN1_PMC = PCR_EC_PRIV_EN1_PMC_Msk,
	PCR_PRIV_EN1_DMA = PCR_EC_PRIV_EN1_DMA_Msk,
	PCR_PRIV_EN1_TFDP = PCR_EC_PRIV_EN1_TFDP_Msk,
	PCR_PRIV_EN1_WDT = PCR_EC_PRIV_EN1_WDT_Msk,
	PCR_PRIV_EN1_SMB_I2C0 = PCR_EC_PRIV_EN1_SMB_I2C0_Msk,
	PCR_PRIV_EN1_EC_REGS = PCR_EC_PRIV_EN1_EC_REGS_Msk,
	PCR_PRIV_EN1_BASIC_TMR0 = PCR_EC_PRIV_EN1_BASIC_TMR0_Msk,
	PCR_PRIV_EN1_BASIC_TMR1 = PCR_EC_PRIV_EN1_BASIC_TMR1_Msk,
	

}PCR_PRIV_EN1;

typedef enum
{
    PCR_PRIV_EN3_HIB_TIM0 = PCR_EC_PRIV_EN3_HIB_TIM0_Msk,
	PCR_PRIV_EN3_SMB_I2C1 = PCR_EC_PRIV_EN3_SMB_I2C1_Msk,
	PCR_PRIV_EN3_SMB_I2C2 = PCR_EC_PRIV_EN3_SMB_I2C2_Msk,
	PCR_PRIV_EN3_SMB_I2C3 = PCR_EC_PRIV_EN3_SMB_I2C3_Msk,
	PCR_PRIV_EN3_LED0 = PCR_EC_PRIV_EN3_LED0_Msk,
	PCR_PRIV_EN3_LED1 = PCR_EC_PRIV_EN3_LED1_Msk,
	PCR_PRIV_EN3_SMB_I2C4 = PCR_EC_PRIV_EN3_SMB_I2C4_Msk,
	PCR_PRIV_EN3_CRYPTO = PCR_EC_PRIV_EN3_CRYPTO_Msk,
	PCR_PRIV_EN3_HIB_TIM1 = PCR_EC_PRIV_EN3_HIB_TIM1_Msk,
	PCR_PRIV_EN3_CCT0 = PCR_EC_PRIV_EN3_CCT0_Msk,
	

}PCR_PRIV_EN3;

typedef enum
{
    PCR_PRIV_EN4_SPIMON0 = PCR_EC_PRIV_EN4_SPIMON0_Msk,
	PCR_PRIV_EN4_SPIMON1 = PCR_EC_PRIV_EN4_SPIMON1_Msk,
	PCR_PRIV_EN4_RTOS_TIM = PCR_EC_PRIV_EN4_RTOS_TIM_Msk,
	PCR_PRIV_EN4_QMSPI0 = PCR_EC_PRIV_EN4_QMSPI0_Msk,
	PCR_PRIV_EN4_UART0 = PCR_EC_PRIV_EN4_UART0_Msk,
	PCR_PRIV_EN4_SPISLV0 = PCR_EC_PRIV_EN4_SPISLV0_Msk,
	PCR_PRIV_EN4_SPISLV1 = PCR_EC_PRIV_EN4_SPISLV1_Msk,
	PCR_PRIV_EN4_QMSPI1 = PCR_EC_PRIV_EN4_QMSPI1_Msk,
	PCR_PRIV_EN4_VBAT_REG = PCR_EC_PRIV_EN4_VBAT_REG_Msk,
	

}PCR_PRIV_EN4;


typedef enum
{
    PCR_RESET_EN0_JTAG_STAP_CLK_REQ = PCR_RST_EN_0_JTAG_STAP_CLK_REQ_Msk,
	PCR_RESET_EN0_OTP_RST_EN = PCR_RST_EN_0_OTP_RST_EN_Msk,
	PCR_RESET_EN0_CHPTST_RST_EN = PCR_RST_EN_0_CHPTST_RST_EN_Msk,
	PCR_RESET_EN0_TSTSPI_RST_EN = PCR_RST_EN_0_TSTSPI_RST_EN_Msk,
	PCR_RESET_EN0_GPIO_RST_EN = PCR_RST_EN_0_GPIO_RST_EN_Msk,
	PCR_RESET_EN0_PCR_RST_EN = PCR_RST_EN_0_PCR_RST_EN_Msk,
	

}PCR_RESET_EN0;

typedef enum
{
    PCR_RESET_EN1_INT_RST_EN = PCR_RST_EN_1_INT_RST_EN_Msk,
	PCR_RESET_EN1_PWM0_RST_EN = PCR_RST_EN_1_PWM0_RST_EN_Msk,
	PCR_RESET_EN1_DMA_RST_EN = PCR_RST_EN_1_DMA_RST_EN_Msk,
	PCR_RESET_EN1_TFDP_RST_EN = PCR_RST_EN_1_TFDP_RST_EN_Msk,
	PCR_RESET_EN1_WDT_RST_EN = PCR_RST_EN_1_WDT_RST_EN_Msk,
	PCR_RESET_EN1_SMB0_RST_EN = PCR_RST_EN_1_SMB0_RST_EN_Msk,
	PCR_RESET_EN1_TMR32_0_RST_EN = PCR_RST_EN_1_TMR32_0_RST_EN_Msk,
	PCR_RESET_EN1_TMR32_1_RST_EN = PCR_RST_EN_1_TMR32_1_RST_EN_Msk,
	

}PCR_RESET_EN1;

typedef enum
{
    PCR_RESET_EN3_HTM_0_RST_EN = PCR_RST_EN_3_HTM_0_RST_EN_Msk,
	PCR_RESET_EN3_SMB1_RST_EN = PCR_RST_EN_3_SMB1_RST_EN_Msk,
	PCR_RESET_EN3_SMB2_RST_EN = PCR_RST_EN_3_SMB2_RST_EN_Msk,
	PCR_RESET_EN3_SMB3_RST_EN = PCR_RST_EN_3_SMB3_RST_EN_Msk,
	PCR_RESET_EN3_LED0_RST_EN = PCR_RST_EN_3_LED0_RST_EN_Msk,
	PCR_RESET_EN3_LED1_RST_EN = PCR_RST_EN_3_LED1_RST_EN_Msk,
	PCR_RESET_EN3_SMB_4_RST_EN = PCR_RST_EN_3_SMB_4_RST_EN_Msk,
	PCR_RESET_EN3_CRYPTO_RST_EN = PCR_RST_EN_3_CRYPTO_RST_EN_Msk,
	PCR_RESET_EN3_HTM_1_RST_EN = PCR_RST_EN_3_HTM_1_RST_EN_Msk,
	PCR_RESET_EN3_CCTIMER_RST_EN = PCR_RST_EN_3_CCTIMER_RST_EN_Msk,
	

}PCR_RESET_EN3;

typedef enum
{
    PCR_RESET_EN4_SECMON0_RST_EN = PCR_RST_EN_4_SECMON0_RST_EN_Msk,
	PCR_RESET_EN4_SECMON1_RST_EN = PCR_RST_EN_4_SECMON1_RST_EN_Msk,
	PCR_RESET_EN4_RTOS_RST_EN = PCR_RST_EN_4_RTOS_RST_EN_Msk,
	PCR_RESET_EN4_QMSPI0_RST_EN = PCR_RST_EN_4_QMSPI0_RST_EN_Msk,
	PCR_RESET_EN4_UART0_RST_EN = PCR_RST_EN_4_UART0_RST_EN_Msk,
	PCR_RESET_EN4_SPIPER0_RST_EN = PCR_RST_EN_4_SPIPER0_RST_EN_Msk,
	PCR_RESET_EN4_SPIPER1_RST_EN = PCR_RST_EN_4_SPIPER1_RST_EN_Msk,
	PCR_RESET_EN4_QMSPI_1_RST_EN = PCR_RST_EN_4_QMSPI_1_RST_EN_Msk,
	PCR_RESET_EN4_VBAT_REG_RST_EN = PCR_RST_EN_4_VBAT_REG_RST_EN_Msk,
	

}PCR_RESET_EN4;


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
/* The following functions make up the methods (set of possible operations) of
this interface.
*/

// *****************************************************************************
/* Function:
    void PCR_Initialize (void);

  Summary:
    Initializes all the modules related to PCR.

  Description:
    This function initializes the clock as defined by the MCC and Clock Manager
    selections.

  Precondition:
    MCC GUI should be configured with the right values. Incorrect configuration
    of the Clock will result in incorrect peripheral behavior or a non
    functional device.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
        PCR_Initialize();
    </code>

  Remarks:
    This function should be called before calling other Clock library functions.
*/

void PCR_Initialize (void);

void PCR_PeripheralResetLock (void);

void PCR_PeripheralResetUnLock (void);

void PCR_PrivilegeEnLock (void);

void PCR_PrivilegeEnUnLock (void);

void PCR_SleepEnable0 (PCR_SLEEP_EN0 blockId);
void PCR_SleepDisable0 (PCR_SLEEP_EN0 blockId);
void PCR_SleepEnable1 (PCR_SLEEP_EN1 blockId);
void PCR_SleepDisable1 (PCR_SLEEP_EN1 blockId);
void PCR_SleepEnable3 (PCR_SLEEP_EN3 blockId);
void PCR_SleepDisable3 (PCR_SLEEP_EN3 blockId);
void PCR_SleepEnable4 (PCR_SLEEP_EN4 blockId);
void PCR_SleepDisable4 (PCR_SLEEP_EN4 blockId);

void PCR_PrivilegeEnable0 (PCR_PRIV_EN0 blockId);
void PCR_PrivilegeDisable0 (PCR_PRIV_EN0 blockId);
void PCR_PrivilegeEnable1 (PCR_PRIV_EN1 blockId);
void PCR_PrivilegeDisable1 (PCR_PRIV_EN1 blockId);
void PCR_PrivilegeEnable3 (PCR_PRIV_EN3 blockId);
void PCR_PrivilegeDisable3 (PCR_PRIV_EN3 blockId);
void PCR_PrivilegeEnable4 (PCR_PRIV_EN4 blockId);
void PCR_PrivilegeDisable4 (PCR_PRIV_EN4 blockId);

void PCR_ResetEnable0 (PCR_RESET_EN0 blockId);
void PCR_ResetEnable1 (PCR_RESET_EN1 blockId);
void PCR_ResetEnable3 (PCR_RESET_EN3 blockId);
void PCR_ResetEnable4 (PCR_RESET_EN4 blockId);


#ifdef __cplusplus // Provide C++ Compatibility
}
#endif

#endif /* PLIB_CLOCK_H */
