/*******************************************************************************
  Inter-Integrated Circuit (I2C) Library
  Source File

  Company:
    Microchip Technology Inc.

  File Name:
    plib_smb3_master_slave_common.c

  Summary:
    I2C PLIB Master-Slave Mode Common Implementation file

  Description:
    This file defines the interface to the I2C peripheral library.
    This library provides access to and control of the associated peripheral
    instance.

*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018-2019 Microchip Technology Inc. and its subsidiaries.
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
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "device.h"
#include "interrupts.h"
#include "plib_i2c_smb_common.h"
#include "plib_smb3_master_slave_common.h"
#include "master/plib_smb3_master.h"
#include "slave/plib_smb3_slave.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************
#define NOP()    asm("NOP")

void I2CSMB3_Initialize(void)
{
    /* Reset I2C */
    SMB3_REGS->SMB_CFG[0] = SMB_CFG_RST_Msk;

    /* Reset bit must remain asserted for at-least 1 Baud clock period */
    NOP();NOP();NOP();NOP();NOP();

    SMB3_REGS->SMB_CFG[0] &= ~SMB_CFG_RST_Msk;

    /* Set the port associated with this instance of I2C peripheral */
    SMB3_REGS->SMB_CFG[0] = SMB_CFG_PORT_SEL(9);

    I2CSMB3_HostInitialize();

    I2CSMB3_TargetInitialize();

    SMB3_REGS->SMB_PEC = 0;

    /* Repeated start hold time setup */
    SMB3_REGS->SMB_RSHTM = I2C_SMB_RecommendedValues[2][1];

    /* Data timing register */
    SMB3_REGS->SMB_DATATM = I2C_SMB_RecommendedValues[1][1];


    /* Enable I2C peripheral */
    SMB3_REGS->SMB_CFG[0] |= SMB_CFG_EN_Msk | SMB_CFG_PECEN_Msk  ;

    /* Enable Serial output and set ACK bit */
    SMB3_REGS->SMB_WCTRL = (SMB_WCTRL_ESO_Msk | SMB_WCTRL_ACK_Msk);
}

