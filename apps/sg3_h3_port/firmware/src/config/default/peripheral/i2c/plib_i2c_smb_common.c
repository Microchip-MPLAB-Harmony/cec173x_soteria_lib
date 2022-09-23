/*******************************************************************************
  Inter-Integrated Circuit (I2C) Library
  Source File

  Company:
    Microchip Technology Inc.

  File Name:
    plib_i2c_smbus_common.c

  Summary:
    I2C SMBUS PLIB Common Data file

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

#include "plib_i2c_smb_common.h"

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

//Recommended values with 16 MHz Baud Clock
const unsigned int I2C_SMB_RecommendedValues[][3] =
{
    //100KHz,           400KHz,         1MHz
    {0x00004F4F,        0x00000F17,     0x00000509},    //Bus Clock Register
    {0x0C4D5006,        0x040A0A06,     0x04060601},    //Data Timing Register
    {0x0000004D,        0x0000000A,     0x00000006},    //Repeated START Hold Time Register
    {0x01FC01ED,        0x01000050,     0x01000050},    //Idle Scaling Register
    {0x4B9CC2C7,        0x159CC2C7,     0x089CC2C7},    //Time-Out Scaling Register
};

