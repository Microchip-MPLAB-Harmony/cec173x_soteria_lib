# coding: utf-8
"""*****************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
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
*****************************************************************************"""

def instantiateComponent(sg3LibComponent):

    global sg3InstanceName

    sg3InstanceName = sg3LibComponent.createStringSymbol("SG3_INSTANCE_NAME", None)
    sg3InstanceName.setVisible(False)
    sg3InstanceName.setDefaultValue(sg3LibComponent.getID().upper())

    #SG3 library
    configName = Variables.get("__CONFIGURATION_NAME")

    sg3LibFile = sg3LibComponent.createLibrarySymbol("SG3_FILE_STATIC_LIB", None)

    sg3LibFile.setSourcePath("/libraries/cec173x_sg3/templates/secureboot_app_lib.X.a")
    sg3LibFile.setOutputName("secureboot_app_lib.a")
    sg3LibFile.setDestPath("library/secureboot_app_lib")
    
    #SG3 dep library    
    configName = Variables.get("__CONFIGURATION_NAME")

    sg3DepLibFile = sg3LibComponent.createLibrarySymbol("SG3_FILE_EFUSE_STATIC_LIB", None)

    sg3DepLibFile.setSourcePath("/libraries/cec173x_sg3/templates/secureboot_LIB.X.a")
    sg3DepLibFile.setOutputName("secureboot_LIB.a")
    sg3DepLibFile.setDestPath("library/secureboot_LIB")

