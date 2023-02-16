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
def destroyComponent(sg3LibComponent):
    Database.sendMessage("MCTP", "SOTERIA_CONNECTED", {"isConnected":False})

def instantiateComponent(sg3LibComponent):
    global sg3InstanceName

    Database.sendMessage("MCTP", "SOTERIA_CONNECTED", {"isConnected":True})

    sg3InstanceName = sg3LibComponent.createStringSymbol("SG3_INSTANCE_NAME", None)
    sg3InstanceName.setVisible(False)
    sg3InstanceName.setDefaultValue(sg3LibComponent.getID().upper())

    moduleRoot = Variables.get("__MODULE_ROOT")
    userGuideString = "**** Please refer documentation located in " + moduleRoot + "\docs\ ****"
    userGuideString = userGuideString.replace("\\", "/")
    sg3ModuleRoot = sg3LibComponent.createCommentSymbol("SG3_LIB_DOCS_LOC", None)
    sg3ModuleRoot.setLabel(userGuideString)

    sg3UtilsLinkString = "**** Utilities available at https://github.com/MicrochipTech/cec173x_soteria_utilities ****"
    sg3UtilsLink = sg3LibComponent.createCommentSymbol("SG3_LIB_UTILS_LOC", None)
    sg3UtilsLink.setLabel(sg3UtilsLinkString)

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

    #SG3 library helper files
    #Add common/debug/serial.c
    mctpSmbusTaskSourceFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskSourceFile.setSourcePath("/libraries/cec173x_sg3/src/common/debug/serial.c")
    mctpSmbusTaskSourceFile.setDestPath("../../common/debug")
    mctpSmbusTaskSourceFile.setOutputName("serial.c")
    mctpSmbusTaskSourceFile.setProjectPath("common/debug/")
    mctpSmbusTaskSourceFile.setOverwrite(True)
    mctpSmbusTaskSourceFile.setType("SOURCE")
    mctpSmbusTaskSourceFile.setMarkup(False)

    #Add common/debug/trace.h
    mctpSmbusTaskSourceFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskSourceFile.setSourcePath("/libraries/cec173x_sg3/src/common/debug/trace.h")
    mctpSmbusTaskSourceFile.setDestPath("../../common/debug")
    mctpSmbusTaskSourceFile.setOutputName("trace.h")
    mctpSmbusTaskSourceFile.setProjectPath("common/debug/")
    mctpSmbusTaskSourceFile.setOverwrite(True)
    mctpSmbusTaskSourceFile.setType("HEADER")
    mctpSmbusTaskSourceFile.setMarkup(False)

    #Add common/include/ahb_api_mpu.h
    mctpSmbusTaskSourceFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskSourceFile.setSourcePath("/libraries/cec173x_sg3/src/common/include/ahb_api_mpu.h")
    mctpSmbusTaskSourceFile.setDestPath("../../common/include")
    mctpSmbusTaskSourceFile.setOutputName("ahb_api_mpu.h")
    mctpSmbusTaskSourceFile.setProjectPath("common/include/")
    mctpSmbusTaskSourceFile.setOverwrite(True)
    mctpSmbusTaskSourceFile.setType("HEADER")
    mctpSmbusTaskSourceFile.setMarkup(False)

    #Add common/include/common.h
    mctpSmbusTaskSourceFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskSourceFile.setSourcePath("/libraries/cec173x_sg3/src/common/include/common.h")
    mctpSmbusTaskSourceFile.setDestPath("../../common/include")
    mctpSmbusTaskSourceFile.setOutputName("common.h")
    mctpSmbusTaskSourceFile.setProjectPath("common/include/")
    mctpSmbusTaskSourceFile.setOverwrite(True)
    mctpSmbusTaskSourceFile.setType("HEADER")
    mctpSmbusTaskSourceFile.setMarkup(False)

    #Add common/include/rom_api_mpu.h
    mctpSmbusTaskSourceFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskSourceFile.setSourcePath("/libraries/cec173x_sg3/src/common/include/rom_api_mpu.h")
    mctpSmbusTaskSourceFile.setDestPath("../../common/include")
    mctpSmbusTaskSourceFile.setOutputName("rom_api_mpu.h")
    mctpSmbusTaskSourceFile.setProjectPath("common/include/")
    mctpSmbusTaskSourceFile.setOverwrite(True)
    mctpSmbusTaskSourceFile.setType("HEADER")
    mctpSmbusTaskSourceFile.setMarkup(False)

    #Add common/include/secureboot_app.ld
    mctpSmbusTaskSourceFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskSourceFile.setSourcePath("/libraries/cec173x_sg3/src/common/include/secureboot_app.ld")
    mctpSmbusTaskSourceFile.setDestPath("../../common/include")
    mctpSmbusTaskSourceFile.setOutputName("secureboot_app.ld")  
    mctpSmbusTaskSourceFile.setProjectPath("config/" + configName)
    mctpSmbusTaskSourceFile.setOverwrite(True)
    mctpSmbusTaskSourceFile.setType("LINKER")
    mctpSmbusTaskSourceFile.setMarkup(False)

    #Add common/include/pmci.h
    mctpSmbusTaskSourceFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskSourceFile.setSourcePath("/libraries/cec173x_sg3/src/common/include/pmci.h")
    mctpSmbusTaskSourceFile.setDestPath("../../common/include")
    mctpSmbusTaskSourceFile.setOutputName("pmci.h")  
    mctpSmbusTaskSourceFile.setProjectPath("common/include/")
    mctpSmbusTaskSourceFile.setOverwrite(True)
    mctpSmbusTaskSourceFile.setType("HEADER")
    mctpSmbusTaskSourceFile.setMarkup(False)

    #Add common/include/FreeRTOS.h
    mctpSmbusTaskSourceFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskSourceFile.setSourcePath("/libraries/cec173x_sg3/src/common/include/FreeRTOS.h")
    mctpSmbusTaskSourceFile.setDestPath("../../common/include")
    mctpSmbusTaskSourceFile.setOutputName("FreeRTOS.h")  
    mctpSmbusTaskSourceFile.setProjectPath("common/include/")
    mctpSmbusTaskSourceFile.setOverwrite(True)
    mctpSmbusTaskSourceFile.setType("HEADER")
    mctpSmbusTaskSourceFile.setMarkup(False)

    #Add common/data_iso/data_iso_checks.h
    mctpSmbusTaskSourceFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskSourceFile.setSourcePath("/libraries/cec173x_sg3/src/data_iso/data_iso_checks_rom_api.h")
    mctpSmbusTaskSourceFile.setDestPath("../../data_iso")
    mctpSmbusTaskSourceFile.setOutputName("data_iso_checks_rom_api.h")  
    mctpSmbusTaskSourceFile.setProjectPath("data_iso/")
    mctpSmbusTaskSourceFile.setOverwrite(True)
    mctpSmbusTaskSourceFile.setType("HEADER")
    mctpSmbusTaskSourceFile.setMarkup(False)

    #Add hal/gpio/gpio_api.c
    mctpSmbusTaskSourceFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskSourceFile.setSourcePath("/libraries/cec173x_sg3/src/hal/gpio/gpio_api.c")
    mctpSmbusTaskSourceFile.setDestPath("../../hal/gpio")
    mctpSmbusTaskSourceFile.setOutputName("gpio_api.c")
    mctpSmbusTaskSourceFile.setProjectPath("hal/gpio/")
    mctpSmbusTaskSourceFile.setOverwrite(True)
    mctpSmbusTaskSourceFile.setType("SOURCE")
    mctpSmbusTaskSourceFile.setMarkup(False)

    #Add hal/gpio/gpio_api.h
    mctpSmbusTaskHeaderFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskHeaderFile.setSourcePath("/libraries/cec173x_sg3/src/hal/gpio/gpio_api.h")
    mctpSmbusTaskHeaderFile.setDestPath("../../hal/gpio")
    mctpSmbusTaskHeaderFile.setOutputName("gpio_api.h")
    mctpSmbusTaskHeaderFile.setProjectPath("hal/gpio/")
    mctpSmbusTaskHeaderFile.setOverwrite(True)
    mctpSmbusTaskHeaderFile.setType("HEADER")
    mctpSmbusTaskHeaderFile.setMarkup(False)

    #Add hal/interrupt/interrupt_api.c
    mctpSmbusTaskSourceFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskSourceFile.setSourcePath("/libraries/cec173x_sg3/src/hal/interrupt/interrupt_api.c")
    mctpSmbusTaskSourceFile.setDestPath("../../hal/interrupt")
    mctpSmbusTaskSourceFile.setOutputName("interrupt_api.c")
    mctpSmbusTaskSourceFile.setProjectPath("hal/interrupt/")
    mctpSmbusTaskSourceFile.setOverwrite(True)
    mctpSmbusTaskSourceFile.setType("SOURCE")
    mctpSmbusTaskSourceFile.setMarkup(False)

    #Add hal/interrupt/interrupt_api.h
    mctpSmbusTaskHeaderFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskHeaderFile.setSourcePath("/libraries/cec173x_sg3/src/hal/interrupt/interrupt_api.h")
    mctpSmbusTaskHeaderFile.setDestPath("../../hal/interrupt")
    mctpSmbusTaskHeaderFile.setOutputName("interrupt_api.h")
    mctpSmbusTaskHeaderFile.setProjectPath("hal/interrupt/")
    mctpSmbusTaskHeaderFile.setOverwrite(True)
    mctpSmbusTaskHeaderFile.setType("HEADER")
    mctpSmbusTaskHeaderFile.setMarkup(False)

    #Add hal/qmspi/qmspi_api.c
    mctpSmbusTaskSourceFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskSourceFile.setSourcePath("/libraries/cec173x_sg3/src/hal/qmspi/qmspi_api.c")
    mctpSmbusTaskSourceFile.setDestPath("../../hal/qmspi")
    mctpSmbusTaskSourceFile.setOutputName("qmspi_api.c")
    mctpSmbusTaskSourceFile.setProjectPath("hal/qmspi/")
    mctpSmbusTaskSourceFile.setOverwrite(True)
    mctpSmbusTaskSourceFile.setType("SOURCE")
    mctpSmbusTaskSourceFile.setMarkup(False)

    #Add hal/qmspi/qmspi_api.h
    mctpSmbusTaskHeaderFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskHeaderFile.setSourcePath("/libraries/cec173x_sg3/src/hal/qmspi/qmspi_api.h")
    mctpSmbusTaskHeaderFile.setDestPath("../../hal/qmspi")
    mctpSmbusTaskHeaderFile.setOutputName("qmspi_api.h")
    mctpSmbusTaskHeaderFile.setProjectPath("hal/qmspi/")
    mctpSmbusTaskHeaderFile.setOverwrite(True)
    mctpSmbusTaskHeaderFile.setType("HEADER")
    mctpSmbusTaskHeaderFile.setMarkup(False)

    #Add hal/uart/uart_api.c
    mctpSmbusTaskSourceFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskSourceFile.setSourcePath("/libraries/cec173x_sg3/src/hal/uart/uart_api.c")
    mctpSmbusTaskSourceFile.setDestPath("../../hal/uart")
    mctpSmbusTaskSourceFile.setOutputName("uart_api.c")
    mctpSmbusTaskSourceFile.setProjectPath("hal/uart/")
    mctpSmbusTaskSourceFile.setOverwrite(True)
    mctpSmbusTaskSourceFile.setType("SOURCE")
    mctpSmbusTaskSourceFile.setMarkup(False)

    #Add hal/uart/uart_api.h
    mctpSmbusTaskHeaderFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskHeaderFile.setSourcePath("/libraries/cec173x_sg3/src/hal/uart/uart_api.h")
    mctpSmbusTaskHeaderFile.setDestPath("../../hal/uart")
    mctpSmbusTaskHeaderFile.setOutputName("uart_api.h")
    mctpSmbusTaskHeaderFile.setProjectPath("hal/uart/")
    mctpSmbusTaskHeaderFile.setOverwrite(True)
    mctpSmbusTaskHeaderFile.setType("HEADER")
    mctpSmbusTaskHeaderFile.setMarkup(False)

    #Add hal/vtr_mon/vtr_mon_api.c
    mctpSmbusTaskSourceFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskSourceFile.setSourcePath("/libraries/cec173x_sg3/src/hal/vtr_mon/vtr_mon_api.c")
    mctpSmbusTaskSourceFile.setDestPath("../../hal/vtr_mon")
    mctpSmbusTaskSourceFile.setOutputName("vtr_mon_api.c")
    mctpSmbusTaskSourceFile.setProjectPath("hal/vtr_mon/")
    mctpSmbusTaskSourceFile.setOverwrite(True)
    mctpSmbusTaskSourceFile.setType("SOURCE")
    mctpSmbusTaskSourceFile.setMarkup(False)

    #Add hal/vtr_mon/vtr_mon_api.h
    mctpSmbusTaskHeaderFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskHeaderFile.setSourcePath("/libraries/cec173x_sg3/src/hal/vtr_mon/vtr_mon_api.h")
    mctpSmbusTaskHeaderFile.setDestPath("../../hal/vtr_mon")
    mctpSmbusTaskHeaderFile.setOutputName("vtr_mon_api.h")
    mctpSmbusTaskHeaderFile.setProjectPath("hal/vtr_mon/")
    mctpSmbusTaskHeaderFile.setOverwrite(True)
    mctpSmbusTaskHeaderFile.setType("HEADER")
    mctpSmbusTaskHeaderFile.setMarkup(False)

    #Add kernel/app.h
    mctpSmbusTaskHeaderFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskHeaderFile.setSourcePath("/libraries/cec173x_sg3/src/kernel/app.h")
    mctpSmbusTaskHeaderFile.setDestPath("../../kernel")
    mctpSmbusTaskHeaderFile.setOutputName("app.h")
    mctpSmbusTaskHeaderFile.setProjectPath("kernel/")
    mctpSmbusTaskHeaderFile.setOverwrite(True)
    mctpSmbusTaskHeaderFile.setType("HEADER")
    mctpSmbusTaskHeaderFile.setMarkup(False)

    #Add oem/oem_task1/oem_task1.c
    mctpSmbusTaskSourceFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskSourceFile.setSourcePath("/libraries/cec173x_sg3/src/oem/oem_task1/oem_task1.c")
    mctpSmbusTaskSourceFile.setDestPath("../../oem/oem_task1")
    mctpSmbusTaskSourceFile.setOutputName("oem_task1.c")
    mctpSmbusTaskSourceFile.setProjectPath("oem/oem_task1/")
    mctpSmbusTaskSourceFile.setOverwrite(True)
    mctpSmbusTaskSourceFile.setType("SOURCE")
    mctpSmbusTaskSourceFile.setMarkup(False)

    #Add oem/oem_task1/oem_task1.h
    mctpSmbusTaskHeaderFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskHeaderFile.setSourcePath("/libraries/cec173x_sg3/src/oem/oem_task1/oem_task1.h")
    mctpSmbusTaskHeaderFile.setDestPath("../../oem/oem_task1")
    mctpSmbusTaskHeaderFile.setOutputName("oem_task1.h")
    mctpSmbusTaskHeaderFile.setProjectPath("oem/oem_task1/")
    mctpSmbusTaskHeaderFile.setOverwrite(True)
    mctpSmbusTaskHeaderFile.setType("HEADER")
    mctpSmbusTaskHeaderFile.setMarkup(False)

    #Add oem/oem_task2/oem_task2.c
    mctpSmbusTaskSourceFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskSourceFile.setSourcePath("/libraries/cec173x_sg3/src/oem/oem_task2/oem_task2.c")
    mctpSmbusTaskSourceFile.setDestPath("../../oem/oem_task2")
    mctpSmbusTaskSourceFile.setOutputName("oem_task2.c")
    mctpSmbusTaskSourceFile.setProjectPath("oem/oem_task2/")
    mctpSmbusTaskSourceFile.setOverwrite(True)
    mctpSmbusTaskSourceFile.setType("SOURCE")
    mctpSmbusTaskSourceFile.setMarkup(False)

    #Add oem/oem_task2/oem_task2.h
    mctpSmbusTaskHeaderFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskHeaderFile.setSourcePath("/libraries/cec173x_sg3/src/oem/oem_task2/oem_task2.h")
    mctpSmbusTaskHeaderFile.setDestPath("../../oem/oem_task2")
    mctpSmbusTaskHeaderFile.setOutputName("oem_task2.h")
    mctpSmbusTaskHeaderFile.setProjectPath("oem/oem_task2/")
    mctpSmbusTaskHeaderFile.setOverwrite(True)
    mctpSmbusTaskHeaderFile.setType("HEADER")
    mctpSmbusTaskHeaderFile.setMarkup(False)

    #Add oem/oem_task3/oem_task3.c
    mctpSmbusTaskSourceFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskSourceFile.setSourcePath("/libraries/cec173x_sg3/src/oem/oem_task3/oem_task3.c")
    mctpSmbusTaskSourceFile.setDestPath("../../oem/oem_task3")
    mctpSmbusTaskSourceFile.setOutputName("oem_task3.c")
    mctpSmbusTaskSourceFile.setProjectPath("oem/oem_task3/")
    mctpSmbusTaskSourceFile.setOverwrite(True)
    mctpSmbusTaskSourceFile.setType("SOURCE")
    mctpSmbusTaskSourceFile.setMarkup(False)

    #Add oem/oem_task3/oem_task3.h
    mctpSmbusTaskHeaderFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskHeaderFile.setSourcePath("/libraries/cec173x_sg3/src/oem/oem_task3/oem_task3.h")
    mctpSmbusTaskHeaderFile.setDestPath("../../oem/oem_task3")
    mctpSmbusTaskHeaderFile.setOutputName("oem_task3.h")
    mctpSmbusTaskHeaderFile.setProjectPath("oem/oem_task3/")
    mctpSmbusTaskHeaderFile.setOverwrite(True)
    mctpSmbusTaskHeaderFile.setType("HEADER")
    mctpSmbusTaskHeaderFile.setMarkup(False)

    #Add platform/irqhandler.c
    mctpSmbusTaskSourceFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskSourceFile.setSourcePath("/libraries/cec173x_sg3/src/platform/irqhandler.c")
    mctpSmbusTaskSourceFile.setDestPath("../../platform")
    mctpSmbusTaskSourceFile.setOutputName("irqhandler.c")
    mctpSmbusTaskSourceFile.setProjectPath("platform/")
    mctpSmbusTaskSourceFile.setOverwrite(True)
    mctpSmbusTaskSourceFile.setType("SOURCE")
    mctpSmbusTaskSourceFile.setMarkup(False)

    #Add platform/platform.h
    mctpSmbusTaskHeaderFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskHeaderFile.setSourcePath("/libraries/cec173x_sg3/src/platform/platform.h")
    mctpSmbusTaskHeaderFile.setDestPath("../../platform")
    mctpSmbusTaskHeaderFile.setOutputName("platform.h")
    mctpSmbusTaskHeaderFile.setProjectPath("platform/")
    mctpSmbusTaskHeaderFile.setOverwrite(True)
    mctpSmbusTaskHeaderFile.setType("HEADER")
    mctpSmbusTaskHeaderFile.setMarkup(False)

    #Add platform/platform_serial_flash.c
    mctpSmbusTaskSourceFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskSourceFile.setSourcePath("/libraries/cec173x_sg3/src/platform/platform_serial_flash.c")
    mctpSmbusTaskSourceFile.setDestPath("../../platform")
    mctpSmbusTaskSourceFile.setOutputName("platform_serial_flash.c")
    mctpSmbusTaskSourceFile.setProjectPath("platform/")
    mctpSmbusTaskSourceFile.setOverwrite(True)
    mctpSmbusTaskSourceFile.setType("SOURCE")
    mctpSmbusTaskSourceFile.setMarkup(False)

    #Add platform/platform_serial_flash.h
    mctpSmbusTaskHeaderFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskHeaderFile.setSourcePath("/libraries/cec173x_sg3/src/platform/platform_serial_flash.h")
    mctpSmbusTaskHeaderFile.setDestPath("../../platform")
    mctpSmbusTaskHeaderFile.setOutputName("platform_serial_flash.h")
    mctpSmbusTaskHeaderFile.setProjectPath("platform/")
    mctpSmbusTaskHeaderFile.setOverwrite(True)
    mctpSmbusTaskHeaderFile.setType("HEADER")
    mctpSmbusTaskHeaderFile.setMarkup(False)

    #Add startup/startup.h
    mctpSmbusTaskHeaderFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskHeaderFile.setSourcePath("/libraries/cec173x_sg3/src/startup/startup.h")
    mctpSmbusTaskHeaderFile.setDestPath("../../startup")
    mctpSmbusTaskHeaderFile.setOutputName("startup.h")
    mctpSmbusTaskHeaderFile.setProjectPath("startup/")
    mctpSmbusTaskHeaderFile.setOverwrite(True)
    mctpSmbusTaskHeaderFile.setType("HEADER")
    mctpSmbusTaskHeaderFile.setMarkup(False)

    #Add startup/startup_CEC173x.S
    mctpSmbusTaskSourceFile = sg3LibComponent.createFileSymbol(None, None)
    mctpSmbusTaskSourceFile.setSourcePath("/libraries/cec173x_sg3/src/startup/startup_CEC173x.S")
    mctpSmbusTaskSourceFile.setDestPath("../../startup")
    mctpSmbusTaskSourceFile.setOutputName("startup_CEC173x.S")
    mctpSmbusTaskSourceFile.setProjectPath("startup/")
    mctpSmbusTaskSourceFile.setOverwrite(True)
    mctpSmbusTaskSourceFile.setType("SOURCE")
    mctpSmbusTaskSourceFile.setMarkup(False)

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32_0", None)
    xc32Optimization.setCategory("C32")
    xc32Optimization.setKey("optimization-level")
    xc32Optimization.setValue("-Os")

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32_1", None)
    xc32Optimization.setCategory("C32")
    xc32Optimization.setKey("isolate-each-function")
    xc32Optimization.setValue("false")

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32_2", None)
    xc32Optimization.setCategory("C32")
    xc32Optimization.setKey("make-warnings-into-errors")
    xc32Optimization.setValue("false")

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32_3", None)
    xc32Optimization.setCategory("C32")
    xc32Optimization.setKey("additional-warnings")
    xc32Optimization.setValue("false")

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32_4", None)
    xc32Optimization.setCategory("C32")
    xc32Optimization.setKey("place-data-into-section")
    xc32Optimization.setValue("false")

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32_5", None)
    xc32Optimization.setCategory("C32-LD")
    xc32Optimization.setKey("extra-lib-directories")
    xc32Optimization.setValue("../src/config/" + configName + "/library/secureboot_app_lib")
    xc32Optimization.setAppend(True, ";")

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32_6", None)
    xc32Optimization.setCategory("C32-LD")
    xc32Optimization.setKey("extra-lib-directories")
    xc32Optimization.setValue("../src/config/" + configName + "/library/secureboot_LIB")
    xc32Optimization.setAppend(True, ";")

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32_7", None)
    xc32Optimization.setCategory("C32-LD")
    xc32Optimization.setKey("appendMe")
    xc32Optimization.setValue("-u irqhandler_tag")

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32_8", None)
    xc32Optimization.setCategory("C32")
    xc32Optimization.setAppend(True, ";")
    xc32Optimization.setKey("extra-include-directories")
    xc32Optimization.setValue("../src/common/include")
    xc32Optimization.setAppend(True, ";")

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32_9", None)
    xc32Optimization.setCategory("C32")
    xc32Optimization.setAppend(True, ";")
    xc32Optimization.setKey("extra-include-directories")
    xc32Optimization.setValue("../src/common/debug")
    xc32Optimization.setAppend(True, ";")

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32_10", None)
    xc32Optimization.setCategory("C32")
    xc32Optimization.setAppend(True, ";")
    xc32Optimization.setKey("extra-include-directories")
    xc32Optimization.setValue("../src/hal")
    xc32Optimization.setAppend(True, ";")

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32_11", None)
    xc32Optimization.setCategory("C32")
    xc32Optimization.setAppend(True, ";")
    xc32Optimization.setKey("extra-include-directories")
    xc32Optimization.setValue("../src/kernel")
    xc32Optimization.setAppend(True, ";")

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32_12", None)
    xc32Optimization.setCategory("C32")
    xc32Optimization.setAppend(True, ";")
    xc32Optimization.setKey("extra-include-directories")
    xc32Optimization.setValue("../src/oem/oem_task1")
    xc32Optimization.setAppend(True, ";")

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32_13", None)
    xc32Optimization.setCategory("C32")
    xc32Optimization.setAppend(True, ";")
    xc32Optimization.setKey("extra-include-directories")
    xc32Optimization.setValue("../src/oem/oem_task2")
    xc32Optimization.setAppend(True, ";")

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32_14", None)
    xc32Optimization.setCategory("C32")
    xc32Optimization.setAppend(True, ";")
    xc32Optimization.setKey("extra-include-directories")
    xc32Optimization.setValue("../src/oem/oem_task3")
    xc32Optimization.setAppend(True, ";")

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32_15", None)
    xc32Optimization.setCategory("C32")
    xc32Optimization.setAppend(True, ";")
    xc32Optimization.setKey("extra-include-directories")
    xc32Optimization.setValue("../src/platform")
    xc32Optimization.setAppend(True, ";")

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32_16", None)
    xc32Optimization.setCategory("C32")
    xc32Optimization.setAppend(True, ";")
    xc32Optimization.setKey("extra-include-directories")
    xc32Optimization.setValue("../src/startup")
    xc32Optimization.setAppend(True, ";")

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32CPP_0", None)
    xc32Optimization.setCategory("C32CPP")
    xc32Optimization.setKey("optimization-level")
    xc32Optimization.setValue("-Os")

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32CPP_1", None)
    xc32Optimization.setCategory("C32CPP")
    xc32Optimization.setKey("isolate-each-function")
    xc32Optimization.setValue("false")

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32CPP_2", None)
    xc32Optimization.setCategory("C32CPP")
    xc32Optimization.setKey("make-warnings-into-errors")
    xc32Optimization.setValue("false")

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32CPP_3", None)
    xc32Optimization.setCategory("C32CPP")
    xc32Optimization.setKey("additional-warnings")
    xc32Optimization.setValue("false")

    xc32Optimization = sg3LibComponent.createSettingSymbol("XC32CPP_4", None)
    xc32Optimization.setCategory("C32CPP")
    xc32Optimization.setKey("place-data-into-section")
    xc32Optimization.setValue("false")
