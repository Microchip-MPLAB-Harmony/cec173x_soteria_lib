#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/sg3_h3_port.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/sg3_h3_port.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../src/common/debug/serial.c ../src/config/default/peripheral/dma/plib_dma.c ../src/config/default/peripheral/ecia/plib_ecia.c ../src/config/default/peripheral/gpio/plib_gpio.c ../src/config/default/peripheral/i2c/master/plib_smb1_master.c ../src/config/default/peripheral/i2c/master/plib_smb0_master.c ../src/config/default/peripheral/i2c/master/plib_smb2_master.c ../src/config/default/peripheral/i2c/master/plib_smb4_master.c ../src/config/default/peripheral/i2c/master/plib_smb3_master.c ../src/config/default/peripheral/i2c/slave/plib_smb0_slave.c ../src/config/default/peripheral/i2c/slave/plib_smb1_slave.c ../src/config/default/peripheral/i2c/slave/plib_smb2_slave.c ../src/config/default/peripheral/i2c/slave/plib_smb3_slave.c ../src/config/default/peripheral/i2c/slave/plib_smb4_slave.c ../src/config/default/peripheral/i2c/plib_smb0_master_slave_common.c ../src/config/default/peripheral/i2c/plib_smb1_master_slave_common.c ../src/config/default/peripheral/i2c/plib_i2c_smb_common.c ../src/config/default/peripheral/i2c/plib_smb4_master_slave_common.c ../src/config/default/peripheral/i2c/plib_smb2_master_slave_common.c ../src/config/default/peripheral/i2c/plib_smb3_master_slave_common.c ../src/config/default/peripheral/i2c/plib_i2c_smbus_lowlevel_intf.c ../src/config/default/peripheral/nvic/plib_nvic.c ../src/config/default/peripheral/pcr/plib_pcr.c ../src/config/default/peripheral/qmspi/plib_qmspi1.c ../src/config/default/peripheral/qmspi/plib_qmspi0.c ../src/config/default/peripheral/uart/plib_uart0.c ../src/config/default/exceptions.c ../src/config/default/interrupts.c ../src/config/default/initialization.c ../src/hal/gpio/gpio_api.c ../src/hal/interrupt/interrupt_api.c ../src/hal/qmspi/qmspi_api.c ../src/hal/uart/uart_api.c ../src/kernel/main.c ../src/oem/oem_task1/oem_task1.c ../src/oem/oem_task2/oem_task2.c ../src/oem/oem_task3/oem_task3.c ../src/platform/irqhandler.c ../src/platform/platform_serial_flash.c ../src/startup/startup_CEC173x.S

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1506558953/serial.o ${OBJECTDIR}/_ext/60166502/plib_dma.o ${OBJECTDIR}/_ext/1865182088/plib_ecia.o ${OBJECTDIR}/_ext/1865254177/plib_gpio.o ${OBJECTDIR}/_ext/513455433/plib_smb1_master.o ${OBJECTDIR}/_ext/513455433/plib_smb0_master.o ${OBJECTDIR}/_ext/513455433/plib_smb2_master.o ${OBJECTDIR}/_ext/513455433/plib_smb4_master.o ${OBJECTDIR}/_ext/513455433/plib_smb3_master.o ${OBJECTDIR}/_ext/1917247984/plib_smb0_slave.o ${OBJECTDIR}/_ext/1917247984/plib_smb1_slave.o ${OBJECTDIR}/_ext/1917247984/plib_smb2_slave.o ${OBJECTDIR}/_ext/1917247984/plib_smb3_slave.o ${OBJECTDIR}/_ext/1917247984/plib_smb4_slave.o ${OBJECTDIR}/_ext/60169480/plib_smb0_master_slave_common.o ${OBJECTDIR}/_ext/60169480/plib_smb1_master_slave_common.o ${OBJECTDIR}/_ext/60169480/plib_i2c_smb_common.o ${OBJECTDIR}/_ext/60169480/plib_smb4_master_slave_common.o ${OBJECTDIR}/_ext/60169480/plib_smb2_master_slave_common.o ${OBJECTDIR}/_ext/60169480/plib_smb3_master_slave_common.o ${OBJECTDIR}/_ext/60169480/plib_i2c_smbus_lowlevel_intf.o ${OBJECTDIR}/_ext/1865468468/plib_nvic.o ${OBJECTDIR}/_ext/60177741/plib_pcr.o ${OBJECTDIR}/_ext/1997460222/plib_qmspi1.o ${OBJECTDIR}/_ext/1997460222/plib_qmspi0.o ${OBJECTDIR}/_ext/1865657120/plib_uart0.o ${OBJECTDIR}/_ext/1171490990/exceptions.o ${OBJECTDIR}/_ext/1171490990/interrupts.o ${OBJECTDIR}/_ext/1171490990/initialization.o ${OBJECTDIR}/_ext/222663291/gpio_api.o ${OBJECTDIR}/_ext/684516973/interrupt_api.o ${OBJECTDIR}/_ext/1696528154/qmspi_api.o ${OBJECTDIR}/_ext/222260348/uart_api.o ${OBJECTDIR}/_ext/174097801/main.o ${OBJECTDIR}/_ext/1203168206/oem_task1.o ${OBJECTDIR}/_ext/1203168205/oem_task2.o ${OBJECTDIR}/_ext/1203168204/oem_task3.o ${OBJECTDIR}/_ext/1756295213/irqhandler.o ${OBJECTDIR}/_ext/1756295213/platform_serial_flash.o ${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1506558953/serial.o.d ${OBJECTDIR}/_ext/60166502/plib_dma.o.d ${OBJECTDIR}/_ext/1865182088/plib_ecia.o.d ${OBJECTDIR}/_ext/1865254177/plib_gpio.o.d ${OBJECTDIR}/_ext/513455433/plib_smb1_master.o.d ${OBJECTDIR}/_ext/513455433/plib_smb0_master.o.d ${OBJECTDIR}/_ext/513455433/plib_smb2_master.o.d ${OBJECTDIR}/_ext/513455433/plib_smb4_master.o.d ${OBJECTDIR}/_ext/513455433/plib_smb3_master.o.d ${OBJECTDIR}/_ext/1917247984/plib_smb0_slave.o.d ${OBJECTDIR}/_ext/1917247984/plib_smb1_slave.o.d ${OBJECTDIR}/_ext/1917247984/plib_smb2_slave.o.d ${OBJECTDIR}/_ext/1917247984/plib_smb3_slave.o.d ${OBJECTDIR}/_ext/1917247984/plib_smb4_slave.o.d ${OBJECTDIR}/_ext/60169480/plib_smb0_master_slave_common.o.d ${OBJECTDIR}/_ext/60169480/plib_smb1_master_slave_common.o.d ${OBJECTDIR}/_ext/60169480/plib_i2c_smb_common.o.d ${OBJECTDIR}/_ext/60169480/plib_smb4_master_slave_common.o.d ${OBJECTDIR}/_ext/60169480/plib_smb2_master_slave_common.o.d ${OBJECTDIR}/_ext/60169480/plib_smb3_master_slave_common.o.d ${OBJECTDIR}/_ext/60169480/plib_i2c_smbus_lowlevel_intf.o.d ${OBJECTDIR}/_ext/1865468468/plib_nvic.o.d ${OBJECTDIR}/_ext/60177741/plib_pcr.o.d ${OBJECTDIR}/_ext/1997460222/plib_qmspi1.o.d ${OBJECTDIR}/_ext/1997460222/plib_qmspi0.o.d ${OBJECTDIR}/_ext/1865657120/plib_uart0.o.d ${OBJECTDIR}/_ext/1171490990/exceptions.o.d ${OBJECTDIR}/_ext/1171490990/interrupts.o.d ${OBJECTDIR}/_ext/1171490990/initialization.o.d ${OBJECTDIR}/_ext/222663291/gpio_api.o.d ${OBJECTDIR}/_ext/684516973/interrupt_api.o.d ${OBJECTDIR}/_ext/1696528154/qmspi_api.o.d ${OBJECTDIR}/_ext/222260348/uart_api.o.d ${OBJECTDIR}/_ext/174097801/main.o.d ${OBJECTDIR}/_ext/1203168206/oem_task1.o.d ${OBJECTDIR}/_ext/1203168205/oem_task2.o.d ${OBJECTDIR}/_ext/1203168204/oem_task3.o.d ${OBJECTDIR}/_ext/1756295213/irqhandler.o.d ${OBJECTDIR}/_ext/1756295213/platform_serial_flash.o.d ${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1506558953/serial.o ${OBJECTDIR}/_ext/60166502/plib_dma.o ${OBJECTDIR}/_ext/1865182088/plib_ecia.o ${OBJECTDIR}/_ext/1865254177/plib_gpio.o ${OBJECTDIR}/_ext/513455433/plib_smb1_master.o ${OBJECTDIR}/_ext/513455433/plib_smb0_master.o ${OBJECTDIR}/_ext/513455433/plib_smb2_master.o ${OBJECTDIR}/_ext/513455433/plib_smb4_master.o ${OBJECTDIR}/_ext/513455433/plib_smb3_master.o ${OBJECTDIR}/_ext/1917247984/plib_smb0_slave.o ${OBJECTDIR}/_ext/1917247984/plib_smb1_slave.o ${OBJECTDIR}/_ext/1917247984/plib_smb2_slave.o ${OBJECTDIR}/_ext/1917247984/plib_smb3_slave.o ${OBJECTDIR}/_ext/1917247984/plib_smb4_slave.o ${OBJECTDIR}/_ext/60169480/plib_smb0_master_slave_common.o ${OBJECTDIR}/_ext/60169480/plib_smb1_master_slave_common.o ${OBJECTDIR}/_ext/60169480/plib_i2c_smb_common.o ${OBJECTDIR}/_ext/60169480/plib_smb4_master_slave_common.o ${OBJECTDIR}/_ext/60169480/plib_smb2_master_slave_common.o ${OBJECTDIR}/_ext/60169480/plib_smb3_master_slave_common.o ${OBJECTDIR}/_ext/60169480/plib_i2c_smbus_lowlevel_intf.o ${OBJECTDIR}/_ext/1865468468/plib_nvic.o ${OBJECTDIR}/_ext/60177741/plib_pcr.o ${OBJECTDIR}/_ext/1997460222/plib_qmspi1.o ${OBJECTDIR}/_ext/1997460222/plib_qmspi0.o ${OBJECTDIR}/_ext/1865657120/plib_uart0.o ${OBJECTDIR}/_ext/1171490990/exceptions.o ${OBJECTDIR}/_ext/1171490990/interrupts.o ${OBJECTDIR}/_ext/1171490990/initialization.o ${OBJECTDIR}/_ext/222663291/gpio_api.o ${OBJECTDIR}/_ext/684516973/interrupt_api.o ${OBJECTDIR}/_ext/1696528154/qmspi_api.o ${OBJECTDIR}/_ext/222260348/uart_api.o ${OBJECTDIR}/_ext/174097801/main.o ${OBJECTDIR}/_ext/1203168206/oem_task1.o ${OBJECTDIR}/_ext/1203168205/oem_task2.o ${OBJECTDIR}/_ext/1203168204/oem_task3.o ${OBJECTDIR}/_ext/1756295213/irqhandler.o ${OBJECTDIR}/_ext/1756295213/platform_serial_flash.o ${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o

# Source Files
SOURCEFILES=../src/common/debug/serial.c ../src/config/default/peripheral/dma/plib_dma.c ../src/config/default/peripheral/ecia/plib_ecia.c ../src/config/default/peripheral/gpio/plib_gpio.c ../src/config/default/peripheral/i2c/master/plib_smb1_master.c ../src/config/default/peripheral/i2c/master/plib_smb0_master.c ../src/config/default/peripheral/i2c/master/plib_smb2_master.c ../src/config/default/peripheral/i2c/master/plib_smb4_master.c ../src/config/default/peripheral/i2c/master/plib_smb3_master.c ../src/config/default/peripheral/i2c/slave/plib_smb0_slave.c ../src/config/default/peripheral/i2c/slave/plib_smb1_slave.c ../src/config/default/peripheral/i2c/slave/plib_smb2_slave.c ../src/config/default/peripheral/i2c/slave/plib_smb3_slave.c ../src/config/default/peripheral/i2c/slave/plib_smb4_slave.c ../src/config/default/peripheral/i2c/plib_smb0_master_slave_common.c ../src/config/default/peripheral/i2c/plib_smb1_master_slave_common.c ../src/config/default/peripheral/i2c/plib_i2c_smb_common.c ../src/config/default/peripheral/i2c/plib_smb4_master_slave_common.c ../src/config/default/peripheral/i2c/plib_smb2_master_slave_common.c ../src/config/default/peripheral/i2c/plib_smb3_master_slave_common.c ../src/config/default/peripheral/i2c/plib_i2c_smbus_lowlevel_intf.c ../src/config/default/peripheral/nvic/plib_nvic.c ../src/config/default/peripheral/pcr/plib_pcr.c ../src/config/default/peripheral/qmspi/plib_qmspi1.c ../src/config/default/peripheral/qmspi/plib_qmspi0.c ../src/config/default/peripheral/uart/plib_uart0.c ../src/config/default/exceptions.c ../src/config/default/interrupts.c ../src/config/default/initialization.c ../src/hal/gpio/gpio_api.c ../src/hal/interrupt/interrupt_api.c ../src/hal/qmspi/qmspi_api.c ../src/hal/uart/uart_api.c ../src/kernel/main.c ../src/oem/oem_task1/oem_task1.c ../src/oem/oem_task2/oem_task2.c ../src/oem/oem_task3/oem_task3.c ../src/platform/irqhandler.c ../src/platform/platform_serial_flash.c ../src/startup/startup_CEC173x.S

# Pack Options 
PACK_COMMON_OPTIONS=-I "${CMSIS_DIR}/CMSIS/Core/Include"



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk ${DISTDIR}/sg3_h3_port.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=CEC1736_S0_2ZW
MP_LINKER_FILE_OPTION=,--script="..\src\common\include\secureboot_app.ld"
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o: ../src/startup/startup_CEC173x.S  .generated_files/flags/default/6bceca9226aa04f13fc205ae5550e06dd170d036 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/2116868995" 
	@${RM} ${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o.d 
	@${RM} ${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o 
	@${RM} ${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o.ok ${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG  -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o.d"  -o ${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o ../src/startup/startup_CEC173x.S  -DXPRJ_default=$(CND_CONF)    -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1 -mdfp="${DFP_DIR}/cec1736"
	@${FIXDEPS} "${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o.d" "${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o: ../src/startup/startup_CEC173x.S  .generated_files/flags/default/d2a41fb9483e9725dd6896530ca8339ed8f43b0b .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/2116868995" 
	@${RM} ${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o.d 
	@${RM} ${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o 
	@${RM} ${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o.ok ${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o.d"  -o ${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o ../src/startup/startup_CEC173x.S  -DXPRJ_default=$(CND_CONF)    -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o.asm.d",--gdwarf-2 -mdfp="${DFP_DIR}/cec1736"
	@${FIXDEPS} "${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o.d" "${OBJECTDIR}/_ext/2116868995/startup_CEC173x.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1506558953/serial.o: ../src/common/debug/serial.c  .generated_files/flags/default/39ff1cd9a311b7e88e2189292dc11e96aafd385a .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1506558953" 
	@${RM} ${OBJECTDIR}/_ext/1506558953/serial.o.d 
	@${RM} ${OBJECTDIR}/_ext/1506558953/serial.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1506558953/serial.o.d" -o ${OBJECTDIR}/_ext/1506558953/serial.o ../src/common/debug/serial.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60166502/plib_dma.o: ../src/config/default/peripheral/dma/plib_dma.c  .generated_files/flags/default/fe4aef6584946cc537c31063b11c8c81b39b1aa9 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/60166502" 
	@${RM} ${OBJECTDIR}/_ext/60166502/plib_dma.o.d 
	@${RM} ${OBJECTDIR}/_ext/60166502/plib_dma.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/60166502/plib_dma.o.d" -o ${OBJECTDIR}/_ext/60166502/plib_dma.o ../src/config/default/peripheral/dma/plib_dma.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1865182088/plib_ecia.o: ../src/config/default/peripheral/ecia/plib_ecia.c  .generated_files/flags/default/41844d5605e0b003dc633fc434e38820b4c626c1 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1865182088" 
	@${RM} ${OBJECTDIR}/_ext/1865182088/plib_ecia.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865182088/plib_ecia.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1865182088/plib_ecia.o.d" -o ${OBJECTDIR}/_ext/1865182088/plib_ecia.o ../src/config/default/peripheral/ecia/plib_ecia.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1865254177/plib_gpio.o: ../src/config/default/peripheral/gpio/plib_gpio.c  .generated_files/flags/default/2fd28929523ab8e5510a555d047d55eb4e3e8f67 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1865254177" 
	@${RM} ${OBJECTDIR}/_ext/1865254177/plib_gpio.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865254177/plib_gpio.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1865254177/plib_gpio.o.d" -o ${OBJECTDIR}/_ext/1865254177/plib_gpio.o ../src/config/default/peripheral/gpio/plib_gpio.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/513455433/plib_smb1_master.o: ../src/config/default/peripheral/i2c/master/plib_smb1_master.c  .generated_files/flags/default/6b74988f0a07695270f5bfc4b1f7e9892469d3d5 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/513455433" 
	@${RM} ${OBJECTDIR}/_ext/513455433/plib_smb1_master.o.d 
	@${RM} ${OBJECTDIR}/_ext/513455433/plib_smb1_master.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/513455433/plib_smb1_master.o.d" -o ${OBJECTDIR}/_ext/513455433/plib_smb1_master.o ../src/config/default/peripheral/i2c/master/plib_smb1_master.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/513455433/plib_smb0_master.o: ../src/config/default/peripheral/i2c/master/plib_smb0_master.c  .generated_files/flags/default/b7f86ebde47c39a5de65642f982820228e924c1b .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/513455433" 
	@${RM} ${OBJECTDIR}/_ext/513455433/plib_smb0_master.o.d 
	@${RM} ${OBJECTDIR}/_ext/513455433/plib_smb0_master.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/513455433/plib_smb0_master.o.d" -o ${OBJECTDIR}/_ext/513455433/plib_smb0_master.o ../src/config/default/peripheral/i2c/master/plib_smb0_master.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/513455433/plib_smb2_master.o: ../src/config/default/peripheral/i2c/master/plib_smb2_master.c  .generated_files/flags/default/95c57dd35eb2e6c96fd2251a4cf332c2c8b03b11 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/513455433" 
	@${RM} ${OBJECTDIR}/_ext/513455433/plib_smb2_master.o.d 
	@${RM} ${OBJECTDIR}/_ext/513455433/plib_smb2_master.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/513455433/plib_smb2_master.o.d" -o ${OBJECTDIR}/_ext/513455433/plib_smb2_master.o ../src/config/default/peripheral/i2c/master/plib_smb2_master.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/513455433/plib_smb4_master.o: ../src/config/default/peripheral/i2c/master/plib_smb4_master.c  .generated_files/flags/default/e01b8c155c6c78ce60d4dd996eb6a2a3c74cf7df .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/513455433" 
	@${RM} ${OBJECTDIR}/_ext/513455433/plib_smb4_master.o.d 
	@${RM} ${OBJECTDIR}/_ext/513455433/plib_smb4_master.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/513455433/plib_smb4_master.o.d" -o ${OBJECTDIR}/_ext/513455433/plib_smb4_master.o ../src/config/default/peripheral/i2c/master/plib_smb4_master.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/513455433/plib_smb3_master.o: ../src/config/default/peripheral/i2c/master/plib_smb3_master.c  .generated_files/flags/default/27945622449ab61b0834eee3513fab18c643fcf0 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/513455433" 
	@${RM} ${OBJECTDIR}/_ext/513455433/plib_smb3_master.o.d 
	@${RM} ${OBJECTDIR}/_ext/513455433/plib_smb3_master.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/513455433/plib_smb3_master.o.d" -o ${OBJECTDIR}/_ext/513455433/plib_smb3_master.o ../src/config/default/peripheral/i2c/master/plib_smb3_master.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1917247984/plib_smb0_slave.o: ../src/config/default/peripheral/i2c/slave/plib_smb0_slave.c  .generated_files/flags/default/cf9306e4cea7b851d80770a19527da1d9bbcddc .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1917247984" 
	@${RM} ${OBJECTDIR}/_ext/1917247984/plib_smb0_slave.o.d 
	@${RM} ${OBJECTDIR}/_ext/1917247984/plib_smb0_slave.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1917247984/plib_smb0_slave.o.d" -o ${OBJECTDIR}/_ext/1917247984/plib_smb0_slave.o ../src/config/default/peripheral/i2c/slave/plib_smb0_slave.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1917247984/plib_smb1_slave.o: ../src/config/default/peripheral/i2c/slave/plib_smb1_slave.c  .generated_files/flags/default/a66317618e2c5fd9669a9b9b21dddf179b977a37 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1917247984" 
	@${RM} ${OBJECTDIR}/_ext/1917247984/plib_smb1_slave.o.d 
	@${RM} ${OBJECTDIR}/_ext/1917247984/plib_smb1_slave.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1917247984/plib_smb1_slave.o.d" -o ${OBJECTDIR}/_ext/1917247984/plib_smb1_slave.o ../src/config/default/peripheral/i2c/slave/plib_smb1_slave.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1917247984/plib_smb2_slave.o: ../src/config/default/peripheral/i2c/slave/plib_smb2_slave.c  .generated_files/flags/default/304da1f23a1dbb1f2a43d12b38c421a4de1e0215 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1917247984" 
	@${RM} ${OBJECTDIR}/_ext/1917247984/plib_smb2_slave.o.d 
	@${RM} ${OBJECTDIR}/_ext/1917247984/plib_smb2_slave.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1917247984/plib_smb2_slave.o.d" -o ${OBJECTDIR}/_ext/1917247984/plib_smb2_slave.o ../src/config/default/peripheral/i2c/slave/plib_smb2_slave.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1917247984/plib_smb3_slave.o: ../src/config/default/peripheral/i2c/slave/plib_smb3_slave.c  .generated_files/flags/default/420c01926c1475ab8fc5a9177af75b4691bac63d .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1917247984" 
	@${RM} ${OBJECTDIR}/_ext/1917247984/plib_smb3_slave.o.d 
	@${RM} ${OBJECTDIR}/_ext/1917247984/plib_smb3_slave.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1917247984/plib_smb3_slave.o.d" -o ${OBJECTDIR}/_ext/1917247984/plib_smb3_slave.o ../src/config/default/peripheral/i2c/slave/plib_smb3_slave.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1917247984/plib_smb4_slave.o: ../src/config/default/peripheral/i2c/slave/plib_smb4_slave.c  .generated_files/flags/default/ca7db5c10b82838831d25ffdf065c83cdbf27539 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1917247984" 
	@${RM} ${OBJECTDIR}/_ext/1917247984/plib_smb4_slave.o.d 
	@${RM} ${OBJECTDIR}/_ext/1917247984/plib_smb4_slave.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1917247984/plib_smb4_slave.o.d" -o ${OBJECTDIR}/_ext/1917247984/plib_smb4_slave.o ../src/config/default/peripheral/i2c/slave/plib_smb4_slave.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60169480/plib_smb0_master_slave_common.o: ../src/config/default/peripheral/i2c/plib_smb0_master_slave_common.c  .generated_files/flags/default/2268d14658c20b3ee0ba7fcff8579621e6ce557 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/60169480" 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_smb0_master_slave_common.o.d 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_smb0_master_slave_common.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/60169480/plib_smb0_master_slave_common.o.d" -o ${OBJECTDIR}/_ext/60169480/plib_smb0_master_slave_common.o ../src/config/default/peripheral/i2c/plib_smb0_master_slave_common.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60169480/plib_smb1_master_slave_common.o: ../src/config/default/peripheral/i2c/plib_smb1_master_slave_common.c  .generated_files/flags/default/ad290e3828fb93917100b7debf1e39b0e955a4a1 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/60169480" 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_smb1_master_slave_common.o.d 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_smb1_master_slave_common.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/60169480/plib_smb1_master_slave_common.o.d" -o ${OBJECTDIR}/_ext/60169480/plib_smb1_master_slave_common.o ../src/config/default/peripheral/i2c/plib_smb1_master_slave_common.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60169480/plib_i2c_smb_common.o: ../src/config/default/peripheral/i2c/plib_i2c_smb_common.c  .generated_files/flags/default/47eee31e472b90fb47e961980ea82baf4b54b35a .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/60169480" 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_i2c_smb_common.o.d 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_i2c_smb_common.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/60169480/plib_i2c_smb_common.o.d" -o ${OBJECTDIR}/_ext/60169480/plib_i2c_smb_common.o ../src/config/default/peripheral/i2c/plib_i2c_smb_common.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60169480/plib_smb4_master_slave_common.o: ../src/config/default/peripheral/i2c/plib_smb4_master_slave_common.c  .generated_files/flags/default/72f702a0629a50488075fcbfea46abf9212ecf6d .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/60169480" 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_smb4_master_slave_common.o.d 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_smb4_master_slave_common.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/60169480/plib_smb4_master_slave_common.o.d" -o ${OBJECTDIR}/_ext/60169480/plib_smb4_master_slave_common.o ../src/config/default/peripheral/i2c/plib_smb4_master_slave_common.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60169480/plib_smb2_master_slave_common.o: ../src/config/default/peripheral/i2c/plib_smb2_master_slave_common.c  .generated_files/flags/default/68b8660894ced274d695a1d23740760f0569af67 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/60169480" 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_smb2_master_slave_common.o.d 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_smb2_master_slave_common.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/60169480/plib_smb2_master_slave_common.o.d" -o ${OBJECTDIR}/_ext/60169480/plib_smb2_master_slave_common.o ../src/config/default/peripheral/i2c/plib_smb2_master_slave_common.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60169480/plib_smb3_master_slave_common.o: ../src/config/default/peripheral/i2c/plib_smb3_master_slave_common.c  .generated_files/flags/default/ec1ed2e3203f9284d218b976cd7ac394d6c06122 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/60169480" 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_smb3_master_slave_common.o.d 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_smb3_master_slave_common.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/60169480/plib_smb3_master_slave_common.o.d" -o ${OBJECTDIR}/_ext/60169480/plib_smb3_master_slave_common.o ../src/config/default/peripheral/i2c/plib_smb3_master_slave_common.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60169480/plib_i2c_smbus_lowlevel_intf.o: ../src/config/default/peripheral/i2c/plib_i2c_smbus_lowlevel_intf.c  .generated_files/flags/default/7e3700a4fe51368ccb1721dc85d8c9740e3a0a29 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/60169480" 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_i2c_smbus_lowlevel_intf.o.d 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_i2c_smbus_lowlevel_intf.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/60169480/plib_i2c_smbus_lowlevel_intf.o.d" -o ${OBJECTDIR}/_ext/60169480/plib_i2c_smbus_lowlevel_intf.o ../src/config/default/peripheral/i2c/plib_i2c_smbus_lowlevel_intf.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1865468468/plib_nvic.o: ../src/config/default/peripheral/nvic/plib_nvic.c  .generated_files/flags/default/f79ce7256de58ebf0c91e1e05046d7c304addb1d .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1865468468" 
	@${RM} ${OBJECTDIR}/_ext/1865468468/plib_nvic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865468468/plib_nvic.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1865468468/plib_nvic.o.d" -o ${OBJECTDIR}/_ext/1865468468/plib_nvic.o ../src/config/default/peripheral/nvic/plib_nvic.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60177741/plib_pcr.o: ../src/config/default/peripheral/pcr/plib_pcr.c  .generated_files/flags/default/c82e8a800896fb0e727cfbef6bbdad27cd69f4e3 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/60177741" 
	@${RM} ${OBJECTDIR}/_ext/60177741/plib_pcr.o.d 
	@${RM} ${OBJECTDIR}/_ext/60177741/plib_pcr.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/60177741/plib_pcr.o.d" -o ${OBJECTDIR}/_ext/60177741/plib_pcr.o ../src/config/default/peripheral/pcr/plib_pcr.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1997460222/plib_qmspi1.o: ../src/config/default/peripheral/qmspi/plib_qmspi1.c  .generated_files/flags/default/6d50c5312a6f3bd9ffb456855235d5e4deffafa0 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1997460222" 
	@${RM} ${OBJECTDIR}/_ext/1997460222/plib_qmspi1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1997460222/plib_qmspi1.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1997460222/plib_qmspi1.o.d" -o ${OBJECTDIR}/_ext/1997460222/plib_qmspi1.o ../src/config/default/peripheral/qmspi/plib_qmspi1.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1997460222/plib_qmspi0.o: ../src/config/default/peripheral/qmspi/plib_qmspi0.c  .generated_files/flags/default/76d0738867ba2c31f6ab750cc6f2cd5cb1248e81 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1997460222" 
	@${RM} ${OBJECTDIR}/_ext/1997460222/plib_qmspi0.o.d 
	@${RM} ${OBJECTDIR}/_ext/1997460222/plib_qmspi0.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1997460222/plib_qmspi0.o.d" -o ${OBJECTDIR}/_ext/1997460222/plib_qmspi0.o ../src/config/default/peripheral/qmspi/plib_qmspi0.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1865657120/plib_uart0.o: ../src/config/default/peripheral/uart/plib_uart0.c  .generated_files/flags/default/289c41ea1922b28183528b51af1cf32aeb204d2f .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1865657120" 
	@${RM} ${OBJECTDIR}/_ext/1865657120/plib_uart0.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865657120/plib_uart0.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1865657120/plib_uart0.o.d" -o ${OBJECTDIR}/_ext/1865657120/plib_uart0.o ../src/config/default/peripheral/uart/plib_uart0.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1171490990/exceptions.o: ../src/config/default/exceptions.c  .generated_files/flags/default/dbbfc79a6c0ed468112bd9037f9761f9ceae4510 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/exceptions.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1171490990/exceptions.o.d" -o ${OBJECTDIR}/_ext/1171490990/exceptions.o ../src/config/default/exceptions.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1171490990/interrupts.o: ../src/config/default/interrupts.c  .generated_files/flags/default/a108c1fd6ab1f4ecd5339debd9ecee2bba28b162 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/interrupts.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/interrupts.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1171490990/interrupts.o.d" -o ${OBJECTDIR}/_ext/1171490990/interrupts.o ../src/config/default/interrupts.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1171490990/initialization.o: ../src/config/default/initialization.c  .generated_files/flags/default/168e84ecc5225ec0c417edc5945eb6c0fb56f67c .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/initialization.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/initialization.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1171490990/initialization.o.d" -o ${OBJECTDIR}/_ext/1171490990/initialization.o ../src/config/default/initialization.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/222663291/gpio_api.o: ../src/hal/gpio/gpio_api.c  .generated_files/flags/default/7a6d874c1a453a5407363b8de29ced09ab72cd0e .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/222663291" 
	@${RM} ${OBJECTDIR}/_ext/222663291/gpio_api.o.d 
	@${RM} ${OBJECTDIR}/_ext/222663291/gpio_api.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/222663291/gpio_api.o.d" -o ${OBJECTDIR}/_ext/222663291/gpio_api.o ../src/hal/gpio/gpio_api.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/684516973/interrupt_api.o: ../src/hal/interrupt/interrupt_api.c  .generated_files/flags/default/b05298e4a7df34013ad15557f37d0f0db18e5457 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/684516973" 
	@${RM} ${OBJECTDIR}/_ext/684516973/interrupt_api.o.d 
	@${RM} ${OBJECTDIR}/_ext/684516973/interrupt_api.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/684516973/interrupt_api.o.d" -o ${OBJECTDIR}/_ext/684516973/interrupt_api.o ../src/hal/interrupt/interrupt_api.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1696528154/qmspi_api.o: ../src/hal/qmspi/qmspi_api.c  .generated_files/flags/default/3ec35f71acafafb6d7206c779d457b538de56639 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1696528154" 
	@${RM} ${OBJECTDIR}/_ext/1696528154/qmspi_api.o.d 
	@${RM} ${OBJECTDIR}/_ext/1696528154/qmspi_api.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1696528154/qmspi_api.o.d" -o ${OBJECTDIR}/_ext/1696528154/qmspi_api.o ../src/hal/qmspi/qmspi_api.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/222260348/uart_api.o: ../src/hal/uart/uart_api.c  .generated_files/flags/default/c4e4fc5f3d97c7ece0401900c8bbf452e65e4c88 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/222260348" 
	@${RM} ${OBJECTDIR}/_ext/222260348/uart_api.o.d 
	@${RM} ${OBJECTDIR}/_ext/222260348/uart_api.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/222260348/uart_api.o.d" -o ${OBJECTDIR}/_ext/222260348/uart_api.o ../src/hal/uart/uart_api.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/174097801/main.o: ../src/kernel/main.c  .generated_files/flags/default/4b276e0eef7beb78ecfea73cdc42b5a5e6fb30cf .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/174097801" 
	@${RM} ${OBJECTDIR}/_ext/174097801/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/174097801/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/174097801/main.o.d" -o ${OBJECTDIR}/_ext/174097801/main.o ../src/kernel/main.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1203168206/oem_task1.o: ../src/oem/oem_task1/oem_task1.c  .generated_files/flags/default/b17a5d747653c3b9e595a24c001328bc1d997981 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1203168206" 
	@${RM} ${OBJECTDIR}/_ext/1203168206/oem_task1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1203168206/oem_task1.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1203168206/oem_task1.o.d" -o ${OBJECTDIR}/_ext/1203168206/oem_task1.o ../src/oem/oem_task1/oem_task1.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1203168205/oem_task2.o: ../src/oem/oem_task2/oem_task2.c  .generated_files/flags/default/aa3a2746e42da2dabdea7122893168a9ef44542b .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1203168205" 
	@${RM} ${OBJECTDIR}/_ext/1203168205/oem_task2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1203168205/oem_task2.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1203168205/oem_task2.o.d" -o ${OBJECTDIR}/_ext/1203168205/oem_task2.o ../src/oem/oem_task2/oem_task2.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1203168204/oem_task3.o: ../src/oem/oem_task3/oem_task3.c  .generated_files/flags/default/f41a4def4668f30230f612d61f6168663abea470 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1203168204" 
	@${RM} ${OBJECTDIR}/_ext/1203168204/oem_task3.o.d 
	@${RM} ${OBJECTDIR}/_ext/1203168204/oem_task3.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1203168204/oem_task3.o.d" -o ${OBJECTDIR}/_ext/1203168204/oem_task3.o ../src/oem/oem_task3/oem_task3.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1756295213/irqhandler.o: ../src/platform/irqhandler.c  .generated_files/flags/default/741142ef36bb099cca45c8c4ca43cb9177bb2b9 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1756295213" 
	@${RM} ${OBJECTDIR}/_ext/1756295213/irqhandler.o.d 
	@${RM} ${OBJECTDIR}/_ext/1756295213/irqhandler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1756295213/irqhandler.o.d" -o ${OBJECTDIR}/_ext/1756295213/irqhandler.o ../src/platform/irqhandler.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1756295213/platform_serial_flash.o: ../src/platform/platform_serial_flash.c  .generated_files/flags/default/1566dbee78b4b7e8e39f5dbb930b1e0a68ca54ff .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1756295213" 
	@${RM} ${OBJECTDIR}/_ext/1756295213/platform_serial_flash.o.d 
	@${RM} ${OBJECTDIR}/_ext/1756295213/platform_serial_flash.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1756295213/platform_serial_flash.o.d" -o ${OBJECTDIR}/_ext/1756295213/platform_serial_flash.o ../src/platform/platform_serial_flash.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
else
${OBJECTDIR}/_ext/1506558953/serial.o: ../src/common/debug/serial.c  .generated_files/flags/default/43a5e2b6c5c36b00f73d973d3888d7f71edf7136 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1506558953" 
	@${RM} ${OBJECTDIR}/_ext/1506558953/serial.o.d 
	@${RM} ${OBJECTDIR}/_ext/1506558953/serial.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1506558953/serial.o.d" -o ${OBJECTDIR}/_ext/1506558953/serial.o ../src/common/debug/serial.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60166502/plib_dma.o: ../src/config/default/peripheral/dma/plib_dma.c  .generated_files/flags/default/70560b4f502922a99fcaa5123e36f4a054705aa5 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/60166502" 
	@${RM} ${OBJECTDIR}/_ext/60166502/plib_dma.o.d 
	@${RM} ${OBJECTDIR}/_ext/60166502/plib_dma.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/60166502/plib_dma.o.d" -o ${OBJECTDIR}/_ext/60166502/plib_dma.o ../src/config/default/peripheral/dma/plib_dma.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1865182088/plib_ecia.o: ../src/config/default/peripheral/ecia/plib_ecia.c  .generated_files/flags/default/ca047a43499e6736741a9e2e1b4a0f052fa42423 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1865182088" 
	@${RM} ${OBJECTDIR}/_ext/1865182088/plib_ecia.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865182088/plib_ecia.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1865182088/plib_ecia.o.d" -o ${OBJECTDIR}/_ext/1865182088/plib_ecia.o ../src/config/default/peripheral/ecia/plib_ecia.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1865254177/plib_gpio.o: ../src/config/default/peripheral/gpio/plib_gpio.c  .generated_files/flags/default/b203ebe6e1194ad620646ca087784797fd783c3e .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1865254177" 
	@${RM} ${OBJECTDIR}/_ext/1865254177/plib_gpio.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865254177/plib_gpio.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1865254177/plib_gpio.o.d" -o ${OBJECTDIR}/_ext/1865254177/plib_gpio.o ../src/config/default/peripheral/gpio/plib_gpio.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/513455433/plib_smb1_master.o: ../src/config/default/peripheral/i2c/master/plib_smb1_master.c  .generated_files/flags/default/ea296d0b7c73a1a6852d1f79e7ee3c7436618249 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/513455433" 
	@${RM} ${OBJECTDIR}/_ext/513455433/plib_smb1_master.o.d 
	@${RM} ${OBJECTDIR}/_ext/513455433/plib_smb1_master.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/513455433/plib_smb1_master.o.d" -o ${OBJECTDIR}/_ext/513455433/plib_smb1_master.o ../src/config/default/peripheral/i2c/master/plib_smb1_master.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/513455433/plib_smb0_master.o: ../src/config/default/peripheral/i2c/master/plib_smb0_master.c  .generated_files/flags/default/bccca29c7194457b1821b5c9bcea424e7de830aa .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/513455433" 
	@${RM} ${OBJECTDIR}/_ext/513455433/plib_smb0_master.o.d 
	@${RM} ${OBJECTDIR}/_ext/513455433/plib_smb0_master.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/513455433/plib_smb0_master.o.d" -o ${OBJECTDIR}/_ext/513455433/plib_smb0_master.o ../src/config/default/peripheral/i2c/master/plib_smb0_master.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/513455433/plib_smb2_master.o: ../src/config/default/peripheral/i2c/master/plib_smb2_master.c  .generated_files/flags/default/f7973dd9b30616018157ffcad33df10930142ab7 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/513455433" 
	@${RM} ${OBJECTDIR}/_ext/513455433/plib_smb2_master.o.d 
	@${RM} ${OBJECTDIR}/_ext/513455433/plib_smb2_master.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/513455433/plib_smb2_master.o.d" -o ${OBJECTDIR}/_ext/513455433/plib_smb2_master.o ../src/config/default/peripheral/i2c/master/plib_smb2_master.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/513455433/plib_smb4_master.o: ../src/config/default/peripheral/i2c/master/plib_smb4_master.c  .generated_files/flags/default/af1744a96d698f2c1420f076ccb014e134b569f5 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/513455433" 
	@${RM} ${OBJECTDIR}/_ext/513455433/plib_smb4_master.o.d 
	@${RM} ${OBJECTDIR}/_ext/513455433/plib_smb4_master.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/513455433/plib_smb4_master.o.d" -o ${OBJECTDIR}/_ext/513455433/plib_smb4_master.o ../src/config/default/peripheral/i2c/master/plib_smb4_master.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/513455433/plib_smb3_master.o: ../src/config/default/peripheral/i2c/master/plib_smb3_master.c  .generated_files/flags/default/1e9ada28fd7521bb20d2d63f131ed0a30d0d6485 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/513455433" 
	@${RM} ${OBJECTDIR}/_ext/513455433/plib_smb3_master.o.d 
	@${RM} ${OBJECTDIR}/_ext/513455433/plib_smb3_master.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/513455433/plib_smb3_master.o.d" -o ${OBJECTDIR}/_ext/513455433/plib_smb3_master.o ../src/config/default/peripheral/i2c/master/plib_smb3_master.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1917247984/plib_smb0_slave.o: ../src/config/default/peripheral/i2c/slave/plib_smb0_slave.c  .generated_files/flags/default/66244677ef4f04acc8203d9e46aa72aabfc1f757 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1917247984" 
	@${RM} ${OBJECTDIR}/_ext/1917247984/plib_smb0_slave.o.d 
	@${RM} ${OBJECTDIR}/_ext/1917247984/plib_smb0_slave.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1917247984/plib_smb0_slave.o.d" -o ${OBJECTDIR}/_ext/1917247984/plib_smb0_slave.o ../src/config/default/peripheral/i2c/slave/plib_smb0_slave.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1917247984/plib_smb1_slave.o: ../src/config/default/peripheral/i2c/slave/plib_smb1_slave.c  .generated_files/flags/default/35e71024ff0f7ee14ae53b0ee0f1c07c7adcbc97 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1917247984" 
	@${RM} ${OBJECTDIR}/_ext/1917247984/plib_smb1_slave.o.d 
	@${RM} ${OBJECTDIR}/_ext/1917247984/plib_smb1_slave.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1917247984/plib_smb1_slave.o.d" -o ${OBJECTDIR}/_ext/1917247984/plib_smb1_slave.o ../src/config/default/peripheral/i2c/slave/plib_smb1_slave.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1917247984/plib_smb2_slave.o: ../src/config/default/peripheral/i2c/slave/plib_smb2_slave.c  .generated_files/flags/default/93341471dd8c442a9d7a6bd46d5856f70e0b500a .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1917247984" 
	@${RM} ${OBJECTDIR}/_ext/1917247984/plib_smb2_slave.o.d 
	@${RM} ${OBJECTDIR}/_ext/1917247984/plib_smb2_slave.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1917247984/plib_smb2_slave.o.d" -o ${OBJECTDIR}/_ext/1917247984/plib_smb2_slave.o ../src/config/default/peripheral/i2c/slave/plib_smb2_slave.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1917247984/plib_smb3_slave.o: ../src/config/default/peripheral/i2c/slave/plib_smb3_slave.c  .generated_files/flags/default/391221d4fd0aa268630e7dd2e71c816704f6d659 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1917247984" 
	@${RM} ${OBJECTDIR}/_ext/1917247984/plib_smb3_slave.o.d 
	@${RM} ${OBJECTDIR}/_ext/1917247984/plib_smb3_slave.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1917247984/plib_smb3_slave.o.d" -o ${OBJECTDIR}/_ext/1917247984/plib_smb3_slave.o ../src/config/default/peripheral/i2c/slave/plib_smb3_slave.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1917247984/plib_smb4_slave.o: ../src/config/default/peripheral/i2c/slave/plib_smb4_slave.c  .generated_files/flags/default/b174b3b8759bfb10af55e4d9028b442719cf3a45 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1917247984" 
	@${RM} ${OBJECTDIR}/_ext/1917247984/plib_smb4_slave.o.d 
	@${RM} ${OBJECTDIR}/_ext/1917247984/plib_smb4_slave.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1917247984/plib_smb4_slave.o.d" -o ${OBJECTDIR}/_ext/1917247984/plib_smb4_slave.o ../src/config/default/peripheral/i2c/slave/plib_smb4_slave.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60169480/plib_smb0_master_slave_common.o: ../src/config/default/peripheral/i2c/plib_smb0_master_slave_common.c  .generated_files/flags/default/2237c534fba1cf83cd64de3e99189a2200e0ec11 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/60169480" 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_smb0_master_slave_common.o.d 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_smb0_master_slave_common.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/60169480/plib_smb0_master_slave_common.o.d" -o ${OBJECTDIR}/_ext/60169480/plib_smb0_master_slave_common.o ../src/config/default/peripheral/i2c/plib_smb0_master_slave_common.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60169480/plib_smb1_master_slave_common.o: ../src/config/default/peripheral/i2c/plib_smb1_master_slave_common.c  .generated_files/flags/default/322dba0375b15739205e4699db8c7856f0e31a38 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/60169480" 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_smb1_master_slave_common.o.d 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_smb1_master_slave_common.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/60169480/plib_smb1_master_slave_common.o.d" -o ${OBJECTDIR}/_ext/60169480/plib_smb1_master_slave_common.o ../src/config/default/peripheral/i2c/plib_smb1_master_slave_common.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60169480/plib_i2c_smb_common.o: ../src/config/default/peripheral/i2c/plib_i2c_smb_common.c  .generated_files/flags/default/31bb9858aba222e1dd18287283d6259a9deb34ad .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/60169480" 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_i2c_smb_common.o.d 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_i2c_smb_common.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/60169480/plib_i2c_smb_common.o.d" -o ${OBJECTDIR}/_ext/60169480/plib_i2c_smb_common.o ../src/config/default/peripheral/i2c/plib_i2c_smb_common.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60169480/plib_smb4_master_slave_common.o: ../src/config/default/peripheral/i2c/plib_smb4_master_slave_common.c  .generated_files/flags/default/2002444da99f71d61267f7806b32baeea5a9ee50 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/60169480" 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_smb4_master_slave_common.o.d 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_smb4_master_slave_common.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/60169480/plib_smb4_master_slave_common.o.d" -o ${OBJECTDIR}/_ext/60169480/plib_smb4_master_slave_common.o ../src/config/default/peripheral/i2c/plib_smb4_master_slave_common.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60169480/plib_smb2_master_slave_common.o: ../src/config/default/peripheral/i2c/plib_smb2_master_slave_common.c  .generated_files/flags/default/bb6be5ccc91f4c46ac38f1f49f5c42544705bf96 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/60169480" 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_smb2_master_slave_common.o.d 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_smb2_master_slave_common.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/60169480/plib_smb2_master_slave_common.o.d" -o ${OBJECTDIR}/_ext/60169480/plib_smb2_master_slave_common.o ../src/config/default/peripheral/i2c/plib_smb2_master_slave_common.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60169480/plib_smb3_master_slave_common.o: ../src/config/default/peripheral/i2c/plib_smb3_master_slave_common.c  .generated_files/flags/default/797d04a193b4ab469aaade136b350069aced20f9 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/60169480" 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_smb3_master_slave_common.o.d 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_smb3_master_slave_common.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/60169480/plib_smb3_master_slave_common.o.d" -o ${OBJECTDIR}/_ext/60169480/plib_smb3_master_slave_common.o ../src/config/default/peripheral/i2c/plib_smb3_master_slave_common.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60169480/plib_i2c_smbus_lowlevel_intf.o: ../src/config/default/peripheral/i2c/plib_i2c_smbus_lowlevel_intf.c  .generated_files/flags/default/e955c0a801781302b02964a46ea71946d72a8b96 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/60169480" 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_i2c_smbus_lowlevel_intf.o.d 
	@${RM} ${OBJECTDIR}/_ext/60169480/plib_i2c_smbus_lowlevel_intf.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/60169480/plib_i2c_smbus_lowlevel_intf.o.d" -o ${OBJECTDIR}/_ext/60169480/plib_i2c_smbus_lowlevel_intf.o ../src/config/default/peripheral/i2c/plib_i2c_smbus_lowlevel_intf.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1865468468/plib_nvic.o: ../src/config/default/peripheral/nvic/plib_nvic.c  .generated_files/flags/default/cef70ccd3e49f59d56287212c04e913a9846615f .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1865468468" 
	@${RM} ${OBJECTDIR}/_ext/1865468468/plib_nvic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865468468/plib_nvic.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1865468468/plib_nvic.o.d" -o ${OBJECTDIR}/_ext/1865468468/plib_nvic.o ../src/config/default/peripheral/nvic/plib_nvic.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60177741/plib_pcr.o: ../src/config/default/peripheral/pcr/plib_pcr.c  .generated_files/flags/default/7cfa3929a8f92f5840a24139aed7a67833e7e10c .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/60177741" 
	@${RM} ${OBJECTDIR}/_ext/60177741/plib_pcr.o.d 
	@${RM} ${OBJECTDIR}/_ext/60177741/plib_pcr.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/60177741/plib_pcr.o.d" -o ${OBJECTDIR}/_ext/60177741/plib_pcr.o ../src/config/default/peripheral/pcr/plib_pcr.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1997460222/plib_qmspi1.o: ../src/config/default/peripheral/qmspi/plib_qmspi1.c  .generated_files/flags/default/88e5369b52124817743993ea05810a1a7c011809 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1997460222" 
	@${RM} ${OBJECTDIR}/_ext/1997460222/plib_qmspi1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1997460222/plib_qmspi1.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1997460222/plib_qmspi1.o.d" -o ${OBJECTDIR}/_ext/1997460222/plib_qmspi1.o ../src/config/default/peripheral/qmspi/plib_qmspi1.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1997460222/plib_qmspi0.o: ../src/config/default/peripheral/qmspi/plib_qmspi0.c  .generated_files/flags/default/ef65a35e1f4e4e7cbd274e574e2797107868d347 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1997460222" 
	@${RM} ${OBJECTDIR}/_ext/1997460222/plib_qmspi0.o.d 
	@${RM} ${OBJECTDIR}/_ext/1997460222/plib_qmspi0.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1997460222/plib_qmspi0.o.d" -o ${OBJECTDIR}/_ext/1997460222/plib_qmspi0.o ../src/config/default/peripheral/qmspi/plib_qmspi0.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1865657120/plib_uart0.o: ../src/config/default/peripheral/uart/plib_uart0.c  .generated_files/flags/default/3c041efba3a06552f3b1783b4268df273f80fb58 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1865657120" 
	@${RM} ${OBJECTDIR}/_ext/1865657120/plib_uart0.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865657120/plib_uart0.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1865657120/plib_uart0.o.d" -o ${OBJECTDIR}/_ext/1865657120/plib_uart0.o ../src/config/default/peripheral/uart/plib_uart0.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1171490990/exceptions.o: ../src/config/default/exceptions.c  .generated_files/flags/default/69c1d4518facb9e4d8ea9c56e7f332ac9b212f3e .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/exceptions.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1171490990/exceptions.o.d" -o ${OBJECTDIR}/_ext/1171490990/exceptions.o ../src/config/default/exceptions.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1171490990/interrupts.o: ../src/config/default/interrupts.c  .generated_files/flags/default/5718c1919a45d6af226d621adf08b676328370f2 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/interrupts.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/interrupts.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1171490990/interrupts.o.d" -o ${OBJECTDIR}/_ext/1171490990/interrupts.o ../src/config/default/interrupts.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1171490990/initialization.o: ../src/config/default/initialization.c  .generated_files/flags/default/c84bf39fb5d83cd1d43d2dcb502d507d58788ae2 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/initialization.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/initialization.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1171490990/initialization.o.d" -o ${OBJECTDIR}/_ext/1171490990/initialization.o ../src/config/default/initialization.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/222663291/gpio_api.o: ../src/hal/gpio/gpio_api.c  .generated_files/flags/default/f623153360d7ddf75681469fe355dd6968962cb1 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/222663291" 
	@${RM} ${OBJECTDIR}/_ext/222663291/gpio_api.o.d 
	@${RM} ${OBJECTDIR}/_ext/222663291/gpio_api.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/222663291/gpio_api.o.d" -o ${OBJECTDIR}/_ext/222663291/gpio_api.o ../src/hal/gpio/gpio_api.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/684516973/interrupt_api.o: ../src/hal/interrupt/interrupt_api.c  .generated_files/flags/default/4fdd2b337c9b1b6223ee01a431d081653ab0524d .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/684516973" 
	@${RM} ${OBJECTDIR}/_ext/684516973/interrupt_api.o.d 
	@${RM} ${OBJECTDIR}/_ext/684516973/interrupt_api.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/684516973/interrupt_api.o.d" -o ${OBJECTDIR}/_ext/684516973/interrupt_api.o ../src/hal/interrupt/interrupt_api.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1696528154/qmspi_api.o: ../src/hal/qmspi/qmspi_api.c  .generated_files/flags/default/8523f7814e49b1e47134862a144cd9e0fe210c9a .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1696528154" 
	@${RM} ${OBJECTDIR}/_ext/1696528154/qmspi_api.o.d 
	@${RM} ${OBJECTDIR}/_ext/1696528154/qmspi_api.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1696528154/qmspi_api.o.d" -o ${OBJECTDIR}/_ext/1696528154/qmspi_api.o ../src/hal/qmspi/qmspi_api.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/222260348/uart_api.o: ../src/hal/uart/uart_api.c  .generated_files/flags/default/b51429ab60c9fced866a7235bbce8d9c39cc29d2 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/222260348" 
	@${RM} ${OBJECTDIR}/_ext/222260348/uart_api.o.d 
	@${RM} ${OBJECTDIR}/_ext/222260348/uart_api.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/222260348/uart_api.o.d" -o ${OBJECTDIR}/_ext/222260348/uart_api.o ../src/hal/uart/uart_api.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/174097801/main.o: ../src/kernel/main.c  .generated_files/flags/default/c082637f242b3630b15fb77e4d63241c3406ee4 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/174097801" 
	@${RM} ${OBJECTDIR}/_ext/174097801/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/174097801/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/174097801/main.o.d" -o ${OBJECTDIR}/_ext/174097801/main.o ../src/kernel/main.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1203168206/oem_task1.o: ../src/oem/oem_task1/oem_task1.c  .generated_files/flags/default/168a5687cd68368713a83dbe4d7d3cf04094c04e .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1203168206" 
	@${RM} ${OBJECTDIR}/_ext/1203168206/oem_task1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1203168206/oem_task1.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1203168206/oem_task1.o.d" -o ${OBJECTDIR}/_ext/1203168206/oem_task1.o ../src/oem/oem_task1/oem_task1.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1203168205/oem_task2.o: ../src/oem/oem_task2/oem_task2.c  .generated_files/flags/default/14973ebfcb646f3af9b6787617366b977afb6083 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1203168205" 
	@${RM} ${OBJECTDIR}/_ext/1203168205/oem_task2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1203168205/oem_task2.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1203168205/oem_task2.o.d" -o ${OBJECTDIR}/_ext/1203168205/oem_task2.o ../src/oem/oem_task2/oem_task2.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1203168204/oem_task3.o: ../src/oem/oem_task3/oem_task3.c  .generated_files/flags/default/13b730bdb672294d6da69da124a01e67d15ed16d .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1203168204" 
	@${RM} ${OBJECTDIR}/_ext/1203168204/oem_task3.o.d 
	@${RM} ${OBJECTDIR}/_ext/1203168204/oem_task3.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1203168204/oem_task3.o.d" -o ${OBJECTDIR}/_ext/1203168204/oem_task3.o ../src/oem/oem_task3/oem_task3.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1756295213/irqhandler.o: ../src/platform/irqhandler.c  .generated_files/flags/default/5a98946d2aa81410f1f97f26bc4a822f014e060 .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1756295213" 
	@${RM} ${OBJECTDIR}/_ext/1756295213/irqhandler.o.d 
	@${RM} ${OBJECTDIR}/_ext/1756295213/irqhandler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1756295213/irqhandler.o.d" -o ${OBJECTDIR}/_ext/1756295213/irqhandler.o ../src/platform/irqhandler.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1756295213/platform_serial_flash.o: ../src/platform/platform_serial_flash.c  .generated_files/flags/default/61c97dc391b7b762fbab202f06ea400ac29517ca .generated_files/flags/default/a8f3454f8687fb695a83edf43efec714da0742cc
	@${MKDIR} "${OBJECTDIR}/_ext/1756295213" 
	@${RM} ${OBJECTDIR}/_ext/1756295213/platform_serial_flash.o.d 
	@${RM} ${OBJECTDIR}/_ext/1756295213/platform_serial_flash.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"../src" -I"../src/config/default" -I"../src/packs/CEC1736_S0_2ZW_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/common/debug" -I"../src/common/include" -I"../src/hal" -I"../src/kernel" -I"../src/oem" -I"../src/platform" -MP -MMD -MF "${OBJECTDIR}/_ext/1756295213/platform_serial_flash.o.d" -o ${OBJECTDIR}/_ext/1756295213/platform_serial_flash.o ../src/platform/platform_serial_flash.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/cec1736" ${PACK_COMMON_OPTIONS} 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${DISTDIR}/sg3_h3_port.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../src/config/default/library/secureboot_LIB/secureboot_LIB.a ../src/config/default/library/secureboot_app_lib/secureboot_app_lib.a  ../src/common/include/secureboot_app.ld
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g   -mprocessor=$(MP_PROCESSOR_OPTION) -mno-device-startup-code -o ${DISTDIR}/sg3_h3_port.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ..\src\config\default\library\secureboot_LIB\secureboot_LIB.a ..\src\config\default\library\secureboot_app_lib\secureboot_app_lib.a      -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=_min_heap_size=512,--gc-sections,-L"../src/config/default/library/secureboot_app_lib",-L"../src/config/default/library/secureboot_LIB",-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,${DISTDIR}/memoryfile.xml,-u,irqhandler_tag -mdfp="${DFP_DIR}/cec1736"
	
else
${DISTDIR}/sg3_h3_port.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../src/config/default/library/secureboot_LIB/secureboot_LIB.a ../src/config/default/library/secureboot_app_lib/secureboot_app_lib.a ../src/common/include/secureboot_app.ld
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION) -mno-device-startup-code -o ${DISTDIR}/sg3_h3_port.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ..\src\config\default\library\secureboot_LIB\secureboot_LIB.a ..\src\config\default\library\secureboot_app_lib\secureboot_app_lib.a      -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=512,--gc-sections,-L"../src/config/default/library/secureboot_app_lib",-L"../src/config/default/library/secureboot_LIB",-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,${DISTDIR}/memoryfile.xml,-u,irqhandler_tag -mdfp="${DFP_DIR}/cec1736"
	${MP_CC_DIR}\\xc32-bin2hex ${DISTDIR}/sg3_h3_port.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${OBJECTDIR}
	${RM} -r ${DISTDIR}

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
