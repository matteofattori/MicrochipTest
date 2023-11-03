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
ifeq "$(wildcard nbproject/Makefile-local-for_bootloader.mk)" "nbproject/Makefile-local-for_bootloader.mk"
include nbproject/Makefile-local-for_bootloader.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=for_bootloader
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/XWB11635_fw.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/XWB11635_fw.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS
SUB_IMAGE_ADDRESS_COMMAND=--image-address $(SUB_IMAGE_ADDRESS)
else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=I2C_driver.c consolle.c control.c main.c math_u.c system.c smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.s smps_control_library_vAugust_05_2016/src/smps_pid_dspic.s i2c_stam.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/I2C_driver.o ${OBJECTDIR}/consolle.o ${OBJECTDIR}/control.o ${OBJECTDIR}/main.o ${OBJECTDIR}/math_u.o ${OBJECTDIR}/system.o ${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.o ${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_pid_dspic.o ${OBJECTDIR}/i2c_stam.o
POSSIBLE_DEPFILES=${OBJECTDIR}/I2C_driver.o.d ${OBJECTDIR}/consolle.o.d ${OBJECTDIR}/control.o.d ${OBJECTDIR}/main.o.d ${OBJECTDIR}/math_u.o.d ${OBJECTDIR}/system.o.d ${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.o.d ${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_pid_dspic.o.d ${OBJECTDIR}/i2c_stam.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/I2C_driver.o ${OBJECTDIR}/consolle.o ${OBJECTDIR}/control.o ${OBJECTDIR}/main.o ${OBJECTDIR}/math_u.o ${OBJECTDIR}/system.o ${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.o ${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_pid_dspic.o ${OBJECTDIR}/i2c_stam.o

# Source Files
SOURCEFILES=I2C_driver.c consolle.c control.c main.c math_u.c system.c smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.s smps_control_library_vAugust_05_2016/src/smps_pid_dspic.s i2c_stam.c


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
	${MAKE}  -f nbproject/Makefile-for_bootloader.mk dist/${CND_CONF}/${IMAGE_TYPE}/XWB11635_fw.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33EP256MC204
MP_LINKER_FILE_OPTION=,--script="app_p33EP256MC204.gld"
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/I2C_driver.o: I2C_driver.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/I2C_driver.o.d 
	@${RM} ${OBJECTDIR}/I2C_driver.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  I2C_driver.c  -o ${OBJECTDIR}/I2C_driver.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/I2C_driver.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -Drelease_app -DXPRJ_for_bootloader=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/I2C_driver.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/consolle.o: consolle.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/consolle.o.d 
	@${RM} ${OBJECTDIR}/consolle.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  consolle.c  -o ${OBJECTDIR}/consolle.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/consolle.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -Drelease_app -DXPRJ_for_bootloader=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/consolle.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/control.o: control.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/control.o.d 
	@${RM} ${OBJECTDIR}/control.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  control.c  -o ${OBJECTDIR}/control.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/control.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -Drelease_app -DXPRJ_for_bootloader=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/control.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -Drelease_app -DXPRJ_for_bootloader=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/math_u.o: math_u.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/math_u.o.d 
	@${RM} ${OBJECTDIR}/math_u.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  math_u.c  -o ${OBJECTDIR}/math_u.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/math_u.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -Drelease_app -DXPRJ_for_bootloader=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/math_u.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/system.o: system.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/system.o.d 
	@${RM} ${OBJECTDIR}/system.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  system.c  -o ${OBJECTDIR}/system.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/system.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -Drelease_app -DXPRJ_for_bootloader=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/system.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/i2c_stam.o: i2c_stam.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/i2c_stam.o.d 
	@${RM} ${OBJECTDIR}/i2c_stam.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  i2c_stam.c  -o ${OBJECTDIR}/i2c_stam.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/i2c_stam.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -Drelease_app -DXPRJ_for_bootloader=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/i2c_stam.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/I2C_driver.o: I2C_driver.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/I2C_driver.o.d 
	@${RM} ${OBJECTDIR}/I2C_driver.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  I2C_driver.c  -o ${OBJECTDIR}/I2C_driver.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/I2C_driver.o.d"      -mno-eds-warn  -g -omf=elf -Drelease_app -DXPRJ_for_bootloader=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/I2C_driver.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/consolle.o: consolle.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/consolle.o.d 
	@${RM} ${OBJECTDIR}/consolle.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  consolle.c  -o ${OBJECTDIR}/consolle.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/consolle.o.d"      -mno-eds-warn  -g -omf=elf -Drelease_app -DXPRJ_for_bootloader=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/consolle.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/control.o: control.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/control.o.d 
	@${RM} ${OBJECTDIR}/control.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  control.c  -o ${OBJECTDIR}/control.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/control.o.d"      -mno-eds-warn  -g -omf=elf -Drelease_app -DXPRJ_for_bootloader=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/control.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d"      -mno-eds-warn  -g -omf=elf -Drelease_app -DXPRJ_for_bootloader=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/math_u.o: math_u.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/math_u.o.d 
	@${RM} ${OBJECTDIR}/math_u.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  math_u.c  -o ${OBJECTDIR}/math_u.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/math_u.o.d"      -mno-eds-warn  -g -omf=elf -Drelease_app -DXPRJ_for_bootloader=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/math_u.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/system.o: system.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/system.o.d 
	@${RM} ${OBJECTDIR}/system.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  system.c  -o ${OBJECTDIR}/system.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/system.o.d"      -mno-eds-warn  -g -omf=elf -Drelease_app -DXPRJ_for_bootloader=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/system.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/i2c_stam.o: i2c_stam.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/i2c_stam.o.d 
	@${RM} ${OBJECTDIR}/i2c_stam.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  i2c_stam.c  -o ${OBJECTDIR}/i2c_stam.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/i2c_stam.o.d"      -mno-eds-warn  -g -omf=elf -Drelease_app -DXPRJ_for_bootloader=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/i2c_stam.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.o: smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/smps_control_library_vAugust_05_2016/src" 
	@${RM} ${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.o.d 
	@${RM} ${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.s  -o ${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -Drelease_app -DXPRJ_for_bootloader=$(CND_CONF)  -legacy-libc  -Wa,-MD,"${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,-g,--no-relax,-ahmlsi=${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.lst$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_pid_dspic.o: smps_control_library_vAugust_05_2016/src/smps_pid_dspic.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/smps_control_library_vAugust_05_2016/src" 
	@${RM} ${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_pid_dspic.o.d 
	@${RM} ${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_pid_dspic.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  smps_control_library_vAugust_05_2016/src/smps_pid_dspic.s  -o ${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_pid_dspic.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -Drelease_app -DXPRJ_for_bootloader=$(CND_CONF)  -legacy-libc  -Wa,-MD,"${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_pid_dspic.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,-g,--no-relax,-ahmlsi=${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_pid_dspic.lst$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_pid_dspic.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.o: smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/smps_control_library_vAugust_05_2016/src" 
	@${RM} ${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.o.d 
	@${RM} ${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.s  -o ${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -Drelease_app -DXPRJ_for_bootloader=$(CND_CONF)  -legacy-libc  -Wa,-MD,"${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax,-ahmlsi=${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.lst$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_3p3z_dspic.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_pid_dspic.o: smps_control_library_vAugust_05_2016/src/smps_pid_dspic.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/smps_control_library_vAugust_05_2016/src" 
	@${RM} ${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_pid_dspic.o.d 
	@${RM} ${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_pid_dspic.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  smps_control_library_vAugust_05_2016/src/smps_pid_dspic.s  -o ${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_pid_dspic.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -Drelease_app -DXPRJ_for_bootloader=$(CND_CONF)  -legacy-libc  -Wa,-MD,"${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_pid_dspic.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax,-ahmlsi=${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_pid_dspic.lst$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/smps_control_library_vAugust_05_2016/src/smps_pid_dspic.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/XWB11635_fw.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  C:/Program\ Files\ (x86)/Microchip/xc16/v1.35/lib/libdsp-elf.a C:/Program\ Files\ (x86)/Microchip/xc16/v1.35/lib/libdsp-elf.a  app_p33EP256MC204.gld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/XWB11635_fw.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    "C:\Program Files (x86)\Microchip\xc16\v1.35\lib\libdsp-elf.a" "C:\Program Files (x86)\Microchip\xc16\v1.35\lib\libdsp-elf.a"  -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG=__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -Drelease_app -DXPRJ_for_bootloader=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  --coresident  -mreserve=data@0x1000:0x101B -mreserve=data@0x101C:0x101D -mreserve=data@0x101E:0x101F -mreserve=data@0x1020:0x1021 -mreserve=data@0x1022:0x1023 -mreserve=data@0x1024:0x1027 -mreserve=data@0x1028:0x104F   -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,--defsym=__MPLAB_DEBUGGER_PK3=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/XWB11635_fw.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  C:/Program\ Files\ (x86)/Microchip/xc16/v1.35/lib/libdsp-elf.a C:/Program\ Files\ (x86)/Microchip/xc16/v1.35/lib/libdsp-elf.a app_p33EP256MC204.gld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/XWB11635_fw.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    "C:\Program Files (x86)\Microchip\xc16\v1.35\lib\libdsp-elf.a" "C:\Program Files (x86)\Microchip\xc16\v1.35\lib\libdsp-elf.a"  -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -Drelease_app -DXPRJ_for_bootloader=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  --coresident -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/XWB11635_fw.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf  
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/for_bootloader
	${RM} -r dist/for_bootloader

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
