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
OUTPUT_SUFFIX=a
DEBUGGABLE_SUFFIX=a
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/libsmps_control_dspic33e-elf.X.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=a
DEBUGGABLE_SUFFIX=a
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/libsmps_control_dspic33e-elf.X.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../../../src/smps_2p2z_dspic.s ../../../src/smps_2p2z_dspic_v2.s ../../../src/smps_3p3z_dspic.s ../../../src/smps_3p3z_dspic_v2.s ../../../src/smps_4p4z_dspic_v2.s ../../../src/smps_pid_dspic.s ../../../src/smps_pid_dspic_v2.s

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic.o ${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic_v2.o ${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic.o ${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic_v2.o ${OBJECTDIR}/_ext/1386528437/smps_4p4z_dspic_v2.o ${OBJECTDIR}/_ext/1386528437/smps_pid_dspic.o ${OBJECTDIR}/_ext/1386528437/smps_pid_dspic_v2.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic.o.d ${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic_v2.o.d ${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic.o.d ${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic_v2.o.d ${OBJECTDIR}/_ext/1386528437/smps_4p4z_dspic_v2.o.d ${OBJECTDIR}/_ext/1386528437/smps_pid_dspic.o.d ${OBJECTDIR}/_ext/1386528437/smps_pid_dspic_v2.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic.o ${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic_v2.o ${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic.o ${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic_v2.o ${OBJECTDIR}/_ext/1386528437/smps_4p4z_dspic_v2.o ${OBJECTDIR}/_ext/1386528437/smps_pid_dspic.o ${OBJECTDIR}/_ext/1386528437/smps_pid_dspic_v2.o

# Source Files
SOURCEFILES=../../../src/smps_2p2z_dspic.s ../../../src/smps_2p2z_dspic_v2.s ../../../src/smps_3p3z_dspic.s ../../../src/smps_3p3z_dspic_v2.s ../../../src/smps_4p4z_dspic_v2.s ../../../src/smps_pid_dspic.s ../../../src/smps_pid_dspic_v2.s


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
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/libsmps_control_dspic33e-elf.X.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33EP64GS502
MP_LINKER_FILE_OPTION=,--script=p33EP64GS502.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic.o: ../../../src/smps_2p2z_dspic.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1386528437" 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../../../src/smps_2p2z_dspic.s  -o ${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic_v2.o: ../../../src/smps_2p2z_dspic_v2.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1386528437" 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic_v2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic_v2.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../../../src/smps_2p2z_dspic_v2.s  -o ${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic_v2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic_v2.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic_v2.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic.o: ../../../src/smps_3p3z_dspic.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1386528437" 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../../../src/smps_3p3z_dspic.s  -o ${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic_v2.o: ../../../src/smps_3p3z_dspic_v2.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1386528437" 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic_v2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic_v2.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../../../src/smps_3p3z_dspic_v2.s  -o ${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic_v2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic_v2.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic_v2.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1386528437/smps_4p4z_dspic_v2.o: ../../../src/smps_4p4z_dspic_v2.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1386528437" 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_4p4z_dspic_v2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_4p4z_dspic_v2.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../../../src/smps_4p4z_dspic_v2.s  -o ${OBJECTDIR}/_ext/1386528437/smps_4p4z_dspic_v2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/_ext/1386528437/smps_4p4z_dspic_v2.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1386528437/smps_4p4z_dspic_v2.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1386528437/smps_pid_dspic.o: ../../../src/smps_pid_dspic.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1386528437" 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_pid_dspic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_pid_dspic.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../../../src/smps_pid_dspic.s  -o ${OBJECTDIR}/_ext/1386528437/smps_pid_dspic.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/_ext/1386528437/smps_pid_dspic.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1386528437/smps_pid_dspic.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1386528437/smps_pid_dspic_v2.o: ../../../src/smps_pid_dspic_v2.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1386528437" 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_pid_dspic_v2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_pid_dspic_v2.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../../../src/smps_pid_dspic_v2.s  -o ${OBJECTDIR}/_ext/1386528437/smps_pid_dspic_v2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/_ext/1386528437/smps_pid_dspic_v2.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1386528437/smps_pid_dspic_v2.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic.o: ../../../src/smps_2p2z_dspic.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1386528437" 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../../../src/smps_2p2z_dspic.s  -o ${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic_v2.o: ../../../src/smps_2p2z_dspic_v2.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1386528437" 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic_v2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic_v2.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../../../src/smps_2p2z_dspic_v2.s  -o ${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic_v2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic_v2.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1386528437/smps_2p2z_dspic_v2.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic.o: ../../../src/smps_3p3z_dspic.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1386528437" 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../../../src/smps_3p3z_dspic.s  -o ${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic_v2.o: ../../../src/smps_3p3z_dspic_v2.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1386528437" 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic_v2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic_v2.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../../../src/smps_3p3z_dspic_v2.s  -o ${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic_v2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic_v2.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1386528437/smps_3p3z_dspic_v2.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1386528437/smps_4p4z_dspic_v2.o: ../../../src/smps_4p4z_dspic_v2.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1386528437" 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_4p4z_dspic_v2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_4p4z_dspic_v2.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../../../src/smps_4p4z_dspic_v2.s  -o ${OBJECTDIR}/_ext/1386528437/smps_4p4z_dspic_v2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/_ext/1386528437/smps_4p4z_dspic_v2.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1386528437/smps_4p4z_dspic_v2.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1386528437/smps_pid_dspic.o: ../../../src/smps_pid_dspic.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1386528437" 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_pid_dspic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_pid_dspic.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../../../src/smps_pid_dspic.s  -o ${OBJECTDIR}/_ext/1386528437/smps_pid_dspic.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/_ext/1386528437/smps_pid_dspic.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1386528437/smps_pid_dspic.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1386528437/smps_pid_dspic_v2.o: ../../../src/smps_pid_dspic_v2.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1386528437" 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_pid_dspic_v2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1386528437/smps_pid_dspic_v2.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../../../src/smps_pid_dspic_v2.s  -o ${OBJECTDIR}/_ext/1386528437/smps_pid_dspic_v2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/_ext/1386528437/smps_pid_dspic_v2.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1386528437/smps_pid_dspic_v2.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: archive
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/libsmps_control_dspic33e-elf.X.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	@${RM} dist/${CND_CONF}/${IMAGE_TYPE}/libsmps_control_dspic33e-elf.X.${OUTPUT_SUFFIX} 
	${MP_AR} $(MP_EXTRA_AR_PRE)  -omf=elf -r dist/${CND_CONF}/${IMAGE_TYPE}/libsmps_control_dspic33e-elf.X.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/libsmps_control_dspic33e-elf.X.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	@${RM} dist/${CND_CONF}/${IMAGE_TYPE}/libsmps_control_dspic33e-elf.X.${OUTPUT_SUFFIX} 
	${MP_AR} $(MP_EXTRA_AR_PRE)  -omf=elf -r dist/${CND_CONF}/${IMAGE_TYPE}/libsmps_control_dspic33e-elf.X.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
