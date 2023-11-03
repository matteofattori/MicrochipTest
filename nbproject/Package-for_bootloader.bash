#!/bin/bash -x

#
# Generated - do not edit!
#

# Macros
TOP=`pwd`
CND_CONF=for_bootloader
CND_DISTDIR=dist
TMPDIR=build/${CND_CONF}/${IMAGE_TYPE}/tmp-packaging
TMPDIRNAME=tmp-packaging
OUTPUT_PATH=dist/${CND_CONF}/${IMAGE_TYPE}/XWB11635_fw.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
OUTPUT_BASENAME=XWB11635_fw.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
PACKAGE_TOP_DIR=xwb11635fw/

# Functions
function checkReturnCode
{
    rc=$?
    if [ $rc != 0 ]
    then
        exit $rc
    fi
}
function makeDirectory
# $1 directory path
# $2 permission (optional)
{
    mkdir -p "$1"
    checkReturnCode
    if [ "$2" != "" ]
    then
      chmod $2 "$1"
      checkReturnCode
    fi
}
function copyFileToTmpDir
# $1 from-file path
# $2 to-file path
# $3 permission
{
    cp "$1" "$2"
    checkReturnCode
    if [ "$3" != "" ]
    then
        chmod $3 "$2"
        checkReturnCode
    fi
}

# Setup
cd "${TOP}"
mkdir -p ${CND_DISTDIR}/${CND_CONF}/package
rm -rf ${TMPDIR}
mkdir -p ${TMPDIR}

# Copy files and create directories and links
cd "${TOP}"
makeDirectory ${TMPDIR}/xwb11635fw/bin
copyFileToTmpDir "${OUTPUT_PATH}" "${TMPDIR}/${PACKAGE_TOP_DIR}bin/${OUTPUT_BASENAME}" 0755


# Generate tar file
cd "${TOP}"
rm -f ${CND_DISTDIR}/${CND_CONF}/package/xwb11635fw.tar
cd ${TMPDIR}
tar -vcf ../../../../${CND_DISTDIR}/${CND_CONF}/package/xwb11635fw.tar *
checkReturnCode

# Cleanup
cd "${TOP}"
rm -rf ${TMPDIR}
