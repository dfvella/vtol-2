#!/bin/sh

# A script for compiling the vtol-2 flight controller project.

BUILD_DIR=/home/david/projects/vtol-2/build
BUILD_TAR=all

if [ $# -gt 1 ]; then
    echo "usage: $0 [build target]"
    exit 1
fi

if [ -n "$1" ]; then
    BUILD_TAR="$1"
fi

(set -x ; cmake --build $BUILD_DIR --target $BUILD_TAR) || exit 1

# if phony target all was used to build, default to main
if [ "$BUILD_TAR" = "all" ]; then
    BUILD_TAR=main
fi

# check the size of the program
TARGET_ELF=$(find $BUILD_DIR -name $BUILD_TAR.elf)

if [ -z "$TARGET_ELF" ]; then
    echo "error: failed to find target $BUILD_TAR.elf"
    exit 1
fi

PROGRAM_END=$(objdump --all $TARGET_ELF | grep flash_binary_end | cut -d' ' -f1)
XIP_BASE="10000000"
PROGRAM_SIZE=$((16#$PROGRAM_END - 16#$XIP_BASE))

# check if the program size interferes with flash logging
TARGET_SRC=$(find . -type f -name $BUILD_TAR.c)
USING_LOGGING=$(grep '#include "logging.h"' $TARGET_SRC)
if [ -n "$USING_LOGGING" ]; then
    LOG_FLASH_START=$(grep '#define LOG_FLASH_START' src/logging.h | cut -d' ' -f3)
    LOG_FLASH_START=$(( $LOG_FLASH_START * 1024 ))
    if [ $PROGRAM_SIZE -gt $LOG_FLASH_START ]; then
        echo "error: program interferes with flash logging"
        echo "program uses the first $PROGRAM_SIZE Bytes of flash but flash logging start at $LOG_FLASH_START"
        exit 1
    fi
fi

echo "compilation complete"
echo "program $BUILD_TAR uses $PROGRAM_SIZE Bytes of flash"
