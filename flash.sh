#!/bin/sh

# cmake wrapper for vtol-2 project

SRC_DIR=/home/david/projects/vtol-2
BUILD_DIR=/home/david/projects/vtol-2/build
BUILD_CFG=Debug
BUILD_TAR=all

MSG_HELP="cmake wrapper for vtol-2 project
  -b,--build    path to build directory
  -c,--config   cmake build type: Debug or Release
  -t,--target   build target or phony target all
  -h,--help     display this help message"

OPTIONS='c:t:h'
LONGOPT='config:,target:,help'

getopt --test &> /dev/null
if [ $? -ne 4 ]; then
    echo "error: getopt not available in this environment"
    exit 1
fi

PARSED=$(getopt --options=$OPTIONS --longoptions=$LONGOPT --name $0 -- $@)
if [ $? -ne 0 ]; then
    exit 1
fi
eval set -- "$PARSED"

while true; do
    case "$1" in
        -c|--config)
            BUILD_CFG="$2"
            shift 2
            ;;
        -t|--target)
            BUILD_TAR="$2"
            shift 2
            ;;
        -h|--help)
            echo "$MSG_HELP"
            exit 0
            ;;
        --)
            shift
            break
            ;;
    esac
done

# configure cmake project
(set -x ; cmake -DCMAKE_BUILD_TYPE:STRING=$BUILD_CFG -B $BUILD_DIR -S $SRC_DIR) || exit 1

# build cmake project
(set -x ; cmake --build $BUILD_DIR --config $BUILD_CFG --target $BUILD_TAR) || exit 1

# flash the Raspberry Pi Pico if connected over USB
DEVICE_PATH=$(find /dev -wholename '/dev/sd[a-b]1')
MOUNT_PATH=/dev/pico

if [ -n "$DEVICE_PATH" ]; then
    echo "Found board $DEVICE_PATH"

    # if phony target all was used to build, flash board with main
    if [ "$BUILD_TAR" = "all" ]; then
        BUILD_TAR=main
    fi

    TARGET_PATH=$(find $BUILD_DIR -name $BUILD_TAR.uf2)
    
    if [ -z "$TARGET_PATH" ]; then
        echo "error: failed to find target $BUILD_TAR.uf2"
        exit 1
    fi

    echo "mounting $DEVICE_PATH to $MOUNT_PATH"
    sudo mkdir -p $MOUNT_PATH
    sudo mount $DEVICE_PATH $MOUNT_PATH || exit 1
    
    echo "copying $TARGET_PATH to $MOUNT_PATH"
    sudo cp $TARGET_PATH $MOUNT_PATH || exit 1
    sudo sync

    echo "un-mounting board at $DEVICE_PATH"
    sudo umount $DEVICE_PATH
else    
    echo "could not find board, compiling only"
fi

echo "===== BUILD SUMMARY ====="
echo "target : $BUILD_TAR"
echo "config : $BUILD_CFG"
