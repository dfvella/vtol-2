#!/bin/sh

# A script for flashing the Raspberry Pi Pico

BUILD_DIR=/home/david/projects/vtol-2/build
BUILD_TAR=main

if [ -n "$1" ]; then
    BUILD_TAR="$1"
fi

DEVICE_PATH=$(find /dev -wholename '/dev/sd[a-b]1')
MOUNT_PATH=/dev/pico

if [ -z "$DEVICE_PATH" ]; then
    echo "error: failed to find the board"
    exit 1
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

exit 0
