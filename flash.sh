#!/bin/sh

DEVICE_PATH=/dev/sda1
MOUNT_PATH=/mnt/pico

TARGET_PATH=build/src/main

if [ $EUID -ne 0 ]; then
    echo "error: script must be run as root"
    exit 1
fi

if [ -n "$1" ]; then
    TARGET_PATH="$1"
fi

BUILD_TARGET=$(echo $TARGET_PATH | cut -d '/' -f 3)
# BUILD_DIR=$(echo $TARGET_PATH | cut -d '/' -f 1-2)

# echo "Building target at $TARGET_PATH ..."

# cd $BUILD_DIR || exit 1
# make $BUILD_TARGET || exit 1
# cd ..

echo "Flashing board at $DEVICE_PATH with target $TARGET_PATH ..."

echo "Mounting $DEVICE_PATH to $MOUNT_PATH ..."
mkdir -p $MOUNT_PATH
mount $DEVICE_PATH $MOUNT_PATH || exit 1

echo "Copying $TARGET_PATH.uf2 to $MOUNT_PATH ..."
cp $TARGET_PATH.uf2 $MOUNT_PATH || exit 1
sync

echo "Un-mounting board at $DEVICE_PATH ..."
umount $DEVICE_PATH

echo "Finished"
