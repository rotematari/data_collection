#!/bin/bash
VID=2833
PID=0209

for dev in $(lsusb | grep "$VID:$PID" | awk '{print "/dev/bus/usb/" $2 "/" $4}' | sed 's/://'); do
    echo "Resetting $dev"
    sudo usbreset "$dev"
done

