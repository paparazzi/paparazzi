#!/bin/bash

# handle input/eventX devices for joystick devices
# copy it to /lib/udev/input_event.sh

# declarations from <linux/input>
EV_ABS=0x03
EV_FF=0x15
ABS_X=0x00
ABS_THROTTLE=0x06
ABS_WHEEL=0x08

ABS_ALL=$(( 1 << $ABS_X | 1 << $ABS_THROTTLE | 1 << $ABS_WHEEL))

# get capabilities from parent input device
parent=${1%%event*}
EV="0x$(< /sys${parent}capabilities/ev)"
ABS="0x$(< /sys${parent}capabilities/abs)"

if (( $EV & (1 << $EV_ABS) )) && (( $ABS & $ABS_ALL )); then
	if (( $EV & (1 << $EV_FF) )); then 
		echo "FF_DEVICE=1";
	else
		echo "FF_DEVICE=0";
	fi
	exit  0
fi

# no joystick device
exit 1
