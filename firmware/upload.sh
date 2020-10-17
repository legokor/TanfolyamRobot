#!/bin/bash

if [ $# -ne 1 ]; then
	echo "Usage: $0 port"
	exit 1
fi

stty -F $1 115200

echo -n ENTER_DFU > $1
sleep 0.1

stm32flash -b 115200 -w Debug/tanfolyamrobot.bin -v -g 0x0 $1
