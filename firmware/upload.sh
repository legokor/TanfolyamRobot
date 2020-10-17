#!/bin/bash

if [ $# -ne 1 ]; then
	echo "Usage: $0 port"
	exit 1
fi

stm32flash -w Debug/tanfolyamrobot.bin -v -g 0x0 $1