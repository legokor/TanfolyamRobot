#!/bin/bash

APPLICATION_BAUD=115200
UPLOAD_BAUD=500000

if [ $# -ne 1 ]; then
	echo "Usage: $0 port"
	exit 1
fi

stty -F $1 $APPLICATION_BAUD

echo "Enter DFU mode..."
echo -n ENTER_DFU > $1

sleep 1

START_TIME=$(date +%s.%N)

stm32flash -b $UPLOAD_BAUD -w Debug/tanfolyamrobot.bin -v -g 0x0 $1

END_TIME=$(date +%s.%N)
DIFF=$(echo "$END_TIME - $START_TIME" | bc)
echo "Upload time: $DIFF"