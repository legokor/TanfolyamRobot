#!/bin/bash

APPLICATION_BAUD=115200 # Baud rate for communication with the firmware
DFU_COMMAND="ENTER_DFU" # Serial command for entering DFU mode
UPLOAD_BAUD=500000      # Baud rate for stm32flash utility

# Check parameters
if [ $# -ne 1 ]; then
	echo "Usage: $0 port"
	exit 1
fi

# Set baud rate and send ENTER_DFU command
stty -F $1 $APPLICATION_BAUD
echo "Enter DFU mode..."
echo -n $DFU_COMMAND > $1

# Wait for system reset
sleep 1

# Measure upload time
START_TIME=$(date +%s.%N)

# Flash MCU
./stm32flash -b $UPLOAD_BAUD -w ../Debug/tanfolyamrobot.bin -v -g 0x0 $1

# Calculate and print upload time
END_TIME=$(date +%s.%N)
DIFF=$(echo "$END_TIME - $START_TIME" | bc)
echo "Upload time: $DIFF"
