#!/bin/bash

# Define the sudoers rule file and content
SUDOERS_FILE="/etc/sudoers.d/modprobe_nopasswd"
RULE="ALL ALL=NOPASSWD: /sbin/modprobe, /usr/sbin/modprobe"

# Check if the rule already exists
if [ -f "$SUDOERS_FILE" ] && grep -q "$RULE" "$SUDOERS_FILE"; then
    echo "The sudoers rule is already present. No changes made."
    exit 0
fi

# Create or overwrite the sudoers file
echo "$RULE" | sudo tee "$SUDOERS_FILE" > /dev/null
sudo chmod 0440 "$SUDOERS_FILE"

# Validate the sudoers file
if sudo visudo -c -f "$SUDOERS_FILE"; then
    echo "Successfully updated sudoers to allow modprobe without password."
else
    echo "Error: Invalid sudoers file. No changes applied."
    sudo rm "$SUDOERS_FILE"
    exit 1
fi

exit 0
