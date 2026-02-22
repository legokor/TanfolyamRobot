#!/bin/bash

# Check if the script is run with superuser privileges
if [[ $EUID -ne 0 ]]; then
    echo "This script must be run as root or with sudo."
    exit 1
fi

# Define the udev rule file path
UDEV_RULE_FILE="/etc/udev/rules.d/99-ftdi.rules"

# Create the udev rule
echo "Creating udev rule for FTDI FT232 USB-serial converter..."

cat > "$UDEV_RULE_FILE" <<EOL
# FTDI FT232 USB-serial converter
SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6001", MODE="0666", GROUP="dialout"
EOL

echo "Udev rule created at $UDEV_RULE_FILE."

# Reload udev rules
echo "Reloading udev rules..."
udevadm control --reload-rules

# Check if the user is provided as an argument
if [ -z "$1" ]; then
    echo "No username provided. Please add a user to the 'dialout' group manually."
    exit 1
fi

# Add the user to the dialout group
USER="$1"
echo "Adding user '$USER' to the 'dialout' group..."
usermod -aG dialout "$USER"

echo "User '$USER' added to the 'dialout' group."

# Final message
echo "Installation complete. Please log out and log back in for the changes to take effect."
