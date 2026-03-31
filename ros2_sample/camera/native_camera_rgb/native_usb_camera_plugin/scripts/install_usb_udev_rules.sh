#!/bin/bash

# Check if user is root/running with sudo
if [ "$(whoami)" != root ]; then
  echo Please run this script with sudo
  exit
fi

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

if [ "$(uname -s)" != "Darwin" ]; then
  # Install UDEV rules for USB device
  cp "${CURR_DIR}"/usb_cameras.rules /etc/udev/rules.d/usb_cameras.rules
  echo "usb rules file install at /etc/udev/rules.d/usb_cameras.rules"
fi
echo "reload udev rules"
udevadm control --reload-rules && udevadm trigger
echo "udev rules reload done"
echo "exit"
