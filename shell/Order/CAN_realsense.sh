#!/bin/bash
modprobe gs_usb
sudo -S ip link set can0 up type can bitrate 500000 << EOF
nvidia
EOF


