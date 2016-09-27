#!/bin/bash

# Set up the robot
sudo $(rospack find digi_realport_serial_ethernet)/scripts/dgrp-script.sh start 2 192.168.0.51

sudo chmod a+rw /dev/tty200

# Set up the gamepad
sudo rmmod xpad
sudo modprobe uinput
sudo modprobe joydev
sudo xboxdrv
