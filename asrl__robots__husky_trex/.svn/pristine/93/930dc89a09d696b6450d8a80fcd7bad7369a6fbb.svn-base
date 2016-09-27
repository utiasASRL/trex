#!/bin/bash

# require password
#sudo chmod a+rw /dev/ttyS0

#don't require password
echo robots | sudo -S chmod a+rw /dev/ttyS0

# Set up the gamepad
sudo rmmod xpad
sudo modprobe uinput
sudo modprobe joydev
sudo xboxdrv
