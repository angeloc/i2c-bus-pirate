#!/bin/sh

sudo killall inputattach
sudo rmmod i2c-bus-pirate
sudo insmod i2c-bus-pirate.ko
sudo inputattach -buspirate /dev/ttyUSB0 &
sleep 2

sudo sh -c "echo 1 > /sys/bus/i2c/devices/i2c-0/psu_enable"
sudo sh -c "echo 1 > /sys/bus/i2c/devices/i2c-0/pullup_enable"

sudo rmmod lm75
sudo modprobe lm75
sudo sh -c "echo lm75 0x48 > /sys/bus/i2c/devices/i2c-0/new_device"
cat /sys/bus/i2c/devices/0-0048/temp1_input
