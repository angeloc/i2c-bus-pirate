#!/bin/sh

sudo killall inputattach
sudo rmmod i2c-bus-pirate
sudo insmod i2c-bus-pirate.ko
sudo inputattach -buspirate /dev/ttyUSB0 &
sleep 2

sudo sh -c "echo 1 > /sys/bus/i2c/devices/i2c-0/psu_enable"
sudo sh -c "echo 1 > /sys/bus/i2c/devices/i2c-0/pullup_enable"

sudo rmmod eeprom
sudo modprobe eeprom
sudo sh -c "echo 24c32 0x50 > /sys/bus/i2c/devices/i2c-0/new_device"
sudo sh -c "echo It works > /sys/bus/i2c/devices/0-0050/eeprom"
sudo sh -c "head -c9 /sys/bus/i2c/devices/0-0050/eeprom"
