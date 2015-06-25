#!/bin/sh

sudo killall inputattach
sudo rmmod i2c-bus-pirate
sudo insmod i2c-bus-pirate.ko
sudo inputattach -buspirate /dev/ttyUSB0 &
sleep 2

sudo sh -c "echo 1 > /sys/bus/i2c/devices/i2c-0/psu_enable"
sudo sh -c "echo 1 > /sys/bus/i2c/devices/i2c-0/pullup_enable"

sudo rmmod rtc-ds1307
sudo modprobe rtc-ds1307
sudo sh -c "echo ds1307 0x68 > /sys/bus/i2c/devices/i2c-0/new_device"
cat /sys/bus/i2c/devices/0-0068/rtc/rtc1/time
sudo hwclock -w -f /dev/rtc1
cat /sys/bus/i2c/devices/0-0068/rtc/rtc1/time
