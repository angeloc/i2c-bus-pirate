# i2c-bus-pirate
Linux kernel module for Bus Pirate as I2C bus adapter

To use this kernel module you need the modified inputattach binary from
[https://github.com/angeloc/linuxconsoletools-buspirate]

```
make
sudo insmod i2c-bus-pirate.ko
sudo inputattach -buspirate /dev/ttyUSB0 &

sudo sh -c "echo 1 > /sys/bus/i2c/devices/i2c-0/psu_enable"
sudo sh -c "echo 1 > /sys/bus/i2c/devices/i2c-0/pullup_enable"

sudo modprobe lm75
sudo sh -c "echo lm75 0x48 > /sys/bus/i2c/devices/i2c-0/new_device"
cat /sys/bus/i2c/devices/0-0048/temp1_input
```
