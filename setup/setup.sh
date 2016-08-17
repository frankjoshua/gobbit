#!/bin/bash

sudo cp ./rc.local /etc/rc.local
sudo cp ./start.sh /start.sh

#Setup i2c
echo i2c-bcm2708 >> /etc/modules
echo i2c-dev >> /etc/modules
echo dtparam=i2c1=on >> /boot/config.txt
echo dtparam=i2c_arm=on >> /boot/config.txt

sudo reboot

