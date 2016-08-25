#!/bin/bash

cd "${0%/*}"
sudo cp ./rc.local /etc/rc.local
sudo cp ./start.sh /start.sh

#Setup i2c
sudo echo i2c-bcm2708 >> /etc/modules
sudo echo i2c-dev >> /etc/modules
sudo echo dtparam=i2c1=on >> /boot/config.txt
sudo echo dtparam=i2c_arm=on >> /boot/config.txt

echo "Rebooting....."
sudo reboot
