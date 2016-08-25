#!/bin/bash

#execute commands in script directory 
cd "${0%/*}"

#replace rc.local (probally not the best way) to run start.sh on boot
sudo cp ./rc.local /etc/rc.local
#copy start.sh to / (maybe this should go in /usr/bin instead but I wanted it to be visible)
sudo cp ./start.sh /start.sh

#Setup i2c
sudo echo i2c-bcm2708 >> /etc/modules
sudo echo i2c-dev >> /etc/modules
sudo echo dtparam=i2c1=on >> /boot/config.txt
sudo echo dtparam=i2c_arm=on >> /boot/config.txt

#run start.sh once so docker image will download
sudo ./start.sh

echo "Rebooting....."
sudo reboot
