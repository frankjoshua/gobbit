#!/bin/bash
COLOR='\033[0;36m'
NO_COLOR='\033[0;0m'

#Must be run as root
[[ `id -u` -eq 0 ]] || { echo -e "${COLOR}Must be root to run script${NO_COLOR}"; exit 1; }

#execute commands in script directory 
cd "${0%/*}"

#Setup i2c
grep -q i2c-dev /etc/modules
if [ $? -eq 0 ]; then
	echo "Don't run this twice"
	exit 1
fi
sudo echo i2c-bcm2708 >> /etc/modules
sudo echo i2c-dev >> /etc/modules
sudo echo dtparam=i2c1=on >> /boot/config.txt
sudo echo dtparam=i2c_arm=on >> /boot/config.txt

#run start.sh once so docker image will download
sudo ./start.sh -d

echo "Rebooting....."
sudo reboot
