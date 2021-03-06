# Gobbit Robot with ROS and Raspberry Pi 3

##IDE
There is a web based editor that can be used to edit code directly on the robot.
Just run ./ace_ide/start.sh
Then open the ide from http://ROBOTS_IP_ADDRESS:8181

## Quick Start
1. Install [HypriotOS](http://blog.hypriot.com/) onto a **32GB** or larger micro SD<br>
2. git clone https://github.com/frankjoshua/gobbit.git<br>
3. cd gobbit && ./start.sh<br>
4. Wait.... The PI will download many gigs of data it's going to take a while

## Hypriot OS has an issue
Version 1.5.0 of Hypriot has a bug in Docker. Use the following command to fix it. https://github.com/docker/compose/issues/4972
```
sudo apt-get remove python-pip
sudo easy_install pip
sudo pip install -U docker==2.4.2
```

Gobbit robot raspberry pi drivers. They work well for other robots also. This is designed to run with [HypriotOS](http://blog.hypriot.com/). It may work with Raspberian or other OSs if you install Docker first. 

Just clone the repo and run `cd gobbit && ./start.sh` This will download Docker images and run them. They should also auto restart after rebooting. After a minute a ROS master should be running at http://pirate@black-pearl.local:11311

If you have an Android phone the robot can be controled over wifi with an app called [PocketBot](https://play.google.com/store/apps/details?id=com.tesseractmobile.pocketbot).

[You can buy a Gobbit robot from Zagros Robotics](http://www.zagrosrobotics.com/shop/item.aspx?itemid=995)
<br>
<img src="http://pocketbot.io/wp-content/uploads/2016/08/nathalia_and_pocketbot_gobbit-1.jpg" alt="Gobbit Robot" width="517" height="385"/>
