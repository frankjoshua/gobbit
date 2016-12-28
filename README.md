# Gobbit Robot with ROS and Raspberry Pi 3

Gobbit robot raspberry pi drivers. They work well for other robots also. This is designed to run with [HypriotOS](http://blog.hypriot.com/). It may work with Raspberian or other OSs if you install Docker first. 

Just clone the repo and run `cd gobbit && docker-compose up -d` This will download Docker images and run them. They should also auto restart after rebooting. After a minute a ROS master should be running at http://pirate@black-pearl.local:11311

If you have an Android phone the robot can be controled over wifi with an app called [PocketBot](https://play.google.com/store/apps/details?id=com.tesseractmobile.pocketbot).

[You can buy a Gobbit robot from Zagros Robotics](http://www.zagrosrobotics.com/shop/item.aspx?itemid=995)
<img src="http://pocketbot.io/wp-content/uploads/2016/08/nathalia_and_pocketbot_gobbit-1.jpg" alt="Gobbit Robot" width="517" height="385"/>
