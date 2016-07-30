#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include "wiringPi.h"

void driveL298n(const geometry_msgs::Twist& msg){

}

init main(int argc, char** argv){
    ros:init(argc, argv, "l298n");
    ROS_INFO("Stareted l298n node");

    ros::NodeHandle n;
    ros:Subscriber sub = n.subscribe("/cmd_vel",10,driveL298n);
    ros::spin();

    return 0;
}

