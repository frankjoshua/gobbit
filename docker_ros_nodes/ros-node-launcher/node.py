#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import docker
import os

class NodeLauncher:
    def __init__(self):
        self.activeContainers = {}
        rospy.init_node('node_launcher', anonymous=False)
        #Listen for new detected objects
        rospy.Subscriber('/launch_node', String, self.launchNode, queue_size=1)
        rospy.Subscriber('/stop_node', String, self.stopNode, queue_size=1)

    def createRobotNode(self, client):
        client.images.pull("frankjoshua/ros-irobot-create-2:armhf")
        return client.containers.run(
                "frankjoshua/ros-irobot-create-2:armhf",
                environment={"ROS_MASTER_URI": os.environ['ROS_MASTER_URI'], "ROS_IP": os.environ['ROS_IP']},
                devices=["/dev/create2:/dev/ttyUSB0"],
                network="host",
                detach=True)
        
    def createLaserNode(self, client):
        client.images.pull("frankjoshua/ros-ydlidar-x4:armhf")
        return client.containers.run(
                "frankjoshua/ros-ydlidar-x4:armhf",
                environment={"ROS_MASTER_URI": os.environ['ROS_MASTER_URI'], "ROS_IP": os.environ['ROS_IP']},
                devices=["/dev/ydlidar:/dev/ydlidar"],
                network="host",
                detach=True)
                
    def launchNode(self, msg):
        rospy.loginfo(msg.data)
        client = docker.from_env()
        #Launch the container
        if msg.data == "robot":
            result = self.createRobotNode(client)
        elif msg.data == "laser":
            result = self.createLaserNode(client)
        #Save the container        
        self.activeContainers[msg.data] = result
        rospy.loginfo(result)

    def stopNode(self, msg):
        self.activeContainers[msg.data].stop()
        

def listener():
    NodeLauncher()

    rospy.loginfo("Node launcher Loaded...")

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
