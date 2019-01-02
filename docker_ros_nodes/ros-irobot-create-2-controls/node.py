#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import requests
import time

class Create2Controls:
    def __init__(self):
        rospy.init_node('create2_control_node', anonymous=False)
        rospy.Subscriber('/auto_dock', Empty, self.dock, queue_size=1)
        rospy.Subscriber('/auto_undock', Empty, self.undock, queue_size=1)
        self.launchNode = rospy.Publisher('/launch_node', String, queue_size=1)
        self.stopNode = rospy.Publisher('/stop_node', String, queue_size=1)
        self.motorNode = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.undockNode = rospy.Publisher('/undock', Empty, queue_size=1)
        self.dockNode = rospy.Publisher('/dock', Empty, queue_size=1)
        self.powerNode = rospy.Publisher('/main_motor', Float32, queue_size=1)
        self.docked = True

    def dock(self, msg):
        if self.docked:
            rospy.loginfo("Can not dock call /auto_undock")
            return
        self.docked = True
        rospy.loginfo("Docking...")
        #Disonnect Relay to Power
        requests.get("http://blynk-cloud.com/8034689be7e74fad9f77554e89659817/update/D13?value=0")
        rospy.loginfo("Turn off power relay")
        #Open BRC pin to allow sleep
        r = requests.get("http://blynk-cloud.com/8034689be7e74fad9f77554e89659817/update/D26?value=0")
        rospy.loginfo(r.reason)
        #Turn off power to brush motor
        rospy.loginfo("Turning off power.")
        self.powerNode.publish(0.0)
        time.sleep(1)
        #turn off laser
        rospy.loginfo("Turning off laser.")
        self.stopNode.publish("laser")
        #time.sleep(10)
        #Dock the robot
        rospy.loginfo("Calling /dock")
        self.dockNode.publish()
        time.sleep(20) #Give enough time to complete dock before try again
        rospy.loginfo("Calling /dock") #Call Second time because of brush motor warning on the first
        self.dockNode.publish()
        time.sleep(5)
        #Disonnect from create base
        rospy.loginfo("Disconnecting from create2 base")
        self.stopNode.publish("robot")

        
    
    def undock(self, msg):
        if not self.docked:
            rospy.loginfo("Can not undock call /auto_dock")
            return
        self.docked = False
        #Wake up create base by closing BRC pin
        rospy.loginfo("Waking Create 2 base with blynk...")
        r = requests.get("http://blynk-cloud.com/8034689be7e74fad9f77554e89659817/update/D26?value=1")
        rospy.loginfo(r.reason)
        time.sleep(3)
        #Connect to create base
        rospy.loginfo("Connecting to Create 2 base...")
        self.launchNode.publish("robot")
        time.sleep(5)
        rospy.wait_for_message('battery/charge_ratio', Float32, 30.0) #Wait for up to 30 seconds
        #Undock
        rospy.loginfo("Undocking...")
        self.undockNode.publish()
        time.sleep(2)
        rospy.loginfo("Backing up...")
        vel_msg = Twist()
        vel_msg.linear.x = -0.075
        # Publishing our vel_msg
        self.motorNode.publish(vel_msg)
        time.sleep(1)
        rospy.loginfo("Stopping.")
        vel_msg.linear.x = 0.0
        # Publishing our vel_msg
        self.motorNode.publish(vel_msg)
        #Turn on the laser
        rospy.loginfo("Turning on laser.")
        self.launchNode.publish("laser")
        time.sleep(5)
        #Turn on power to brush motor to recharge pi
        rospy.loginfo("Turning on power.")
        self.powerNode.publish(1.0)
        #Connect Relay to Power
        requests.get("http://blynk-cloud.com/8034689be7e74fad9f77554e89659817/update/D13?value=1")
        rospy.loginfo("Turn on power relay")
        
        
        

def listener():
    Create2Controls()

    rospy.loginfo("Create 2 control nodes loaded...")

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
