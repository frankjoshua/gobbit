#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import time
import atexit

state = 0
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

def cleanup():
    pub.publish(Twist())

atexit.register(cleanup)

lastError = 0
def line(msg):
