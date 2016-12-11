#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import time
import atexit

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def cleanup():
    pub.publish(Twist())

atexit.register(cleanup)

def callback(msg):
#    print str(msg)
    cmd = Twist()
    cmd.linear.x = 0.2
#    print(str(msg.data - 3500))
    error = msg.data - 3500
    if(abs(error) > 150):
        cmd.angular.z = error * 0.000075
    print(str(cmd.angular.z))
#    print str(cmd)
    pub.publish(cmd) 


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('line_follower', anonymous=True)
    rospy.Subscriber('/line', Int32, callback)

    print "Line follower active..."

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
