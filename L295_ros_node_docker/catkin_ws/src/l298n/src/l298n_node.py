#!/usr/bin/env python
import rospy
import L298NHBridge as HBridge
from geometry_msgs.msg import Twist

def constrain(n, minn, maxn):
    return max(min(maxn, n), minn)

def callback(msg):
    dx = msg.linear.x
    dr = msg.angular.z
    w = 2.0
    right = 1.0 * dx + dr * w / 2
    left = 1.0 * dx - dr * w / 2
    HBridge.setMotorLeft(constrain(left,-1.0,1.0))
    HBridge.setMotorRight(constrain(right,-1.0,1.0))
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('l298n', anonymous=True)

    rospy.Subscriber("/pocketbot/cmd_vel", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
