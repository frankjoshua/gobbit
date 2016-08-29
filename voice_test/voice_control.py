#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

pub = rospy.Publisher('/pocketbot/cmd_vel', Twist, queue_size=10)

def callback(msg):
    twist = Twist()

    if 'forward' in msg.data:
        twist.linear.x = 0.25
    if 'right' in msg.data:
        twist.angular.z = -1
    if 'left' in msg.data:
        twist.angular.z = 1
    if 'back' in msg.data:
        twist.linear.x = -0.25
    if 'stop' in msg.data:
        twist.linear.x = 0
        twist.angular.z = 0

    pub.publish(twist)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('voice_control', anonymous=True)

    rospy.Subscriber("/pocketbot/speech", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
