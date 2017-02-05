#!/usr/bin/env python
import rospy
import smbus
import time
import atexit
from std_msgs.msg import Int32

bus = smbus.SMBus(1)
address = 0x10
line = 0
intersection = 0
outOfBounds = 0

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

def readLine():
    global line, intersection, outOfBounds
    value = bus.read_byte_data(address, 1)
    line = translate(value & 63, 1, 63, 0, 7000)
    intersection = (value & 64) / 64
    outOfBounds = (value & 128) / 128

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('linedog_node', anonymous=False)
    pub = rospy.Publisher('line/filtered', Int32, queue_size=1)
    pubIntersection = rospy.Publisher('line/intersection', Int32, queue_size=1)
    pubBounds = rospy.Publisher('line/bounds', Int32, queue_size=1)
    r = rospy.Rate(150) # 150hz
    while not rospy.is_shutdown():
        global line
        readLine()
        pub.publish(line)
        pubIntersection.publish(intersection)
        pubBounds.publish(outOfBounds)
        r.sleep()    

if __name__ == '__main__':
    listener()
