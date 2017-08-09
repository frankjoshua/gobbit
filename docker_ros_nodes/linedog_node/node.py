#!/usr/bin/env python
import rospy
import smbus
import time
import threading
import atexit
from std_msgs.msg import Int32

bus = smbus.SMBus(1)
address = 0x10
line = 0
intersection = 0
bounds = 0

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
    try:
        #bus.write_byte(address, 0x01)
        #value = bus.read_byte_data(address, 1)
        values = bus.read_i2c_block_data(address, 1, 2)
        value = values[0]
        #print str(values)
        #print str(value)
        
        #value = bus.read_byte(address)
        #print "b " + str(bus.read_byte(address))
        #values = bus.read_block_data(address, 0x2)
        #print "b " + str(values)
        #line = translate(value & 63, 63, 1, 0, 7000)
        line = value & 63
        #intersection = (value & 64) / 64
        #bounds = (value & 128) / 128
    except Exception as e:
        line = -1
        print e

def listener():
    #Start calibration
    print "Starting Calibration"
    bus.write_byte(address, 0x2F)
    time.sleep(5)
    
    rospy.init_node('linedog_node', anonymous=False)
    pub = rospy.Publisher('line/filtered', Int32, queue_size=1)
    pubIntersection = rospy.Publisher('line/intersection', Int32, queue_size=1)
    pubBounds = rospy.Publisher('line/bounds', Int32, queue_size=1)
    r = rospy.Rate(150) # 60hz This is about as fast as the line changes at this resolution
    lastLine = line
    lastIntersection = intersection
    lastBounds = bounds
    while not rospy.is_shutdown():
        readLine()
        if(line != lastLine):
            lastLine = line
            pub.publish(line)
        if(intersection != lastIntersection):
            lastIntersection = intersection
            pubIntersection.publish(intersection)
        if(bounds != lastBounds):
            lastBounds = bounds
            pubBounds.publish(bounds)
        r.sleep()    

if __name__ == '__main__':
    listener()
