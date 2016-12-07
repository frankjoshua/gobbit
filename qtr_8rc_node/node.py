#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import time
import atexit

try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!  This is probably because you need superuser privileges.  You can achieve this by using 'sudo' to run your script")

#GPIO Pins
ledPin = 18
pins = [23, 8, 7, 12, 20, 16, 21, 26]

#set up GPIO using BCM numbering
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)

def cleanup():
    GPIO.cleanup()

atexit.register(cleanup)

def readSensor(pin, highValue, lowValue):
    value = -1
    #Turn on LED
    GPIO.output(ledPin, GPIO.HIGH)
    #Set I/O pin to output
    GPIO.setup(pin, GPIO.OUT)
    #Set I/O pin to high
    GPIO.output(pin, GPIO.HIGH)
    #Wait 10 nano seconds at least
    time.sleep(0.0000001)
    #Set I/O to input
    GPIO.setup(pin, GPIO.IN)
    #Time capacitor discharge
    start = time.time()
    while(GPIO.input(pin)):
        value = value + 1
    done = time.time()
    #Turn off LED
    GPIO.output(ledPin, GPIO.LOW)
    value = (done - start) * 1000000
    if value > highValue:
        highValue = value
    if value < lowValue:
        lowValue = value
    return (value - lowValue) / (highValue - lowValue), highValue, lowValue

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    pubs = []
    for i in range(1, 9):
        pubs.append(rospy.Publisher('light' + str(i), Float32, queue_size=10))
    rospy.init_node('light_sensor_node', anonymous=True)

    # spin() simply keeps python from exiting until this node is stopped
    print "Line detection active."

    r = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        highValue = 0
        lowValue = 0
        for i in range(0, 8):
            value, highValue, lowValue = readSensor(pins[i], highValue, lowValue)
            pubs[i].publish(value)
        r.sleep()

if __name__ == '__main__':
    listener()
