#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int32
import time
import atexit
from collections import deque

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

#filters for the input values
len = 4
latestValues = [deque(maxlen=len), deque(maxlen=len), deque(maxlen=len), deque(maxlen=len), deque(maxlen=len), deque(maxlen=len), deque(maxlen=len), deque(maxlen=len)]
for i in range(0,8):
    for j in range(0,len):
        latestValues[i].append(0) #int all to zeros

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
    #Wait 10 microseconds at least
    time.sleep(0.00001)
    #Set I/O to input
    GPIO.setup(pin, GPIO.IN)
    #Time capacitor discharge
    start = time.time()
    timeOut = 0.000001 * 2500
    while(GPIO.input(pin)):
        done = time.time()
        #If this takes too long the sensor is probaly off the line
        if(done - start > timeOut):
            break
    done = time.time()
    #print(str(done))
    #print(str(done - start))
    #Turn off LED
    GPIO.output(ledPin, GPIO.LOW)
    value = (done - start) * 1000000
    if value * 0.7 > highValue:
        highValue = value
    if value > 0.0000001 and value * 1.3 < lowValue:
        lowValue = value

    #watch for divide by 0
    denominator = highValue - lowValue    
    if(denominator == 0):
        denominator = 1
        
    return (value - lowValue) / denominator, highValue, lowValue

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    pubs = []
    for i in range(1, 9):
        pubs.append(rospy.Publisher('light' + str(i), Float32, queue_size=1))
    linePub = rospy.Publisher('line', Int32, queue_size=1)
    intersectionPub = rospy.Publisher('intersection', Int32, queue_size=1)
    rospy.init_node('light_sensor_node', anonymous=True)

    # spin() simply keeps python from exiting until this node is stopped
    print "Line detection active."

    r = rospy.Rate(1000) # 1khz  
    lowValue = 1000
    highValue = 0
    while not rospy.is_shutdown():
        lineValue = 0
        totalValue = 0
        lineValues = []
        for i in range(0, 8):
            global lineValue, lowValue, highValue, lineValues
            value, highValue, lowValue = readSensor(pins[i], highValue, lowValue)
            #apply median filter
            latestValues[i].append(value)
            sortedList = sorted(latestValues[i])
            filteredValue = (sortedList[len / 2] + sortedList[len / 2 + 1]) / 2            
            lineValue += i * 1000 * filteredValue
            totalValue += filteredValue
            #save original values
            lineValues.append(value)
            #publish filtered value
            pubs[i].publish(filteredValue)

        #publish line value
        linePub.publish(int(lineValue / totalValue))
        #check for intersection
        threshHold = 0.4
        if(lineValues[2] > threshHold and lineValues[3] > threshHold and lineValues[4] > threshHold and lineValues[5] > threshHold):
            #line found send a 1
            intersectionPub.publish(1)
        else:
            intersectionPub.publish(0)

        print str(highValue) + " " + str(lineValues[3])  + " " + str(lowValue)
        r.sleep()

if __name__ == '__main__':
    listener()
