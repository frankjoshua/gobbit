#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int32
import time
import atexit
import gc
from collections import deque

try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!  This is probably because you need superuser privileges.  You can achieve this by using 'sudo' to run your script")

#GPIO Pins
ledPin = 18
pins = [23, 8, 7, 12, 20, 16, 21, 26]
highValues = [0,0,0,0,0,0,0,0]
lowValues = [10000,10000,10000,10000,10000,10000,10000,10000]

#set up GPIO using BCM numbering
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)

#filters for the input values
len = 3
latestValues = [deque(maxlen=len), deque(maxlen=len), deque(maxlen=len), deque(maxlen=len), deque(maxlen=len), deque(maxlen=len), deque(maxlen=len), deque(maxlen=len), deque(maxlen=len)]
for i in range(0,9):
    for j in range(0,len):
        latestValues[i].append(0) #int all to zeros

def cleanup():
    GPIO.cleanup()

atexit.register(cleanup)

def readSensor():
    timeOut = 0.000001 * 3000
    values = [timeOut, timeOut, timeOut, timeOut, timeOut, timeOut, timeOut, timeOut]
    set = [False,False,False,False,False,False,False,False]
    #Turn on LED
    GPIO.output(ledPin, GPIO.HIGH)
    #Set I/O pin to output
    GPIO.setup(pins, GPIO.OUT)
    #Set I/O pin to high
    GPIO.output(pins, GPIO.HIGH)
    #Wait 10 microseconds at least
    time.sleep(0.00001)
    gc.disable()
    #Set I/O to input
    GPIO.setup(pins, GPIO.IN)
    #Time capacitor discharge
    start = time.time()
    #loop until timeout
    while(time.time() - start < timeOut):
        #If this takes too long the sensor is probaly off the line
        for i in range(0, 8):
            if(set[i] == False and GPIO.input(pins[i]) == False):
                set[i] = True
                values[i] = time.time() - start
    gc.enable()
    #Turn off LED
    GPIO.output(ledPin, GPIO.LOW)
    #Convert values to a percent of low to high values
    highValue = 0
    lowValue = 100000000
    for i in range(0, 8):
        #apply median filter
        latestValues[i].append(values[i] * 1000000)
        sortedList = sorted(latestValues[i])
        values[i] = sortedList[len / 2]
        #values[i] = values[i] * 1000000
        #Adjust high and lows to auto calibrate
        if values[i] > highValue:
            highValue = values[i]
        if values[i] < lowValue:
            lowValue = values[i]
    #watch for divide by 0
    highValue = 3000
    #lowValue = 0
    denominator = highValue - lowValue    
    if(denominator == 0):
        denominator = 1
    #for i in range(0, 8):
    #    values[i] = (values[i] - lowValue) / denominator
    print(str(lowValue))
    print(values)
    print(str(highValue))        
    return values

def detectIntersection(values, first, last):
    #total the values
    total = 0
    for i in range(first, last + 1):
        total += values[i]
    #get average value
    average = total / (last - first)
    #all four values should be equal to the average within a tolerance
    for i in range(first, last + 1):
        diff = abs(average - values[i])
        if(diff > 0.2):
            return False
    return True

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

    r = rospy.Rate(150) # 150hz  
    while not rospy.is_shutdown():
        lineValue = 0
        totalValue = 0    
        lineValues = readSensor()
        #print(lineValues)
        onLine = False
        #publish filtered value
        for i in range(0, 8):
            if(lineValues[i] > 1000):
                onLine = True
            if(lineValues[i] > 500):
                lineValue += i * 1000 * lineValues[i]
                totalValue += lineValues[i]
            pubs[i].publish(lineValues[i])

        #publish line value
        if(totalValue > 0):
            if(onLine == False):
                if(latestValues[8][len - 1] > 3500):
                    linePub.publish(7000)
                else:
                    linePub.publish(0)
            else:
                latestValues[8].append(int(lineValue / totalValue))
                sortedList = sorted(latestValues[8])
                linePub.publish(sortedList[len / 2])
        #check for intersection
        threshHold = 0.2
        #if(lineValues[2] > threshHold and lineValues[3] > threshHold and lineValues[4] > threshHold and lineValues[5] > threshHold):
        if(detectIntersection(lineValues, 3, 6)):
            #line found send a 1
            intersectionPub.publish(1)
        else:
            intersectionPub.publish(0)

#        print str(highValue) + " " + str(lineValues[3]) + " " + str(lowValue)
        r.sleep()

if __name__ == '__main__':
    listener()
