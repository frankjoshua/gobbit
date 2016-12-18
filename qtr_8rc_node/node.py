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
len = 15
latestValues = [deque(maxlen=len), deque(maxlen=len), deque(maxlen=len), deque(maxlen=len), deque(maxlen=len), deque(maxlen=len), deque(maxlen=len), deque(maxlen=len)]
for i in range(0,8):
    for j in range(0,len):
        latestValues[i].append(0) #int all to zeros

def cleanup():
    GPIO.cleanup()

atexit.register(cleanup)

def readSensor():
    timeOut = 0.000001 * 2500
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
    for i in range(0, 8):
        values[i] = values[i] * 1000000
        #Adjust high and lows to auto calibrate
        if values[i] < timeOut * 1000000 and values[i] > highValues[i]:
            highValues[i] = values[i]
            #print(highValues)
        if values[i] > 0 and values[i] < lowValues[i]:
            lowValues[i] = values[i]
        #watch for divide by 0
        denominator = highValues[i] - lowValues[i]    
        if(denominator == 0):
            denominator = 1
        #apply median filter
        if(values[i] < timeOut * 1000000):
            latestValues[i].append(values[i])
        sortedList = sorted(latestValues[i])
        #values[i] = (sortedList[len / 2] + sortedList[len / 2 + 1]) / 2            
        print(sortedList)
        values[i] = sortedList[len / 2]
        values[i] = (values[i] - lowValues[i]) / denominator
        
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

    r = rospy.Rate(100) # 200hz  
    while not rospy.is_shutdown():
        lineValue = 0
        totalValue = 0    
        lineValues = readSensor()
        #lineValue += i * 1000 * filteredValue
        #totalValue += filteredValue
        #save original values
        #lineValues.append(filteredValue)
        #publish filtered value
        for i in range(0, 8):
            lineValue += i * 1000 * lineValues[i]
            totalValue += lineValues[i]
            pubs[i].publish(lineValues[i])

        #publish line value
        if(totalValue > 0):
            linePub.publish(int(lineValue / totalValue))
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
