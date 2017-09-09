#!/usr/bin/env python
import rospy
from math import radians, cos, sin, asin, sqrt, atan2, degrees
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
import math
import time
import atexit
from pid_controller.pid import PID

#Current destination
destination = NavSatFix()
#Last location
lastFix = NavSatFix()
#Current Heading
currentHeading = 0
#PID controller for stearing
pid = PID(p=0.002, i=0.0, d=0.0)

#Motor controller topic
motorPub = rospy.Publisher('/pocketbot/cmd_vel', Twist, queue_size=1)
directionPub = rospy.Publisher('/direction', Float64, queue_size=1)
distancePub = rospy.Publisher('/distance', Float64, queue_size=1)

def cleanup():
    #Stop the motors
    motorPub.publish(Twist())

atexit.register(cleanup)

def haversine(lon1, lat1, lon2, lat2):
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles
    return c * r
    
def getDistance(navsat, destination):
    #return distance between 2 points
    lat1 = navsat.latitude
    lon1 = navsat.longitude
    lat2 = destination.latitude
    lon2 = destination.longitude

    return haversine(lon1, lat1, lon2, lat2)
    
def getDirection(current, destination):
    #return direction for current location to destination
    startLat = radians(destination.latitude)
    startLong = radians(destination.longitude)
    endLat = radians(current.latitude)
    endLong = radians(current.longitude)
    dLong = endLong - startLong

    dPhi = math.log(math.tan(endLat/2.0+math.pi/4.0)/math.tan(startLat/2.0+math.pi/4.0))
    if abs(dLong) > math.pi:
        if dLong > 0.0:
            dLong = -(2.0 * math.pi - dLong)
        else:
            dLong = (2.0 * math.pi + dLong)

    bearing = (math.degrees(math.atan2(dLong, dPhi)) + 360.0) % 360.0;
    return bearing

def driveRobot(kmToWaypoint, error, setPoint, point):
    #Send command to robot
    drive = Twist()
    if kmToWaypoint > error:
        #Drive forward
        drive.linear.x = error
    else:
        #Stop
        drive.linear.x = 0.0
    
    pid.target = setPoint    
    output = pid(feedback=point)
    drive.angular.y = setPoint
    drive.angular.x = point
    drive.angular.z = output
    #Publish Twist command    
    motorPub.publish(drive)

def gps(navsat):
    global lastFix
    #Save last location
    lastFix = navsat

def waypoint(navsat):
    #Save destination
    global destination
    destination = navsat
    
def heading(newHeading):
    #save heading
    global currentHeading
    currentHeading = newHeading.data
    
def publishUpdates():
    global lastFix, destination, currentHeading
    #Get distance to destination
    distance = getDistance(lastFix, destination)
    distancePub.publish(distance)
    #Get direction of destination
    direction = getDirection(lastFix, destination)
    directionPub.publish(direction)
    #Drive robot
    errorInKilometers = lastFix.position_covariance[0] / 1000
    driveRobot(distance, errorInKilometers, direction, currentHeading)

def listener():
    rospy.init_node('gps_nav', anonymous=False)
    #Listen for gps update
    rospy.Subscriber('/pocketbot/fix', NavSatFix, gps, queue_size=1)
    #Listen for new waypoints
    rospy.Subscriber('/pocketbot/waypoint', NavSatFix, waypoint, queue_size=1)
    #Listen for heading updates
    rospy.Subscriber('/pocketbot/heading', Float32, heading, queue_size=1)
    
    print "GPS Nav Loaded..."
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        publishUpdates()
        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
