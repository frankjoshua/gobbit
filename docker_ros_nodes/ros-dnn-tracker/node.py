#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time
from dnn_detect.msg import DetectedObjectArray

#Motor controller topic
motorPub = rospy.Publisher('/cmd_vel/computer', Twist, queue_size=1)
wagPub = rospy.Publisher('/wag_tail', Empty, queue_size=1)
lastTurn = 0
cmd = Twist()
wag = time.time()

def newImage(msg):
    global lastTurn, wag
    if msg.objects:
        #Sort by object size
        #sortedObjects = sorted(msg.objects, key=lambda object: object.y_min)
        #sortedObjects = sorted(msg.objects, key=lambda object: (object.x_max - object.x_min) + (object.y_max - object.y_min))
        sortedObjects = sorted(msg.objects, key=lambda object: -object.confidence)
        ##Sort by object type
        sortedObjects.sort(key=lambda object: object.class_name, reverse=True)
        #Find center of object
        screen_width = 640
        screen_center = screen_width / 2
        xmin = sortedObjects[0].x_min
        xmax = sortedObjects[0].x_max
        class_name = sortedObjects[0].class_name
        center = xmin + (xmax - xmin) / 2
        turn = (center - screen_center) / screen_width
        if class_name == 'person' and turn != lastTurn: 
            if time.time() - wag > 10:
                wag = time.time()
                wagPub.publish(Empty())
            #Turn towards object
            lastTurn = turn
            p = -turn * 1.25
            cmd.angular.z = p
            cmd.linear.y = 0
            cmd.linear.x = 0.75
            rospy.loginfo("Got dnn image %s", sortedObjects)    
        else:
            #Stop
            #cmd.angular.z = cmd.angular.z * 0.5
            cmd.angular.z = -(lastTurn * 0.5)
            #cmd.angular.z = 0
            cmd.linear.y = 0
            cmd.linear.x = 0
        motorPub.publish(cmd)

def listener():
    rospy.init_node('dnn_tracker', anonymous=False)
    #Listen for new detected objects
    rospy.Subscriber('/dnn_objects', DetectedObjectArray, newImage, queue_size=1)

    rospy.loginfo("DNN tracker Loaded...")

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
