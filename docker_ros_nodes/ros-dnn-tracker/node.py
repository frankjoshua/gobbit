#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from dnn_detect.msg import DetectedObjectArray

#Motor controller topic
motorPub = rospy.Publisher('/pocketbot/cmd_vel', Twist, queue_size=1)
lastTurn = 0

def newImage(msg):
    global lastTurn
    if msg.objects:
        #Sort by object size
        sortedObjects = sorted(msg.objects, key=lambda object: (object.x_max - object.x_min) * (object.y_max - object.y_min))
        #Sort by object type
        sortedObjects.sort(key=lambda object: object.class_name)
        #Find center of object
        screen_width = 640
        screen_center = screen_width / 2
        xmin = sortedObjects[0].x_min
        xmax = sortedObjects[0].x_max
        center = xmin + (xmax - xmin) / 2
        turn = (center - screen_center) / screen_width
        if turn != lastTurn: 
            #Turn towards object
            lastTurn = turn
            rospy.logerr("Got dnn image %s", sortedObjects)
            cmd = Twist()
            cmd.angular.z = -turn
            motorPub.publish(cmd)
        else:
            #Stop
            motorPub.publish(Twist())
    

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
