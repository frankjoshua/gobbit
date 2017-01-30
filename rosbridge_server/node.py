#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import time
import atexit

state = 0
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

def cleanup():
    pub.publish(Twist())

atexit.register(cleanup)

lastError = 0
def line(msg):
    global state
    #create new Twist message
    cmd = Twist()
    #react based on intersection
    if(state == 0):
        global lastError
        cmd.linear.x = 0.25
        error = msg.data - 3500
        if(abs(error) > 400):
            #print(str(lastError))
            kp = 0.000035
            kd = 0.000015
            cmd.angular.z = kp * error + kd * (error - lastError)
            cmd.linear.x = max(cmd.linear.x - abs(cmd.angular.z), 0.01)
        lastError = error
    #Publish the message
    pub.publish(cmd) 

def intersection(msg):
    global state
    #update state
#    state = msg.data
#    print(str(msg.data))

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('line_follower', anonymous=True)
    rospy.Subscriber('/line/filtered', Int32, line, queue_size=1)
    rospy.Subscriber('/line/intersection', Int32, intersection)

    print "Line follower active..."

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
