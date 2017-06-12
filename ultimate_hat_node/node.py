#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import smbus
import time
import atexit

bus = smbus.SMBus(1)
address = 0x20
leftMotor = 0xF0
rightMotor = 0xF1
servo = 0xF6

#Make sure motors are stopped
bus.write_byte_data(address, leftMotor, 0)
bus.write_byte_data(address, rightMotor, 0)
bus.write_byte_data(address, servo, 10000)

def constrain(n, minn, maxn):
    return max(min(maxn, n), minn)

def toTwosComp(value):
    result = value
    if value < 0:
        result = -result + 128
    return result

def driveMotors(msg):
    dx = msg.linear.x
    dr = msg.angular.z
    w = 1.0
    right = 1.0 * dx + dr * w / 2
    left = 1.0 * dx - dr * w / 2
    leftPower = constrain(left,-1.0,1.0)
    rightPower = constrain(right,-1.0,1.0)
    bus.write_byte_data(address, leftMotor, toTwosComp(int(leftPower * 100)))
    bus.write_byte_data(address, rightMotor, toTwosComp(int(rightPower * 100)))

lastControl = time.time()

def controlCallback(msg):
    global lastControl
    #Mark time of manual control
    lastControl = time.time()
    driveMotors(msg)

def callback(msg):
    global lastControl
    timeSinceLastManualControl = time.time() - lastControl
    #Make sure 3 seconds have passed since last manual control attempt
    if(timeSinceLastManualControl > 3):
        driveMotors(msg)


# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
    bus.write_byte_data(address, 0xF1, 0)
    bus.write_byte_data(address, 0xF0, 0)

def listener():
    atexit.register(turnOffMotors)
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('adafruit_motor_hat_node', anonymous=False)
    rospy.Subscriber("/cmd_vel", Twist, callback, queue_size=1)
    rospy.Subscriber("/pocketbot/cmd_vel", Twist, controlCallback, queue_size=1)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
