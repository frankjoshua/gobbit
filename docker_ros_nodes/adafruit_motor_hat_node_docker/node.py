#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

import time
import atexit

def constrain(n, minn, maxn):
    return max(min(maxn, n), minn)

# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x60)

lfMotor = mh.getMotor(1)
rfMotor = mh.getMotor(2)
lrMotor = mh.getMotor(3)
rrMotor = mh.getMotor(4)

	
def driveMotors(msg):
    dx = msg.linear.x
    dr = msg.angular.z
    w = 1.0
    right = 1.0 * dx + dr * w / 2
    left = 1.0 * dx - dr * w / 2
    leftPower = constrain(left,-1.0,1.0)
    rightPower = constrain(right,-1.0,1.0)
    if(leftPower < 0):
        lfMotor.run(Adafruit_MotorHAT.BACKWARD)
        lrMotor.run(Adafruit_MotorHAT.BACKWARD)
    else:
        lfMotor.run(Adafruit_MotorHAT.FORWARD)
        lrMotor.run(Adafruit_MotorHAT.FORWARD)
    if(rightPower < 0):
        rfMotor.run(Adafruit_MotorHAT.BACKWARD)
        rrMotor.run(Adafruit_MotorHAT.BACKWARD)
    else:
        rfMotor.run(Adafruit_MotorHAT.FORWARD)
        rrMotor.run(Adafruit_MotorHAT.FORWARD)

    finalRightPower = int(abs(rightPower) * 255)
    finalLeftPower = int(abs(leftPower) * 255)

    lfMotor.setSpeed(finalLeftPower)
    lrMotor.setSpeed(finalLeftPower)
    rfMotor.setSpeed(finalRightPower)
    rrMotor.setSpeed(finalRightPower)	

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
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

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
