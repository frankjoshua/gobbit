#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from Adafruit_PWM_Servo_Driver import PWM
import time
import atexit

pwm = PWM(0x40)
pwm.setPWMFreq(60)                        # Set frequency to 60 Hz

servoMin = 150  # Min pulse length out of 4096
servoMax = 600  # Max pulse length out of 4096

pulse = servoMin
lastPulse = servoMin

def constrain(n, minn, maxn):
    return max(min(maxn, n), minn)

def setServoPulse(channel, pulse):
    pulseLength = 1000000                   # 1,000,000 us per second
    pulseLength /= 60                       # 60 Hz
    pulseLength /= 4096                     # 12 bits of resolution
    pulse *= 1000
    pulse /= pulseLength

def callback(msg):
    global pulse
    pan = msg.angular.y
    tilt = msg.angular.x
    pulse = int(servoMin + (servoMax - servoMin) * ((pan + 1) / 2))

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('adafruit_servo_hat_node', anonymous=False)

    rospy.Subscriber("/pocketbot/cmd_vel", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
       global lastPulse, pulse
       if(lastPulse != pulse):
           pwm.setPWM(10, 0, pulse)
           lastPulse = pulse       
       #setServoPulse(10, pulse)
       rate.sleep()

if __name__ == '__main__':
    listener()
