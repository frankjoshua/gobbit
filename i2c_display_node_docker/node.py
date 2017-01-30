#!/usr/bin/env python
import rospy
import time
import atexit
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

# Raspberry Pi pin configuration:
RST = 24
# 128x32 display with hardware I2C:
disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)
# Initialize library.
disp.begin()
# Clear display.
disp.clear()
disp.display()
# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = disp.width
height = disp.height
image = Image.new('1', (width, height))
# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)
# Load default font.
font = ImageFont.load_default()
dx = 0
dr = 0
line = 0

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

def cmd_callback(msg):
    global dx, dr
    dx = msg.linear.x
    dr = msg.angular.z

def line_callback(msg):
    global line
    line = msg.data
    updateDisplay()

def updateDisplay():
    # Draw a black filled box to clear the image.
    draw.rectangle((0,0,width,height), outline=0, fill=0)
    draw.text((0, 10), "X: " + str(dx) + " Z: " + str(dr), font=font, fill=255)
    draw.text((0, 20), "Line: " + str(line), font=font, fill=255)
    linePos = translate(line, 7000, 0, 0, 128);
    draw.text((linePos, 0), "-", font=font, fill=255)
    disp.image(image)
    # Display image.
    disp.image(image)
    disp.display()

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('olde_node', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, cmd_callback, queue_size = 1)
    rospy.Subscriber("/line/filtered", Int32, line_callback, queue_size = 1)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
