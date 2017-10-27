#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
import cv2, cv_bridge, numpy
import math
from cv_bridge import CvBridge, CvBridgeError
import atexit

class OpenCVLineDetector:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber("/image_raw/compressed", CompressedImage, self.callback, queue_size=1)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        atexit.register(self.cleanup)

    def cleanup(self):
        #Stop the robot
        self.pub.publish(Twist())

    def bottom(self, hsv):
        #filter out black
        lower = numpy.array([0,0,0])
        upper = numpy.array([80,80,80])
        mask = cv2.inRange(hsv, lower, upper)
        #limit search to bottom of image
        h, w, d = hsv.shape
        search_top = 3*h/4
        search_bot = search_top + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        #find center of mask
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            err = cx - w/2
            return cx, cy, err
        else:
            return 0,0,0

    def top(self, hsv):
        #filter out black
        lower = numpy.array([0,0,0])
        upper = numpy.array([80,80,80])
        mask = cv2.inRange(hsv, lower, upper)
        #limit search to bottom of image
        h, w, d = hsv.shape
        search_top = 2*h/4
        search_bot = search_top + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        #find center of mask
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            err = cx - w/2
            return cx, cy, err
        else:
            return 0,0,0

    def callback(self, msg):
        try:
            #### direct conversion to CV2 ####
            np_arr = numpy.fromstring(msg.data, numpy.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            #cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
          print(e)

        #convert to hsv
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        cxB, cyB, errB = self.bottom(hsv)
        cxT, cyT, errT = self.top(hsv)
        cv2.circle(hsv, (cxT, cyT), 15, (255,0,0), -1)
        cv2.circle(hsv, (cxB, cyB), 15, (0,0,255), -1)
        slope = numpy.arctan2(cyT - cyB, cxT - cxB)

        #Turn so the line is vertical, slope == -PI/2
        slopeErr = -math.pi/2 - slope
        print(str.format('{0:.6f}', slopeErr))
        self.cmd.angular.z = float(slopeErr) / 2 #2
        self.cmd.linear.y = float(errB) / 250 #250

        #Display results
        cv2.imshow("window", hsv)
        cv2.waitKey(3)
        #Send command to robot
        #self.cmd.linear.x = max(0.01, 0.2 - (abs(self.cmd.linear.y) + abs(self.cmd.angular.z)) * 2)#0.25
        self.cmd.linear.x = 0.0 #0.1
        self.pub.publish(self.cmd)



def listener():
    rospy.init_node('opencv_line_detector', anonymous=False)
    openCVLineDetector = OpenCVLineDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    listener()
