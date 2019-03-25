#!/usr/bin/env python
import rospy
import time
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
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback, queue_size=1)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.imageOut = rospy.Publisher('/line/image_raw', Image, queue_size=1)
        self.cmd = Twist()
        # P=0.4
        # I=0.002
        # D=0.03
        P=0.8
        I=0.0
        D=0 #0.1
        self.pid = PID(P, I, D)
        self.pid.SetPoint = 0.0
        P=0.005
        I=0.001
        D=0 #0.0015
        self.pidStrafe = PID(P, I, D)
        self.pidStrafe.SetPoint = 0.0
        atexit.register(self.cleanup)

    def cleanup(self):
        #Stop the robot
        self.pub.publish(Twist())

    def bottom(self, hsv, mask):
        #limit search to bottom of image
        h, w, d = hsv.shape
        search_top = 5*h/6
        search_bot = h
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        # edge = w/4
        # mask[0:h, 0:edge] = 0
        # mask[0:h, w-edge:w] = 0
        #find center of mask
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            err = cx - w/2
            return cx, cy, err

        #Safest to report center
        return -1, -1, -1

    def top(self, hsv, mask):
        #limit search to bottom of image
        h, w, d = hsv.shape
        search_top = 1*h/4
        search_bot = 3*h/4
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        #find center of mask
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            err = cx - w/2
            return cx, cy, err
        #Return error
        return -1, -1, -1

    def callback(self, msg):
        try:
            #### direct conversion to CV2 ####
            np_arr = numpy.fromstring(msg.data, numpy.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            #cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
          print(e)
          return

        #convert to hsv
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        #filter out black
        lower = numpy.array([0,190,0])
        upper = numpy.array([180, 255, 90])
        mask = cv2.inRange(hsv, lower, upper)
        h, w, d = hsv.shape
        # mask[0:h, 0:0] = 0
        # mask[0:h, w-0:w] = 0
        #Create output image for displaying data to user
        output = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        output = cv2.bitwise_not(output)
        #Find center line of Top and Botton
        #cxT, cyT, errT = self.top(hsv, mask.copy())
        width, cxT, cyT, contour = self.contours(mask.copy(), output)
        cxB, cyB, errB = self.bottom(hsv, mask.copy())
        #Display results
        cv2.circle(output, (cxT, cyT), 10, (255,0,0), -1)
        cv2.circle(output, (cxB, cyB), 10, (0,0,255), -1)
        cv2.imshow("window", numpy.hstack([hsv,output]))
        cv2.waitKey(1)
        #Check for error state
        if cxT == -1 or cxB == -1:
            self.cmd.angular.z = 0
            self.cmd.linear.y = 0
            self.cmd.linear.x = 0
            if cxT > -1:
                self.cmd.linear.x = 0.05
            if cxB > -1:
                if cxB > w/2:
                    self.cmd.linear.y = 0.05
                else:
                    self.cmd.linear.y = -0.05
            self.pub.publish(self.cmd)
            return

        slope = numpy.arctan2(cyT - cyB, cxT - cxB)

        #Turn so the line is vertical, slope == -PI/2
        slopeErr = -math.pi/2 - slope
        #Send command to robot
        self.setCommand(zError = slopeErr, yError = errB, width = width)
        self.pub.publish(self.cmd)

        #imageToPub = self.bridge.cv2_to_compressed_imgmsg(numpy.array(output))
        imageToPub = self.bridge.cv2_to_imgmsg(output, encoding="bgr8")
        self.imageOut.publish(imageToPub)

    def setCommand(self, zError, yError, width):
        #Update PID
        self.pid.update( feedback_value = -zError )
        self.pidStrafe.update( feedback_value = -yError )
        #Update command
        if width > 110:
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = 0
            self.cmd.linear.y = 0
        else:
            self.cmd.linear.y = self.pidStrafe.output
            if abs(self.pidStrafe.output) > 0.1:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
            else:
                self.cmd.linear.x = 0.2
                self.cmd.angular.z = self.pid.output


    def contours(self, mask, output):
        h, w, d = output.shape
        search_top = 3*h/6
        search_bot = 4*h/6
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        ret, thresh = cv2.threshold(mask, 127, 255, 0)
        im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) != 0:
            # draw in blue the contours that were founded
            cv2.drawContours(output, contours, -1, 255, 3)

            # find the biggest area
            c = max(contours, key=cv2.contourArea)

            x, y, w, h = cv2.boundingRect(c)
            # draw the largest contour (in green)
            cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.drawContours(output, c, -1, (0, 255, 0), 3)
            M = cv2.moments(c)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(output, (cx, cy), 15, (0, 255, 0), -1)
                return w, cx, cy, c
        #Return error
        return -1, -1, -1, None

class PID:
    """PID Controller
    """

    def __init__(self, P=0.2, I=0.0, D=0.0):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value):
        """Calculates PID value for given reference feedback

        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}

        .. figure:: images/pid_1.png
           :align:   center

           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)

        """
        error = self.SetPoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time

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
