#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class OdomPublisher:

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = rospy.Time.now()
        self.odom_pub = rospy.Publisher("odom_encoder", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()

    def calculateDelta(self, dt, vx, vy, vth, th):
        delta_x = (vx * cos(th) - vy * sin(th)) * dt
        delta_y = (vx * sin(th) + vy * cos(th)) * dt
        delta_th = vth * dt
        return delta_x, delta_y, delta_th

    def run(self, twist):
        
        vx = twist.linear.x
        vy = -twist.linear.y
        vth = -twist.angular.z

        current_time = rospy.Time.now()

        # compute odometry in a typical way given the velocities of the robot
        dt = (current_time - self.last_time).to_sec()
        delta_x, delta_y, delta_th = self.calculateDelta(dt, vx, vy, vth, self.th)

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            current_time,
            "base_footprint",
            "odom/encoder"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom/encoder"

        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_footprint"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        self.odom_pub.publish(odom)

        self.last_time = current_time


def listener():
    rospy.init_node('odometry_publisher')
    odomPublisher = OdomPublisher()
    callback = lambda msg: odomPublisher.run(msg) 
    rospy.Subscriber("encoder/velocity", Twist, callback, queue_size=50)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    listener()
