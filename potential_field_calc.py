#!/usr/bin/env python3

# Farhan Rozaidi, nikahman@oregonstate.edu
#
# This script uses laserscan data to develop a potential field vector
# NOTE: The following script uses solutions from Bill Smart's ROB 599 ROS Class
# Notably, the PointCloud stuff is inspired by hw1 solution of ROB 599


# Import necessary ROS stuff
import rospy
import sys

# Get all the right datatypes to do the LIDAR processing
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from hw1.msg import FieldVector

# Math functions for neat calculations
from math import sin, cos, atan2
from numpy import tanh, sqrt, linspace, inf, isnan

# Import tf, used for transformations
import tf

class PotentialField:
    def __init__(self, robot_width, robot_frame):

        # LIDAR in the center, so we use radius to do comparisons
        self.robot_radius = robot_width / 2.0

        # Robot frame variable used to transform to the base frame of the robot
        self.robot_frame = robot_frame

        # Subscribe to raw laserscan data, publish to a filtered scan topic
        self.sub = rospy.Subscriber(
            'raw_scan', LaserScan, self.field_calc, queue_size=1)
        self.pub = rospy.Publisher('field_vector', FieldVector, queue_size=10)

        # Set up the listenter
        self.listener = tf.TransformListener()

        # Set up PointCloud variable for fast computations later
        self.cloud = PointCloud()

    def field_calc(self, msg):

        # Use the header of the laserscan data as the header for PointCloud object
        self.cloud.header = msg.header

        # Robust means of getting the min and max angle, robot agnostic
        angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

        # For each ray of LIDAR data, convert it to a Point in the PointCloud
        self.cloud.points = [Point32(r * cos(theta), r * sin(theta), 0) for r, theta in zip(msg.ranges, angles)]


        try:

            # Transform from LIDAR frame to the base frame
            self.cloud = self.listener.transformPointCloud(self.robot_frame, self.cloud)

        # Sometimes transformation is too quick, or base frame doesnt exist, so we ignore
        except (tf.ExtrapolationException, tf.LookupException):
            
            # Make the node skip the current tick, essentially
            rospy.Rate(10).sleep()

        # Use the x data from the point cloud and the ranges from laserscan
        # to get an x-component of each obstacle for the potential field
        # Note that the parameters for the hyperbolic tangent function is arbitrary
        dir_x = [-(-tanh(3*(ranges))+1) * points.x /2 for ranges, points in zip(msg.ranges, self.cloud.points)]

        # Do the same procedure for the y points
        dir_y = [(-tanh(3*(ranges))+1) * points.y / 2 for ranges, points in zip(msg.ranges, self.cloud.points)]

        # Laserscan data may return inf for out of range, which results in nan
        # post calculation, so filter those out of the potential field
        dir_x_filtered = [x if not isnan(x) else 0 for x in dir_x]
        dir_y_filtered = [y if not isnan(y) else 0 for y in dir_y]
        
        # Make some arbitrary value for the forward field in the base frame
        base_frame_vector = 0.5

        # Sum the x components and add the base vector to move forward
        sum_x = sum(dir_x_filtered) + base_frame_vector

        # Sum the y components
        sum_y = sum(dir_y_filtered)


        # Some geometry to caclulate the magnitude of the components
        magnitude = sqrt(sum_x ** 2 + sum_y ** 2)

        # Use atan2 to get full 360 degree angles, rather than 180 degrees with atan
        angle = atan2(sum_y,sum_x)

        # Make FieldVector message object (custom message for this hw)
        field_vec = FieldVector()

        # Store magnitude and angle of the field vector to the message
        field_vec.magnitude = magnitude
        field_vec.angle = angle

        # Publish message
        self.pub.publish(field_vec)


if __name__ == "__main__":

    # Startup node
    rospy.init_node('field_calc', log_level=rospy.DEBUG)

    # Initialize FrontFilter with robot width, and base frame
    # Turtlebot3 with radius of rotation of 0.21m
    # Base frame (center of rotation) is 'base_link'
    filter = PotentialField(0.21, 'base_link')

    # Leave it all up to ROS
    rospy.spin()
