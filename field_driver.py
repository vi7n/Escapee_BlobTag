#!/usr/bin/env python3

# Farhan Rozaidi, nikahman@oregonstate.edu
#
# This script uses potential field vectors to move the turtlebot in a space

# Import necessary ROS stuff
import rospy
import sys

# Math functions for neat calculations
from numpy import tanh, cos

# Get all the right messages for velocity commands and field vectors
from geometry_msgs.msg import Twist
from hw1.msg import FieldVector


class FieldDriver:
    def __init__(self):

        # Subscribe to the filtered field vector, and publish velocity
        self.sub = rospy.Subscriber(
            'field_vector', FieldVector, self.scan_callback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Set a conservative maximum velocity
        self.max_vel = 0.5


    def scan_callback(self, msg):

        # Initialize Twist object, with 0 for everything (no movement)
        move = Twist()
        move.linear.x = 0.0
        move.linear.y = 0.0
        move.linear.z = 0.0
        move.angular.x = 0.0
        move.angular.y = 0.0
        move.angular.z = 0.0

        # Calculate the x-component of the field vector
        x_dir = msg.magnitude * cos(msg.angle)

        # If the x movement is too fast, limit speed
        if x_dir > self.max_vel:
            x_dir = self.max_vel
        elif x_dir < -self.max_vel:
            x_dir = -self.max_vel

        # Set the x velocity to the limited speed
        move.linear.x = x_dir

        # Use the hyperbolic tangent function to determine the speed of rotation
        move.angular.z = tanh(-msg.angle)

        # Publish velocity command
        self.pub.publish(move)


if __name__ == "__main__":

    # Initialize node
    rospy.init_node('field_driver', log_level=rospy.DEBUG)

    # Initialize Field Driver object
    field_driver = FieldDriver()

    # Let ROS do its thing
    rospy.spin()
