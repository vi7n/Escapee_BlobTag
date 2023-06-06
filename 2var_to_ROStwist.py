import rospy
import math
from geometry_msgs.msg import Twist

# Assume speed and direction are given as input
speed = 10 # m/s
direction = 45 # degrees

# Create a Twist object

twist = Twist()

# Set the linear velocity in x direction
twist.linear.x = speed * math.cos(math.radians(direction))

# Set the angular velocity in z direction
twist.angular.z = speed * math.sin(math.radians(direction))

# Publish the twist message to a topic
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pub.publish(twist)
