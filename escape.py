# import rospy
# import math
from geonav_transform.geonav_conversions import LonLatToUTM
# from geometry_msgs.msg import Twist
# from gps_common.msg import GPSFix

# Define GPS coordinates of polygon
gps_coords = [[-123.3027021,44.5653358],[-123.3025975,44.5653402],[-123.3024642,44.5653228],[-123.3024653,44.5652226],[-123.3027049,44.5652208],[-123.3027935,44.5652523],[-123.3027909,44.5653197],[-123.3027021,44.5653358]]

# Convert GPS coordinates to UTM coordinates
utm_coords = [LonLatToUTM(lat, lon) for lon, lat in gps_coords]

# # Calculate centroid of polygon
# centroid_x = sum(x for x, y in utm_coords) / len(utm_coords)
# centroid_y = sum(y for x, y in utm_coords) / len(utm_coords)

# # Define current position and heading of robot (start position)
# current_x = None
# current_y = None
# current_heading = 0 # heading in radians

# # Define initial speed of robot
# speed = 1 # m/s

# def gps_callback(data):
#     global current_x, current_y

#     # Get current position from GPSFix message
#     lat = data.latitude
#     lon = data.longitude

#     # Convert current position from GPS to UTM coordinates
#     current_x, current_y = LonLatToUTM(lat, lon)

# # Initialize ROS node and publisher
# rospy.init_node('move_robot')
# pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# # Subscribe to GPSFix topic to get current position of robot
# rospy.Subscriber('/GPSFix', GPSFix, gps_callback)

# while not rospy.is_shutdown():
#     if current_x is not None and current_y is not None:
#         # Calculate distance from current position to closest boundary
#         min_distance = min(math.sqrt((current_x - x)**2 + (current_y - y)**2) for x, y in utm_coords)

#         # Check if robot is within 2.5 meters of any boundary
#         if min_distance <= 2.5:
#             # Calculate difference between current position and goal position (centroid)
#             diff_x = centroid_x - current_x
#             diff_y = centroid_y - current_y

#             # Calculate desired heading towards centroid
#             desired_heading = math.atan2(diff_y, diff_x)

#             # Calculate difference between current heading and desired heading
#             heading_diff = desired_heading - current_heading

#             # Limit heading difference to range [-pi, pi]
#             heading_diff = (heading_diff + math.pi) % (2 * math.pi) - math.pi

#             # Set angular velocity to turn towards desired heading
#             angular_velocity = heading_diff
#         else:
#             # Maintain current heading
#             angular_velocity = 0

#         # Create and publish twist message to move robot towards goal
#         twist = Twist()
#         twist.linear.x = speed * math.cos(current_heading)
#         twist.linear.y = speed * math.sin(current_heading)
#         twist.angular.z = angular_velocity
#         pub.publish(twist)

#     rospy.sleep(0.1)
