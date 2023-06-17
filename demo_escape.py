import rospy
import math
from geonav_transform.geonav_conversions import LonLatToUTM
from geometry_msgs.msg import Twist
from gps_common.msg import GPSFix

this_robot_id = 99999 #name id here ###############################################

gps_coords = [[-123.3027021,44.5653358],[-123.3025975,44.5653402],[-123.3024642,44.5653228],[-123.3024653,44.5652226],[-123.3027049,44.5652208],[-123.3027935,44.5652523],[-123.3027909,44.5653197],[-123.3027021,44.5653358]]

utm_coords = [LonLatToUTM(lat, lon) for lon, lat in gps_coords]

# centroid of polygon is the heading goal of our robot if we are near the boundary
centroid_x = 475968.9857816565 #sum(x for x, y in utm_coords) / len(utm_coords)
centroid_y = 4934704.778117255 #sum(y for x, y in utm_coords) / len(utm_coords)

current_x = None
current_y = None
current_heading = 0 # heading in radians

speed = 1 # m/s

# Dictionary to store positions of other robots
other_robots = {}

def gps_callback(data):
    global current_x, current_y

    # Get robot ID from frame_id field of GPSFix message
    robot_id = data.header.frame_id
    data.gps_loc
    #self gps location
    #group launch
    

    # Get current position from GPSFix message
    lat = data.latitude
    lon = data.longitude

    # Convert current position from GPS to UTM coordinates
    x, y = LonLatToUTM(lat, lon)

    if robot_id == this_robot_id:
        # Update current position of our robot
        current_x = x
        current_y = y
    else:
        # Update position of other robot in dictionary
        other_robots[robot_id] = (x, y)

# Initialize ROS node and publisher
rospy.init_node('move_robot')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

rospy.Subscriber('/info', Escapee, gps_callback)

while not rospy.is_shutdown():
    if current_x is not None and current_y is not None:
        # Calculate distance from current position to closest boundary
        min_distance = min(math.sqrt((current_x - x)**2 + (current_y - y)**2) for x, y in utm_coords)

        # Threshold = 2.5 meters (the shortest side is 8 meters) ######!~!!!!!!!!
        if min_distance <= 2.5:

            diff_x = centroid_x - current_x
            diff_y = centroid_y - current_y

            desired_heading = math.atan2(diff_y, diff_x)

            heading_diff = desired_heading - current_heading

            heading_diff = (heading_diff + math.pi) % (2 * math.pi) - math.pi

            angular_velocity = heading_diff
        else:
            angular_velocity = 0

        # Check for nearby robots 
        for other_robot_id, (other_robot_x, other_robot_y) in other_robots.items():
            distance_to_other_robot = math.sqrt((current_x - other_robot_x)**2 + (current_y - other_robot_y)**2)
            if distance_to_other_robot <= 0.75:
                repulsive_force_x = (current_x - other_robot_x) / distance_to_other_robot**2
                repulsive_force_y = (current_y - other_robot_y) / distance_to_other_robot**2

                angular_velocity += math.atan2(repulsive_force_y, repulsive_force_x) - current_heading

        twist = Twist()
        twist.linear.x = speed * math.cos(current_heading)
        twist.linear.y = speed * math.sin(current_heading)
        twist.angular.z = angular_velocity
        pub.publish(twist)

    rospy.sleep(0.1)
