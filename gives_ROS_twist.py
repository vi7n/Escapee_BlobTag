import numpy as np
import geopandas
from shapely.geometry import Point, LineString
from shapely.geometry.polygon import Polygon
from math import atan, radians, pi
import rospy
from geometry_msgs.msg import Twist

# Define constants
THRESHOLD = 3 # Distance threshold in meters
K = 0.5 # Change the gain factor for the variable vector

# Define functions same as Nathans'
def find_intersection(point: Point, heading: float, poly: Polygon) -> Point:
    """
    cast a ray from a point with a given heading and check where it intersects the polygon

    ** heading should be in radians! convert it with radians(degrees) if needed
    """

    dy = 1000 # this is an arbitrary distance. 1km should be plenty for our application
    dx = atan(heading) * dy

    p1 = np.array([tmp[0] for tmp in point.coords.xy])
    p2 = p1 + np.array([dx, dy])

    ray = LineString([Point(p1), Point(p2)])

    if not ray.intersects(poly):
        intersection = None
    else:
        xs, ys = ray.intersection(poly).coords.xy
        intersection = Point(list(zip(xs, ys))[1])

    return intersection


def border_dist(point: Point, heading: float, poly: Polygon) -> float:
    """
    calculate distance from point to edge of polygon on some heading 

    ** heading should be in radians! convert it with radians(degrees) if needed
    """

    intersection = find_intersection(point, heading, poly)

    if intersection is None:
        return None
 
    return point.distance(intersection)

def calculate_variable_vector(point: Point, heading: float, poly: Polygon) -> np.array:
    """
    calculate the variable vector that adds on to the movement vector to steer away from the boundary

    ** heading should be in radians! convert it with radians(degrees) if needed
    """

    distance = border_dist(point, heading, poly)
    if distance is None or distance > THRESHOLD:
        return np.array([0, 0]) # No need to adjust the movement vector
 
    # Find the normal vector to the boundary at the intersection point
    intersection = find_intersection(point, heading, poly)
    boundary = poly.boundary
    segments = list(zip(boundary.coords[:-1], boundary.coords[1:]))
    for segment in segments:
        line = LineString(segment)
        if line.contains(intersection):
            break
 
    p1, p2 = segment
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    normal = np.array([-dy, dx]) # Perpendicular to the segment vector
 
    # Make sure the normal vector points inward
    centroid = np.array(poly.centroid.coords[0])
    if np.dot(normal, centroid - intersection.coords[0]) < 0:
        normal = -normal
 
    # Scale the normal vector by a factor that depends on the distance and the gain factor
    factor = K * (THRESHOLD - distance)
    variable = factor * normal
 
    return variable

# Initialize ROS node and publisher
rospy.init_node('object_movement')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# Open geojson as UTM zone 10T (lat/ lon in meters)
gp_frame = geopandas.read_file("./export1.json").to_crs('EPSG:32610')
border_poly = gp_frame[gp_frame['name'] == 'Survey Area']['geometry'][0]

# Assume some initial position and heading for the object inside the polygon
object_point = Point(475968, 4934705)
object_heading = radians(45)

# Assume initial movement vector for the object (linear and angular velocities)
movement_vector = np.array([0.5, 0.5]) # [m/s, rad/s] this makes sure our robot is moving (useful because the motor contoller needs instruction every 1 second)

twist = Twist()

rate = rospy.Rate(10) #Hz
while not rospy.is_shutdown():
    # Calculate the variable(potential's replacement) vector based on the position, heading, and polygon
    variable_vector = calculate_variable_vector(object_point, object_heading, border_poly)

    # Add the variable vector to the movement vector
    adjusted_vector = movement_vector + variable_vector #Please add a random uniform vector as you see fit to make sure agents exert different behaviors

    # Assign the adjusted vector to the twist message fields
    twist.linear.x = adjusted_vector[0]
    twist.angular.z = adjusted_vector[1]

    # Publish the twist message
    pub.publish(twist)

    # Update the position and heading of the object based on the movement vector and the time step
    dt = rate.sleep_dur.to_sec()
    object_point = object_point + Point(movement_vector[0] * dt * np.cos(object_heading), movement_vector[0] * dt * np.sin(object_heading))
    object_heading += movement_vector[1] * dt

   # Sleep until next cycle
    rate.sleep()