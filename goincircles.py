import numpy as np
import geopandas
from shapely.geometry import Point, LineString
from shapely.geometry.polygon import Polygon
from math import atan, radians, pi, degrees
import pygame 


THRESHOLD = 1 # Distance threshold, meters
K = 0.5 # Gain factor for the variable vector
DT = 1 # Time step, seconds


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

# Open geojson as UTM zone 10T (lat/ lon in meters)
gp_frame = geopandas.read_file("./export1.json").to_crs('EPSG:32610')
border_poly = gp_frame[gp_frame['name'] == 'Survey Area']['geometry'][0]

# Assume some initial position and heading for the object inside the polygon
object_point = Point(478113, 4934737)
object_heading = radians(45)

# Assume some initial movement vector for the object (linear and angular velocities)
movement_vector = np.array([1.5, 0.25]) # [m/s, rad/s]

# Initialize pygame and create a window for visualization
pygame.init()
screen_width = 800 # Window width in pixels
screen_height = 600 # Window height in pixels
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption('Object Movement')
clock = pygame.time.Clock()

# Define some colors for drawing
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

# Define some scaling factors for converting coordinates to pixels
scale_x = screen_width / (border_poly.bounds[2] - border_poly.bounds[0]) # Pixels per meter along x-axis
scale_y = screen_height / (border_poly.bounds[3] - border_poly.bounds[1]) # Pixels per meter along y-axis

# Define a function to convert a point object to a pixel position on the screen
def point_to_pixel(point: Point) -> tuple:  
    x_pixel=int((point.x - border_poly.bounds [0 ])* scale_x )
    y_pixel=int((border_poly.bounds [3 ]- point.y )* scale_y )
    return (x_pixel , y_pixel )

# Define a function to draw a polygon object on the screen with a given color and width 
def draw_polygon(poly : Polygon , color : tuple , width : int ):
    points=[point_to_pixel(Point(p )) for p in poly.exterior.coords ]
    pygame.draw.polygon(screen , color , points , width )

# Define a function to draw a circle object on the screen with a given color and radius 
def draw_circle(point : Point , color : tuple , radius : int ):
    pixel=point_to_pixel(point )
    pygame.draw.circle(screen , color , pixel , radius )

# Define a function to draw an arrow object on the screen with a given color and length 
def draw_arrow(point : Point , heading : float , color : tuple , length : int ):
    start_pixel=point_to_pixel(point )
    end_pixel_x=start_pixel [0 ]+ int(length * np.cos(heading ))
    end_pixel_y=start_pixel [1 ]- int(length * np.sin(heading ))
    end_pixel=(end_pixel_x , end_pixel_y )
    pygame.draw.line(screen ,color ,start_pixel ,end_pixel ,width=2 )

# Loop until the user quits or closes the window 
running=True 
while running:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running=False 

    # Fill the screen with black color 
    screen.fill(BLACK)

    # Draw the polygon boundary with white color and 2 pixels width 
    draw_polygon(border_poly , WHITE ,2 )

    # Calculate the variable vector based on the position , heading , and polygon 
    variable_vector = calculate_variable_vector(object_point , object_heading , border_poly )

    # Add the variable vector to the movement vector 
    adjusted_vector = movement_vector + variable_vector 
    if variable_vector[0] != 0:
        adjusted_vector = variable_vector
    # movement_vector = adjusted_vector
    print("variable vector",variable_vector)

    # Draw the object as a red circle with 10 pixels radius 
    draw_circle(object_point , RED ,10 )

    # Draw the movement vector as a green arrow with 10 pixels length 
    draw_arrow(object_point , object_heading , GREEN ,10 )

    # Draw the variable vector as a blue arrow with 10 pixels length 
    draw_arrow(object_point , object_heading + pi /2 , BLUE ,10 )

    # Update the position of the object based on the movement vector and the time step 
    object_point = Point(np.array(object_point.coords [0 ])+ adjusted_vector [0 ]* DT * np.array([np.cos(object_heading ), np.sin(object_heading )]))

    # Update the heading of the object based on the movement vector and the time step 
    object_heading += adjusted_vector [1 ]* DT 

    # Print the movement vector 
    print(f'Movement vector : {adjusted_vector }')

    # Update the display 
    pygame.display.flip()

    # Wait until next cycle 
    clock.tick(1 / DT )

# Quit pygame 
pygame.quit()
