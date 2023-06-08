import pygame
import numpy as np
from shapely.geometry import Point, Polygon, LineString
from shapely.wkt import loads

# Initialize pygame
pygame.init()
width, height = 640, 480
screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()

# Create a polygon using the provided coordinates
polygon_wkt = "POLYGON ((475962.7519573149 4934711.119215475, 475971.05994338985 4934711.577173998, 475981.6379940134 4934709.605181078, 475981.5094134559 4934698.475402928, 475962.48225412774 4934698.3459709175, 475955.4595843022 4934701.871042486, 475955.6938117249 4934709.35699199, 475962.7519573149 4934711.119215475))"
polygon = loads(polygon_wkt)
coords = list(polygon.exterior.coords)
num_sides = len(coords) - 1

# Scale and translate polygon to fit screen
minx, miny, maxx, maxy = polygon.bounds
scale = min(width / (maxx - minx), height / (maxy - miny))
coords = [(int((x - minx) * scale), int((y - miny) * scale)) for x,y in coords]
polygon = Polygon(coords)

# Generate random point inside the polygon
minx, miny, maxx, maxy = polygon.bounds
while True:
    point_coords = np.array([np.random.uniform(minx, maxx), np.random.uniform(miny, maxy)])
    point = Point(point_coords)
    if polygon.contains(point):
        break

# Set initial velocity of object (m/s) and heading direction (radians)
speed = 1 * scale / clock.tick(60)
velocity = np.array([speed * np.cos(0), speed * np.sin(0)])

# Main game loop
while True:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()

    # Clear screen
    screen.fill((255,255,255))

    # Draw polygon
    pygame.draw.polygon(screen, (0,0,0), coords)

    # Draw object
    pygame.draw.circle(screen, (255,0,0), point_coords.astype(int), 5)

    # Calculate distance from point to each side of the polygon
    distances = []
    for i in range(num_sides):
        x1, y1 = coords[i]
        x2,y2 = coords[i+1]
        A = y2 - y1
        B = x1 - x2
        C = (x2*y1) - (x1*y2)
        distance = abs((A*point.x) + (B*point.y) + C) / np.sqrt((A**2) + (B**2))
        distances.append(distance)

    # Check if distance < 5 meters and update velocity accordingly
    if any(distance < 2.5 for distance in distances):
        centroid_coords = np.array(polygon.centroid.coords[0])
        velocity += (centroid_coords - point_coords) * 0.001
    print(velocity)


    # Update position of object
    point_coords += velocity
    point = Point(point_coords)

    # Update screen
    pygame.display.flip()
    clock.tick(1000)
