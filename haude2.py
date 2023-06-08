#draws a line to the centroid

import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon, LineString
from shapely.wkt import loads

# Create a polygon using the provided coordinates
polygon_wkt = "POLYGON ((475962.7519573149 4934711.119215475, 475971.05994338985 4934711.577173998, 475981.6379940134 4934709.605181078, 475981.5094134559 4934698.475402928, 475962.48225412774 4934698.3459709175, 475955.4595843022 4934701.871042486, 475955.6938117249 4934709.35699199, 475962.7519573149 4934711.119215475))"
polygon = loads(polygon_wkt)
coords = list(polygon.exterior.coords)
num_sides = len(coords) - 1

# Generate random point inside the polygon
minx, miny, maxx, maxy = polygon.bounds
while True:
    point = Point(np.random.uniform(minx, maxx), np.random.uniform(miny, maxy))
    if polygon.contains(point):
        break

# Calculate distance from point to each side of the polygon
distances = []
for i in range(num_sides):
    x1, y1 = coords[i]
    x2, y2 = coords[i+1]
    A = y2 - y1
    B = x1 - x2
    C = (x2*y1) - (x1*y2)
    distance = abs((A*point.x) + (B*point.y) + C) / np.sqrt((A**2) + (B**2))
    distances.append(distance)
    print(f"Distance from point to side {i+1}: {distance:.2f}")

# Draw a line from point to centroid of polygon if distance < 5 meters
if any(distance < 5 for distance in distances):
    centroid = polygon.centroid
    line = LineString([point, centroid])
    x,y = line.xy
    plt.plot(x,y)

    # Calculate angle between centroid and line made if object was heading towards nearest side
    nearest_side_index = np.argmin(distances)
    x1, y1 = coords[nearest_side_index]
    x2, y2 = coords[nearest_side_index+1]
    side_line = LineString([(x1,y1), (x2,y2)])
    intersection_point = side_line.intersection(line)
    
    if intersection_point:
        angle_line = LineString([centroid, intersection_point])
        angle = np.degrees(np.arctan2(centroid.y-intersection_point.y, centroid.x-intersection_point.x) - np.arctan2(point.y-intersection_point.y, point.x-intersection_point.x))
        print(f"Angle between centroid and line made if object was heading towards nearest side: {angle:.2f} degrees")
        x,y = angle_line.xy
        plt.plot(x,y)

# Plot the polygon and the point using matplotlib
x,y = polygon.exterior.xy
plt.plot(x,y)
plt.scatter(point.x, point.y)
plt.show()
