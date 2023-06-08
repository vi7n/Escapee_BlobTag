import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon

# Create a heptagon with 7 sides
num_sides = 7
radius = 5
theta = np.linspace(0, 2*np.pi, num_sides+1)
x = radius * np.cos(theta)
y = radius * np.sin(theta)
coords = list(zip(x,y))
heptagon = Polygon(coords)

# Generate random point inside the heptagon
minx, miny, maxx, maxy = heptagon.bounds
while True:
    point = Point(np.random.uniform(minx, maxx), np.random.uniform(miny, maxy))
    if heptagon.contains(point):
        break

# Calculate distance from point to each side of the heptagon
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


# Plot the heptagon and the point using matplotlib
x,y = heptagon.exterior.xy
plt.plot(x,y)
plt.scatter(point.x, point.y)
plt.show()
