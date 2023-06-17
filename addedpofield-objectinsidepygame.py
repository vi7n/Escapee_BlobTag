import pygame
import random
from shapely.geometry import Point, Polygon

# Initialize pygame
pygame.init()

# Set screen size
screen = pygame.display.set_mode((500, 500))

# Set colors
white = (255, 255, 255)
black = (0, 0, 0)
red = (255, 0, 0)

# Set polygon points
points = [(100, 100), (400, 100), (400, 400), (100, 400)]
polygon = Polygon(points)

# Set circle properties
circle_pos = [250, 250]
circle_radius = 7
circle_speed = [random.randint(-5,5), random.randint(-5,5)]
circle_speed = [2,2]
# Run game loop
running = True
while running:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Clear screen
    screen.fill(white)

    # Draw polygon
    pygame.draw.polygon(screen, black, points)

    # Move circle
    circle_pos[0] += circle_speed[0]
    circle_pos[1] += circle_speed[1]

    # Check if circle is inside polygon
    if not polygon.contains(Point(circle_pos)):
        # Move circle back inside polygon
        circle_pos[0] -= circle_speed[0]
        circle_pos[1] -= circle_speed[1]
        # Reverse direction
        circle_speed[0] = -circle_speed[0]
        circle_speed[1] = -circle_speed[1]

    # Draw circle
    pygame.draw.circle(screen, red, circle_pos, circle_radius)

    # Update screen
    pygame.display.flip()

# Quit pygame
pygame.quit()

