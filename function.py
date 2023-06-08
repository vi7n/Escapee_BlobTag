import pyproj
import pygame
import numpy as np
from shapely.geometry import Polygon, Point, LineString
from shapely.geometry import Point, LineString

def convert_to_epsg_32610(lat, lon):
    epsg_32610 = pyproj.Proj('epsg:32610')
    x, y = epsg_32610(lon, lat)
    return x, y

def draw_polygon(poly : Polygon , color : tuple , width : int ):
    points=[point_to_pixel(Point(p )) for p in poly.exterior.coords ]
    pygame.draw.polygon(screen , color , points , width )

final_matrix = np.zeros((8, 2))

# (475962.7519573149 4934711.119215475, 475971.05994338985 4934711.577173998, 475981.6379940134 4934709.605181078, 475981.5094134559 4934698.475402928, 475962.48225412774 4934698.3459709175, 475955.4595843022 4934701.871042486, 475955.6938117249 4934709.35699199, 475962.7519573149 4934711.119215475)
lonlat_matrix = [-123.3027021,44.5653358],[-123.3025975,44.5653402],[-123.3024642,44.5653228],[-123.3024653,44.5652226],[-123.3027049,44.5652208],[-123.3027935,44.5652523],[-123.3027909,44.5653197],[-123.3027021,44.5653358]
print(np.shape(lonlat_matrix))

final_matrix[0] = convert_to_epsg_32610(lonlat_matrix[0][1],lonlat_matrix[0][0])
# print(final_matrix)

for i in range(8):
    final_matrix[i] = convert_to_epsg_32610(lonlat_matrix[i][1],lonlat_matrix[i][0])

print(final_matrix)
polygon = Polygon(final_matrix)

print(polygon)

pygame.init()


screen = pygame.display.set_mode((640, 480))
pygame.display.set_caption("Polygon")

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()

    screen.fill((255, 255, 255))

    pygame.draw.polygon(screen, (0, 0, 255), polygon.exterior.coords[:], width=2)

    pygame.display.flip()
