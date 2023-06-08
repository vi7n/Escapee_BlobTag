import pyproj
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
import numpy as np

def convert_to_epsg_32610(lat, lon):
    epsg_32610 = pyproj.Proj('epsg:32610')
    x, y = epsg_32610(lon, lat)
    return x, y

lonlat_matrix = [-123.3027021,44.5653358],[-123.3025975,44.5653402],[-123.3024642,44.5653228],[-123.3024653,44.5652226],[-123.3027049,44.5652208],[-123.3027935,44.5652523],[-123.3027909,44.5653197],[-123.3027021,44.5653358]

final_matrix = np.zeros((8, 2))

for i in range(8):
    final_matrix[i] = convert_to_epsg_32610(lonlat_matrix[i][1],lonlat_matrix[i][0])

polygon = Polygon(final_matrix)

x,y = polygon.exterior.xy

fig = plt.figure(1, figsize=(5,5), dpi=90)
ax = fig.add_subplot(111)
ax.plot(x,y)

plt.show()
