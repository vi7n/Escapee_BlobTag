import utm

latitude = 44.565278920498805
longitude = -123.30262330229871

utm_coords = utm.from_latlon(latitude, longitude)
print(utm_coords)
