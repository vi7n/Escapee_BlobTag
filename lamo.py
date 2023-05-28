import geopandas

gp_frame = geopandas.read_file("./export1.json").to_crs('EPSG:32610')

print(gp_frame)