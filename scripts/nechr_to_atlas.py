import os
import math
import pyproj
import matplotlib.pyplot as plt
import numpy as np

def convert_wgs_to_utm(lon, lat):
    utm_band = str((math.floor((lon + 180) / 6 ) % 60) + 1)
    if len(utm_band) == 1:
        utm_band = '0'+utm_band
    if lat >= 0:
        epsg_code = '326' + utm_band
    else:
        epsg_code = '327' + utm_band
    return epsg_code

lat_c = 50.345749
lon_c = 13.325244
utm_code = convert_wgs_to_utm(lon_c,lat_c)
crs_wgs = pyproj.Proj(init='epsg:4326') # assuming you're using WGS84 geographic
crs_utm = pyproj.Proj(init='epsg:{0}'.format(utm_code))

x_c, y_c = pyproj.transform(crs_wgs, crs_utm, lon_c, lat_c)

ax = plt.subplot()

pylon_index = 1

path = os.path.abspath('../gps/')
positions_output = ''
connections_output = ''
files = os.listdir(path)
plotpos = []
for file in files:
    filepath = os.path.join(path, file)
    file = open(filepath, 'r')
    lines = file.readlines()

    for line in lines:
        line = line.strip().rstrip(', \n')
        line = line.split()
        lat = line[1]
        lon = line[2]
        x, y = pyproj.transform(crs_wgs, crs_utm, lon, lat)
        if np.sqrt((x-x_c)**2 + (y-y_c)**2) < 10000:
            plotpos.append([x-x_c,y-y_c])
            positions_output = positions_output+str(x-x_c)+' '+str(y-y_c)+' 0;\n'

    connections_output = connections_output+str(pylon_index+1)+';\n'
    for i in range(len(positions_output)-2):
        connections_output = connections_output+str(pylon_index)+' '+str(pylon_index + 2)+';\n'
        pylon_index += 1
    connections_output = connections_output + str(pylon_index)+';\n'
    pylon_index += 2
    file.close()

output = open(os.path.join(os.path.abspath('.'),'nechranice_graph_10k.yaml'),'w')
output.write('recharge_land_stations: \"0 0 0;\"\n\nregular_land_stations: \"0 0 0;\"\n\npylon_positions: \"')
output.write(positions_output)
output.write('\"\n\nconnections_indexes: \"')
output.write(connections_output)
output.write('\"')
output.close()

plotpos = np.array(plotpos)
ax.plot(plotpos[:,0], plotpos[:,1])
plt.show()


