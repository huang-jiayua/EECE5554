import bagpy
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import pandas

bag = bagreader('/home/huangjiayua/catkin_ws/src/gps_driver/data/OcculatedSpace.bag')
data = bag.message_by_topic('/gps')
readings = pandas.read_csv(data)
readings['UTM_easting'] = readings['UTM_easting'] - readings['UTM_easting'].min()
readings['UTM_northing'] = readings['UTM_northing'] - readings['UTM_northing'].min()

_, ax = bagpy.create_fig(1)
print(len(ax))
ax[0].scatter(x = 'UTM_easting', y = 'UTM_northing', data = readings, label = 'UTM_easting VS UTM_northing')
ax[0].legend()
ax[0].set_xlabel('UTM_easting')
ax[0].set_ylabel('UTM_northing')
plt.show()
