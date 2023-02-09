import bagpy
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import pandas

bag = bagreader('/home/huangjiayua/catkin_ws/src/gps_driver/Data/OpenSpace.bag')
data = bag.message_by_topic('/gps')
readings = pandas.read_csv(data)

_, ax = bagpy.create_fig(1)

myX = 42.3371198
myY = -71.0905213

# calculate error from my location in x and y
readings['latitude'].substract(myX, axis = 'latitude')
readings['longitude'].substract(myY, axis = 'longitude')

# scatter plot the error in x and y
ax[0].scatter(x = 'latitude', y = 'longitude', data = readings, s= 50, label = 'altitude vs time')

# add labels
ax[0].legend()

# add labels for x and y axis
ax[0].set_xlabel('X error')
ax[0].set_ylabel('Y error')

# show the plot
plt.show()
