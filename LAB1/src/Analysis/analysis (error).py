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

# calculate the mean of error from my location in x and y
meanX = readings['Latitude'].mean() - myX
meanY = readings['Longitude'].mean() - myY

# calculate the median of error from my location in x and y
medianX = readings['Latitude'].median() - myX
medianY = readings['Longitude'].median() - myY

print(f'meanX: {meanX}, meanY: {meanY}')
print(f'medianX: {medianX}, medianY: {medianY}')

# print(readings['Time'])

# calculate error from my location in x and y
readings['Latitude'] = readings['Latitude'] - myX
readings['Longitude'] = readings['Longitude'] - myY

# scatter plot the error in x and y
ax[0].scatter(x = 'Latitude', y = 'Longitude', data = readings, s= 50, label = 'Latitude vs Longitude')

# add range for x and y axis
ax[0].set_xlim(-0.0001, 0.0001)
ax[0].set_ylim(-0.0001, 0.0001)

# add grid
ax[0].grid()

# add labels
ax[0].legend()


# add labels for x and y axis
ax[0].set_xlabel('X error')
ax[0].set_ylabel('Y error')

# show the plot
plt.show()
