import bagpy
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import pandas

bag = bagreader('/home/huangjiayua/catkin_ws/src/gps_driver/data/OpenSpace.bag')
# bag.topic_table
data = bag.message_by_topic('/gps')
readings = pandas.read_csv(data)

_, ax = bagpy.create_fig(1)
print(len(ax))
ax[0].scatter(x = 'header.stamp.secs', y = 'Altitude', data = readings, s= 50, label = 'altitude vs time')
ax[0].legend()
ax[0].set_xlabel('Time')
ax[0].set_ylabel('Altitude')
plt.show()
