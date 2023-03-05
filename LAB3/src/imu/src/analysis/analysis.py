import bagpy
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd
sns.set_theme()
plt.rcParams.update({'font.size': 10})

bag = bagreader('/home/huangjiayua/EECE5554/src/LAB2/Data/Open_Space/Stationry_Data.bag')
#bag.topic_table
data = bag.message_by_topic('/rtk_gps_message')
readings = pd.read_csv(data)
readings['UTM_easting'] = readings['UTM_easting'] - readings['UTM_easting'].min()
readings['UTM_northing'] = readings['UTM_northing'] - readings['UTM_northing'].min()
print(readings[['UTM_easting', 'UTM_northing']])
print(readings)
readings[['UTM_easting','UTM_northing']].plot()
plt.show()
sns.scatterplot(x = 'UTM_easting', y = 'UTM_northing', data = readings, s= 50, label = 'UTM_easting VS UTM_northing')
plt.xlabel('UTM_easting (meters)')
plt.ylabel('UTM_northing (meters)')
plt.title('UTM_easting vs UTM_northing')
plt.show()
