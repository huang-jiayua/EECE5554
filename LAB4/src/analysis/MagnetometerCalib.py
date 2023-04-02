import math
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sea
import pandas

plt.rcParams.update({'font.size': 8})
sea.color_palette(as_cmap=True)

bag = bagreader('/home/huangjiayua/EECE5554/LAB4/src/lab4_driver/data/donut.bag')
Imu_data = bag.message_by_topic('/imu')
Imu_readings = pandas.read_csv(Imu_data)
plt.grid(color='grey', linestyle='--', linewidth=0.5)
plt.scatter(Imu_readings['mag_field.magnetic_field.x'], Imu_readings['mag_field.magnetic_field.y'], marker='.',
            label='Raw/Uncalibrated Data', color='lightcoral')
donut = plt.Circle((-0.38, 0.38), 0.2, fill=False, color='black')
plt.gca().add_patch(donut)
plt.gca().set_aspect("equal")

# CALIBRATION
min_x = min(Imu_readings['mag_field.magnetic_field.x'])
max_x = max(Imu_readings['mag_field.magnetic_field.x'])
min_y = min(Imu_readings['mag_field.magnetic_field.y'])
max_y = max(Imu_readings['mag_field.magnetic_field.y'])

x_offset = (min_x + max_x) / 2.0
y_offset = (min_y + max_y) / 2.0
print("hard-iron x_axis_Offset=", x_offset)
print("hard-iron y_axis_Offset=", y_offset)
HI_x = []
HI_x.extend((Imu_readings['mag_field.magnetic_field.x'] - x_offset))
HI_y = []
HI_y.extend((Imu_readings['mag_field.magnetic_field.y'] - y_offset))

plt.grid(color='grey', linestyle='--', linewidth=1)
plt.scatter(HI_x, HI_y, marker='.', label='Hard-Iron Calibrated Data', color='green')
donut = plt.Circle((0.0, 0.0), 0.2, fill=False, color='black')
plt.gca().add_patch(donut)
plt.gca().set_aspect("equal", adjustable='box')
plt.title('Hard_Iron_Calibration Plot Of Magnetic Field X vs Y')
plt.xlabel('Hard-Iron Calibrated X')
plt.ylabel('Hard-Iron Calibrated Y')
plt.legend()
plt.show()

radius = math.sqrt((HI_x[2000] ** 2) + (HI_y[2000] ** 2))
print('radius = ', radius)
theta = np.arcsin((HI_y[2000] / radius))
print('theta = ', theta)

rotate = [[np.cos(theta), np.sin(theta)], [np.sin(-theta), np.cos(theta)]]
v = [HI_x, HI_y]
rotated = np.matmul(rotate, v)
print(np.shape(rotated))
plt.grid(color='grey', linestyle='--', linewidth=0.5)
plt.scatter(rotated[0], rotated[1], marker='.', label='Soft-Iron Calibrated', color='blue')
donut = plt.Circle((0.0, 0.0), 0.2, fill=False, color='black')
plt.gca().add_patch(donut)
plt.gca().set_aspect("equal", adjustable='box')
plt.title('Soft_Iron_Calibration Of Magnetic Field X vs Y')
plt.xlabel('Soft-Iron Calibrated X')
plt.ylabel('Soft-Iron Calibrated Y')
plt.legend()
plt.show()

sigma = 0.15 / 0.2
print('sigma = ', sigma)

calib = [[1, 0], [0, sigma]]
temp = np.matmul(calib, rotated)
theta = 0 - theta
rotate_new = [[np.cos(theta), np.sin(theta)], [np.sin(-theta), np.cos(theta)]]
rotated_new = np.matmul(rotate_new, temp)
print(np.shape(rotated_new))
plt.grid(color='grey', linestyle='--', linewidth=0.5)
plt.scatter(rotated_new[0], rotated_new[1], marker='.', label='Double Calibrated Data', color='purple')
donut = plt.Circle((0.0, 0.0), 0.15, fill=False, color='black')
plt.gca().add_patch(donut)
plt.gca().set_aspect("equal", adjustable='box')
plt.title('Double Calibrated Plot Of Magnetic Field X vs Y')
plt.xlabel('Double Calibrated X')
plt.ylabel('Double Calibrated Y')
plt.legend()
plt.show()
