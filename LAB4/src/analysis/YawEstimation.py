import math
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sea
import pandas
import scipy.integrate as integrate
from scipy.signal import butter
from scipy import signal

plt.rcParams.update({'font.size': 8})
sea.color_palette(as_cmap=True)

bag = bagreader('/home/huangjiayua/EECE5554/LAB4/src/lab4_driver/data/boston_mini_tour.bag')
Imu_data = bag.message_by_topic('/imu')
Imu_readings = pandas.read_csv(Imu_data)

# From MagnetometerCalib.py, recalculate the rotation matrix
min_x = min(Imu_readings['mag_field.magnetic_field.x'])
max_x = max(Imu_readings['mag_field.magnetic_field.x'])
min_y = min(Imu_readings['mag_field.magnetic_field.y'])
max_y = max(Imu_readings['mag_field.magnetic_field.y'])
x_offset = (min_x + max_x) / 2.0
y_offset = (min_y + max_y) / 2.0
HI_x = []
HI_x.extend((Imu_readings['mag_field.magnetic_field.x'] - x_offset))
HI_y = []
HI_y.extend((Imu_readings['mag_field.magnetic_field.y'] - y_offset))
radius = math.sqrt((HI_x[2000] ** 2) + (HI_y[2000] ** 2))
theta = np.arcsin((HI_y[2000] / radius))
rotate = [[np.cos(theta), np.sin(theta)], [np.sin(-theta), np.cos(theta)]]
v = [HI_x, HI_y]
rotated = np.matmul(rotate, v)
rotated = np.expand_dims(rotated, axis=0)

w = Imu_readings['imu.orientation.w']
x = Imu_readings['imu.orientation.x']
y = Imu_readings['imu.orientation.y']
z = Imu_readings['imu.orientation.z']

# Manual to Euler
roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x ** 2 + y ** 2))
pitch = np.arcsin(2 * (w * y - z * x))
yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))

mag_x = rotated[:, 0]
mag_y = rotated[:, 1]
mag_z = Imu_readings['mag_field.magnetic_field.z']

# Only calculate Yaw
mag_z = mag_z.to_numpy()
mag_z = np.reshape(mag_z, (1, 66488))

# print the size of mag_x, mag_y, mag_z
print('mag_x', mag_x.shape)
print('mag_y', mag_y.shape)
print('mag_z', mag_z.shape)

# Calibrated Yaw
i = mag_z * list(np.sin(roll)) - mag_y * list(np.cos(roll))
j = mag_x * list(np.cos(pitch)) + mag_y * list(np.sin(pitch) * np.sin(roll))
k = mag_z * list(np.sin(pitch) * np.cos(roll))
calib = np.arctan2(i, j + k)
calib_yaw = pandas.Series(max(np.unwrap(calib)))
calib_yaw = calib_yaw * 180 / np.pi

# Non-Calibrated Yaw
mag_x = Imu_readings['mag_field.magnetic_field.x']
mag_y = Imu_readings['mag_field.magnetic_field.y']
mag_z = Imu_readings['mag_field.magnetic_field.z']
raw_x = mag_z * list(np.sin(roll)) - mag_y * list(np.cos(roll))
raw_x = raw_x.squeeze()
y_a = mag_x * list(np.cos(pitch))
y_b = mag_y * list(np.sin(pitch)) * list(np.sin(roll))
y_c = mag_z * list(np.sin(pitch)) * list(np.cos(roll))
y_a = y_a.squeeze()
y_b = y_b.squeeze()
y_c = y_c.squeeze()
raw_y = y_a + y_b + y_c
print('raw_x', raw_x)
print('raw_y', raw_y)
raw_yaw = np.arctan2(raw_x, raw_y)
non_calib_yaw = pandas.Series(np.unwrap(raw_yaw))
non_calib_yaw = non_calib_yaw * 180 / np.pi

# Gyro
gyro = integrate.cumtrapz(Imu_readings['imu.angular_velocity.z'], initial=0)
plt.figure(figsize=(16, 8))
plt.plot(gyro, label='Gyro Integrated Yaw', color='red')
plt.plot(calib_yaw, label='Calibrated Yaw', color='blue')
plt.plot(non_calib_yaw, label='Non-Calibrated Yaw', color='yellow')
plt.legend(loc='upper right')
plt.grid(color='black', linestyle='-', linewidth=0.5)
plt.title('Three Yaw Estimation')
plt.xlabel('Data points (40Hz)')
plt.ylabel('Yaw (degree)')
plt.show()

# Filter
low_pass_filter = signal.filtfilt(*butter(3, 0.1, "lowpass", fs=40, analog=False), calib_yaw)
high_pass_filter = signal.filtfilt(*butter(3, 0.0001, 'highpass', fs=40, analog=False), gyro)
complementary_yaw = 0.98 * (low_pass_filter + high_pass_filter) + 0.02 * calib_yaw

plt.figure(figsize=(16, 8))
plt.plot(calib_yaw, label='Original Yaw')
plt.plot(low_pass_filter, label='Low Pass Filter: Calibrated Yaw', color='red')
plt.plot(high_pass_filter, label='High Pass Filter: Gyro Yaw', color='blue')
plt.plot(complementary_yaw, label='Complementary Yaw', color='yellow')
plt.legend(loc='upper right')
plt.grid(color='black', linestyle='--', linewidth=0.5)
plt.title('Filtered Yaw Estimation')
plt.xlabel('Data points (40Hz)')
plt.ylabel('Yaw (degrees)')
plt.show()

# Comparison between sensor fusion yaw and IMU yaw
a = 0.75
filtered_yaw = np.append([], 0)
for i in range(len(yaw) - 1):
    filtered_yaw = np.append(filtered_yaw, a * (filtered_yaw[i] + high_pass_filter[i + 1] * 0.05)
                             + (1 - a) * low_pass_filter[i + 1])
plt.figure(figsize=(16, 8))
plt.plot(filtered_yaw, label='Fusion Yaw', color='red')
plt.plot(yaw*200, label='IMU Yaw', color='blue')
plt.legend(loc='lower right')
plt.grid(color='black', linestyle='--', linewidth=0.5)
plt.legend(loc='upper right')
plt.xlabel('Data points (40Hz)')
plt.ylabel('Yaw (degrees)')
plt.title('Comparison between sensor fusion yaw and IMU yaw')
plt.show()
