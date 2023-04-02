import bagpy
import math
import csv
import time
import seaborn as sea
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import pandas
from scipy.optimize import fsolve
import scipy.integrate as integrate
from scipy.signal import butter
from scipy import signal

sea.color_palette(as_cmap=True)
plt.rcParams.update({'font.size': 8})

bag = bagreader('/home/huangjiayua/EECE5554/LAB4/src/lab4_driver/data/boston_mini_tour.bag')
data = bag.message_by_topic('/imu')
readings = pandas.read_csv(data)
gpscsv = pandas.read_csv('/home/huangjiayua/EECE5554/LAB4/src/lab4_driver/data/boston_mini_tour-gps.csv')

# check all the keys in gpscsv
# print(gpscsv.keys())

sec = np.double(readings['header.stamp.secs'])
nsec = np.double(readings['header.stamp.nsecs'])
nsec = nsec * 10e-9
imu_time = sec + nsec

imu_acc_x = readings['imu.linear_acceleration.x']
x = np.mean(imu_acc_x)
imu_linear_acc = imu_acc_x - x

difference = []
for i in range(66487):
    difference = np.append(difference, (imu_linear_acc[i + 1] - imu_linear_acc[i]) / 0.025)
print(difference)

velocity_calib = integrate.cumtrapz(imu_linear_acc[1:] - difference, initial=0)
velocity_calib[velocity_calib < 0] = 0
Forward_velocity_raw = integrate.cumtrapz(imu_linear_acc, initial=0)

# GPS Velocity
gps_time = gpscsv['.Header.stamp.secs']
UTM_east = gpscsv['.UTM_easting']
UTM_north = gpscsv['.UTM_northing']
Latitude = gpscsv['.Latitude']
Longitude = gpscsv['.Longitude']
gps_distance = []

plt.figure(figsize=(8, 8))
plt.plot(UTM_east, UTM_north, c='palevioletred')
plt.grid(color='black', linestyle='--', linewidth=0.5)
plt.title('UTM Easting V/S UTM Northing')
plt.xlabel('UTM Easting')
plt.ylabel('UTM Northing')
plt.show()

UTM_north = np.gradient(UTM_north, gps_time)
UTM_east = np.gradient(UTM_east, gps_time)

testVelo = np.sqrt(UTM_north ** 2 + UTM_east ** 2)

plt.figure(figsize=(16, 8))
plt.plot(gps_time, testVelo, label='GPS Velocity', c='red')
# plt.plot(imu_time[1:], velocity_calib, label='Adjusted', c='blueviolet')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.show()

for i in range(1661):
    gps_distance = np.append(gps_distance,
                             math.sqrt(((UTM_north[i + 1] - UTM_north[i]) ** 2) + (UTM_east[i + 1] - UTM_east[i]) ** 2))
gps_vel = gps_distance / gps_time[1:]

# gpscsv_time = gpscsv['time']
# plt.figure(figsize=(16, 8))
# plt.plot(imu_time[1:], velocity_calib / 1000, label='imu Adjusted Velocity', c='red')
# plt.plot(gps_time[1:], gps_vel * 2000, label='GPS adjusted Velocity')
# plt.legend(loc='upper right')
# plt.grid(color='black', linestyle='--', linewidth=0.5)
# plt.title('Forward velocity from imu and GPS after adjustment')
# plt.xlabel('Time (secs)')
# plt.ylabel('Velocity (m/sec)')
# plt.show()

imu_angVel_z = readings['imu.angular_velocity.z']
y_dot = imu_angVel_z[1:] * integrate.cumtrapz(imu_acc_x)
imu_acc_y = readings['imu.linear_acceleration.y']
plt.figure(figsize=(8, 8))
plt.plot(imu_acc_y, label='Raw Y acceleration', c='blueviolet')
plt.plot(y_dot / 1000, label='wX(dot)', c='red')
plt.legend(loc='upper right')
plt.grid(color='black', linestyle='--', linewidth=0.5)
plt.title('Y_observed V/S wX(dot)')
plt.xlabel('Data points (40Hz)')
plt.ylabel('acceleration')
plt.show()

w = readings['imu.orientation.w']
x = readings['imu.orientation.x']
y = readings['imu.orientation.y']
z = readings['imu.orientation.z']

roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
pitch = np.arcsin(2.0 * (w * y - z * x))
yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

fv = np.unwrap(velocity_calib)
rot = (-108 * np.pi / 180)

Ve = np.cos(yaw[1:] + rot) * fv - np.sin(yaw[1:] + rot) * fv
Vn = np.cos(yaw[1:] + rot) * fv + np.sin(yaw[1:] + rot) * fv
Xe = integrate.cumtrapz(Ve)
Xn = integrate.cumtrapz(Vn)

plt.figure(figsize=(8, 8))
plt.plot((Xe / (10 ** 6)) / 2, -Xn / (10 ** 5), c='blueviolet')
plt.grid(color='black', linestyle='--', linewidth=0.5)
plt.title('Trajectory of Vehicle')
plt.xlabel('Xe')
plt.ylabel('Xn')
plt.plot()
plt.show()
