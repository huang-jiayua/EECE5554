import statistics as sta
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import pandas

plt.rcParams.update({'font.size': 8})

bag = bagreader('/home/huangjiayua/catkin_ws_new/src/imu/data/myStationary.bag')
data = bag.message_by_topic('/imu')
readings = pandas.read_csv(data)

# Switch back from quaternion to Euler angles
q = readings[['IMU.orientation.w', 'IMU.orientation.x', 'IMU.orientation.y', 'IMU.orientation.z']].values
q *= np.pi / 180
roll, pitch, yaw = np.degrees(np.apply_along_axis(
    lambda x: np.arctan2(2.0 * (x[0] * x[1] + x[2] * x[3]), 1.0 - 2.0 * (x[1] * x[1] + x[2] * x[2])), 1, q)), \
    np.degrees(np.apply_along_axis(lambda x: np.arcsin(2.0 * (x[0] * x[2] - x[3] * x[1])), 1, q)), \
    np.degrees(np.apply_along_axis(
        lambda x: np.arctan2(2.0 * (x[0] * x[3] + x[1] * x[2]), 1.0 - 2.0 * (x[2] * x[2] + x[3] * x[3])), 1, q))
print(roll, readings['IMU.orientation.w'])

readings[['Time', 'IMU.angular_velocity.x', 'IMU.angular_velocity.y', 'IMU.angular_velocity.z',
          'IMU.linear_acceleration.x', 'IMU.linear_acceleration.y', 'IMU.linear_acceleration.z',
          'MagField.magnetic_field.x', 'MagField.magnetic_field.y', 'MagField.magnetic_field.z']] -= readings.min()

# roll, pitch, yaw
print('Mean & Standard Deviation of RPY:')
print('mean = ', sta.mean(roll))
print('mean = ', sta.mean(pitch))
print('mean = ', sta.mean(yaw))
print('standard deviation = ', sta.stdev(roll))
print('standard deviation = ', sta.stdev(pitch))
print('standard deviation = ', sta.stdev(yaw))

# Angular Velocity
print('Mean & Standard Deviation of Angular Velocity:')
for i in ['IMU.angular_velocity.x', 'IMU.angular_velocity.y', 'IMU.angular_velocity.z']:
    print('mean = ', readings[i].mean())
print('standard deviation = ', readings[i].std())

# Line Acceleration
print('Mean & Standard Deviation of Linear Acceleration:')
for i in ['IMU.linear_acceleration.x', 'IMU.linear_acceleration.y', 'IMU.linear_acceleration.z']:
    print('mean = ', readings[i].mean())
print('standard deviation = ', readings[i].std())

# Magnetic Field
print('Mean & Standard Deviation of Magnetic Field:')
for i in ['MagField.magnetic_field.x', 'MagField.magnetic_field.y', 'MagField.magnetic_field.z']:
    print('mean = ', readings[i].mean())
print('standard deviation = ', readings[i].std())


# create three separate figures each with 3x1 subplots
fig1, axs1 = plt.subplots(3, 1, figsize=(10, 18))
fig2, axs2 = plt.subplots(3, 1, figsize=(10, 18))
fig3, axs3 = plt.subplots(3, 1, figsize=(10, 18))

# set the titles and units for each sensor type
titles = {
    'IMU.linear_acceleration': 'Linear Acceleration',
    'MagField.magnetic_field': 'Magnetic Field',
    'Orientation': 'Orientation'
}
units = {
    'IMU.linear_acceleration': 'm/s\u00b2',
    'MagField.magnetic_field': 'Gauss',
    'Orientation': 'Radians'
}

# plot the data for accelerometer
for i, axis in enumerate(['x', 'y', 'z']):
    axs1[i].plot(readings['Time'], readings[f'IMU.linear_acceleration.{axis}'])
    axs1[i].set_xlabel('Time (Seconds)')
    axs1[i].set_ylabel(f'{titles["IMU.linear_acceleration"]}_{axis.upper()} ({units["IMU.linear_acceleration"]})')
    axs1[i].set_title(f'Time vs {titles["IMU.linear_acceleration"]}_{axis.upper()}')

# plot the data for magnetometer
for i, axis in enumerate(['x', 'y', 'z']):
    axs2[i].plot(readings['Time'], readings[f'MagField.magnetic_field.{axis}'])
    axs2[i].set_xlabel('Time (Seconds)')
    axs2[i].set_ylabel(f'{titles["MagField.magnetic_field"]}_{axis.upper()} ({units["MagField.magnetic_field"]})')
    axs2[i].set_title(f'Time vs {titles["MagField.magnetic_field"]}_{axis.upper()}')

# plot the data for orientation
axs3[0].plot(readings['Time'], roll)
axs3[0].set_xlabel('Time (Seconds)')
axs3[0].set_ylabel(f'{titles["Orientation"]}_Roll ({units["Orientation"]})')
axs3[0].set_title(f'Time vs {titles["Orientation"]}_Roll')

axs3[1].plot(readings['Time'], pitch)
axs3[1].set_xlabel('Time (Seconds)')
axs3[1].set_ylabel(f'{titles["Orientation"]}_Pitch ({units["Orientation"]})')
axs3[1].set_title(f'Time vs {titles["Orientation"]}_Pitch')

axs3[2].plot(readings['Time'], yaw)
axs3[2].set_xlabel('Time (Seconds)')
axs3[2].set_ylabel(f'{titles["Orientation"]}_Yaw ({units["Orientation"]})')
axs3[2].set_title(f'Time vs {titles["Orientation"]}_Yaw')

plt.rcParams.update({'font.size': 22})

# display the three figures
plt.show(fig1)
plt.show(fig2)
plt.show(fig3)
