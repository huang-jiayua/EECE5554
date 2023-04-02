#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import utm
import serial
import sys
import numpy
from imu_driver.msg import *
from imu_driver.msg import Vectornav
from imu_driver.srv import convert_to_quaternion, convert_to_quaternionResponse


def euler_to_quaternion(roll, pitch, yaw):
    rospy.wait_for_service('euler_to_quaternion')
    try:
        convert = rospy.ServiceProxy('euler_to_quaternion', convert_to_quaternion)
        response = convert(roll, pitch, yaw)
        return response.x, response.y, response.z, response.w
    except rospy.ServiceException as e:
        print("Service call failed: ", e)


def driver():
    pub = rospy.Publisher('imu', Vectornav, queue_size=10)
    rospy.init_node('drive', anonymous=True)
    rate = rospy.Rate(40)
    msg = Vectornav()

    args = rospy.myargv(argv=sys.argv)

    nextline = serial.Serial(rospy.get_param('~port', args[1]),
                             rospy.get_param('~baudrate', 115200))
                             
    # Change the rate of output to 40Hz
    nextline.write(b'$VNWRG,07,40*XX\r\n')

    while not rospy.is_shutdown():
        rawData = str(nextline.readline())

        if "VNYMR" in rawData:
            data = str(rawData).split(",")
            print(data)

            yaw = float(data[1])
            pitch = float(data[2])
            roll = float(data[3])
            
            # change yaw pitch roll from degree to radian
            yaw = yaw * numpy.pi / 180
            pitch = pitch * numpy.pi / 180
            roll = roll * numpy.pi / 180
            
            mag_x = float(data[4])
            mag_y = float(data[5])
            mag_z = float(data[6])
            acc_x = float(data[7])
            acc_y = float(data[8])
            acc_z = float(data[9])
            gyro_x = float(data[10])
            gyro_y = float(data[11])
            gyro_z = float(data[12][0:9])

            msg.imu.orientation.x, msg.imu.orientation.y, msg.imu.orientation.z, msg.imu.orientation.w = euler_to_quaternion(
                roll, pitch, yaw)

            msg.imu.linear_acceleration.x = acc_x
            msg.imu.linear_acceleration.y = acc_y
            msg.imu.linear_acceleration.z = acc_z
            msg.imu.angular_velocity.x = gyro_x
            msg.imu.angular_velocity.y = gyro_y
            msg.imu.angular_velocity.z = gyro_z
            msg.mag_field.magnetic_field.x = mag_x
            msg.mag_field.magnetic_field.y = mag_y
            msg.mag_field.magnetic_field.z = mag_z
            
            msg.VNYMR = rawData
            
            pub.publish(msg)


if __name__ == '__main__':
    driver()

