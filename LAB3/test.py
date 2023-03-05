#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import utm
import serial
import sys
import numpy
from imu_driver.msg import *
from imu_driver.msg import imu_msg
from imu_driver.srv import convert_to_quaternion, convert_to_quaternionResponse


def driver():
    pub = rospy.Publisher('imu', imu_msg, queue_size=10)
    rospy.init_node('drive', anonymous=True)
    msg = imu_msg()

    args = rospy.myargv(argv=sys.argv)

    nextline = serial.Serial(rospy.get_param('~port', args[1]),
                             rospy.get_param('~baudrate', 115200))

    while not rospy.is_shutdown():
        rawData = str(nextline.readline())

        if "VNYMR" in rawData:
            data = str(rawData).split(",")
            print(data)

            yaw = float(data[1])
            pitch = float(data[2])
            roll = float(data[3])
            mag_x = float(data[4])
            mag_y = float(data[5])
            mag_z = float(data[6])
            acc_x = float(data[7])
            acc_y = float(data[8])
            acc_z = float(data[9])
            gyro_x = float(data[10])
            gyro_y = float(data[11])
            gyro_z = float(data[12])

            msg.yaw = yaw
            msg.pitch = pitch
            msg.roll = roll
            msg.mag_x = mag_x
            msg.mag_y = mag_y
            msg.mag_z = mag_z
            msg.acc_x = acc_x
            msg.acc_y = acc_y
            msg.acc_z = acc_z
            msg.gyro_x = gyro_x
            msg.gyro_y = gyro_y
            msg.gyro_z = gyro_z

            pub.publish(msg)


if __name__ == '__main__':
    driver()
