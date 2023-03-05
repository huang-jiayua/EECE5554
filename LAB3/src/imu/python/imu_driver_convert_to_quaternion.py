#!/usr/bin/env python

import rospy
import math
from imu.srv import convert_to_quaternion
import tf


class Quaternion:
    def __init__(self, x=0, y=0, z=0, w=1):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


def handle_convert_to_quaternion(req):
    roll = req.roll
    pitch = req.pitch
    yaw = req.yaw

    quaternion = euler_to_quaternion(roll, pitch, yaw)

    return convert_to_quaternionResponse(
        x=quaternion.x, y=quaternion.y, z=quaternion.z, w=quaternion.w)


def euler_to_quaternion_server():
    rospy.init_node('euler_to_quaternion_server')
    s = rospy.Service('euler_to_quaternion', convert_to_quaternion, handle_euler_to_quaternion)
    rospy.spin()


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr

    return qx, qy, qz, qw


if __name__ == '__main__':
    euler_to_quaternion_server()

