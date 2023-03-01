#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import utm
import serial
import sys
from gnss_driver.msg import *
from gnss_driver.msg import gnss_msg


def driver():
    pub = rospy.Publisher('gnss', gnss_msg, queue_size=10)
    rospy.init_node('drive', anonymous=True)
    msg = gnss_msg()

    args = rospy.myargv(argv=sys.argv)

    nextline = serial.Serial(rospy.get_param('~port', args[1]),
                        rospy.get_param('~baudrate', 4800))

    while not rospy.is_shutdown():
        rawData = str(nextline.readline())

        if "$GNGGA" in str(rawData):
            data = str(rawData).split(",")
            print(data)
            time = float(data[1])
            hour = int(time / 10000)
            second = time % 100
            minute = time % 10000 - second
            IncorporatedSecond = int(hour * 3600 + minute * 60 + second)
            nanosecond = int((IncorporatedSecond * (10 ** 9)) % (10 ** 9))

            rawLat = float(data[2])
            lat1 = int(rawLat / 100)
            lat2 = rawLat % 100
            if data[3] == 'S':
                lat_converted = 0 - float(lat1 + lat2 / 60)
            else:
                lat_converted = float(lat1 + lat2 / 60)

            rawLong = float(data[4])
            long1 = int(rawLong / 100)
            long2 = rawLong % 100
            long_converted = float(long1 + long2 / 60)
            if data[5] == 'W':
                long_converted = float(long1 + long2 / 60)
            else:
                long_converted = 0 - float(long1 + long2 / 60)

            alt = float(data[9])
            
            fix = float(data[6])

            utmLatLon = utm.from_latlon(lat_converted, long_converted)
            print(f'UTM_East, UTM_north, Zone, Letter: {utmLatLon}, Fix: {fix}')
            # time.sleep(1)
            # msg.header.stamp = rospy.Time.from_sec(utc_final)
            msg.header.stamp.secs = IncorporatedSecond
            msg.header.stamp.nsecs = nanosecond
            msg.header.frame_id = 'GNSS1_Frame'
            msg.Latitude = lat_converted
            msg.Longitude = long_converted
            msg.Altitude = alt
            msg.Fix = fix
            msg.UTM_easting = utmLatLon[0]
            msg.UTM_northing = utmLatLon[1]
            msg.Zone = utmLatLon[2]
            msg.Letter = utmLatLon[3]
            pub.publish(msg)
            # rate.sleep()


if __name__ == '__main__':
    driver()
