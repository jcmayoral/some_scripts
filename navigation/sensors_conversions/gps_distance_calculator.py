#!/usr/bin/python

#Method from https://www.movable-type.co.uk/scripts/latlong.html
from sensor_msgs.msg import NavSatFix
import rospy
import math


class GPSTool:
    def __init__(self):
        rospy.init_node("gps_conversions")
        self.init_position = NavSatFix()
        self.is_not_initialize = True
        self.distance_tolerance = 10
        self.R = 6371000
        rospy.Subscriber("/android/fix", NavSatFix, self.gps_cb)
        rospy.spin()

    def gps_cb(self, msg):
        if self.is_not_initialize:
            self.init_position = msg
            self.is_not_initialize = False

        long1 = self.init_position.longitude
        long2 = msg.longitude

        lat1 = self.init_position.latitude
        lat2 = msg.latitude

        #To radians
        d_lat = (lat2-lat1)
        d_lon = (long2-long1)

        a = math.pow(math.sin(d_lat/2),2) + math.cos(lat1) * math.cos(lat2)  *math.pow(math.sin(d_lon/2),2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        d = self.R * c
        rospy.loginfo("Distance %f", d)


if __name__ == '__main__':
    GPSTool()
