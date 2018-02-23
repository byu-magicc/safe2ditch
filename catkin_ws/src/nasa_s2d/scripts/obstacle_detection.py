#!/usr/bin/env python
from builtins import range

import rospy
from tf import TransformListener
from geodesy import utm

import numpy as np

from sensor_msgs.msg import NavSatFix
from visual_mtt.msg import Tracks

class Node:
    """
    Obstacle Detection ROS node.

    Using the geolocated output of Visual MTT, determine which tracks
    are actionable. Then, calculate the lat/lon of those targets (intruders)
    and send to the Ditch Site Selector through the dss_comms node.

    Assumptions:
        - The `map` frame has its origin at the home position of the Pixhawk
        - Incoming measurements (tracks3d) are w.r.t the map origin
            (i.e., the header of tracks3d has frame_id="base_link")
    """
    def __init__(self):

        # Connect to tf tree
        self.tf = TransformListener()

        # Keep track of the most recent UAV GPS fix as a UTMPoint
        self.utm_pt = utm.UTMPoint()

        # ROS subscribers
        self.sub0 = rospy.Subscriber('uav_gps_fix', NavSatFix, self.gps_cb)
        self.sub1 = rospy.Subscriber('tracks3d', Tracks, self.tracks_cb)


    def gps_cb(self, msg):
        # Convert sensor_msgs/NavSatFix to utm.UTMPoint
        self.utm_pt = utm.fromMsg(msg)

        
    def tracks_cb(self, msg):
        # Get most recent UAV position as UTM
        uav_easting = self.utm_pt.easting
        uav_northing = self.utm_pt.northing


        

if __name__ == '__main__':
    rospy.init_node('dss_comms', anonymous=False)

    try:
        obj = Node()

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass