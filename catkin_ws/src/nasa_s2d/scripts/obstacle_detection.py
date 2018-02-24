#!/usr/bin/env python
from builtins import range

import copy

import rospy
import tf
from geodesy import utm

import numpy as np

from sensor_msgs.msg import NavSatFix
from visual_mtt.msg import Tracks, Track

class Node:
    """
    Obstacle Detection ROS node.

    Using the geolocated output of Visual MTT, determine which tracks
    are actionable. Then, calculate the lat/lon of those targets (intruders)
    and send to the Ditch Site Selector through the dss_comms node.

    Assumptions:
        - The `map` frame has its origin at the home position of the Pixhawk
        - Incoming measurements (tracks3d) are w.r.t the ENU map origin
            (i.e., the header of tracks3d has frame_id="map")
        - Obstacle lat/lon are computed on a place (flat earth)
    """
    def __init__(self):

        # Connect to tf tree
        self.listener = tf.TransformListener()

        # Keep track of the most recent UAV GPS fix as a UTMPoint
        self.utm_pt = utm.UTMPoint()

        # ROS subscribers
        self.sub0 = rospy.Subscriber('uav_gps_fix', NavSatFix, self.gps_cb)
        self.sub1 = rospy.Subscriber('tracks3d', Tracks, self.tracks_cb)

        # ROS publishers
        self.pub_obstacles = rospy.Publisher('obstacles', Tracks, queue_size=1)


    def gps_cb(self, msg):
        # Convert sensor_msgs/NavSatFix to utm.UTMPoint
        self.utm_pt = utm.fromMsg(msg)

        
    def tracks_cb(self, msg):
        # If there are no tracks to process, just bail!
        if not msg.tracks:
            return

        # Check that we have a global point-of-reference for the UAV
        if not self.utm_pt.valid():
            rospy.logwarn_throttle(5, "UTM position of UAV not set or is invalid")
            return

        # Attempt to receive the latest body position in the map frame (ENU)
        try:
            (uav_enu, rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(5, "Could not get body position from tf")
            return

        # Get most recent UAV position as UTM
        uav_easting = self.utm_pt.easting
        uav_northing = self.utm_pt.northing

        # Create new msg for obstacle tracks in LLA
        obstacles = Tracks()

        # Each track is defined in the ENU map frame (see assumptions)
        for track in msg.tracks:

            # Calculate the offset vector from UAV to obstacle (ENU coords)
            uav_obs_x = track.position.x - uav_enu[0]
            uav_obs_y = track.position.y - uav_enu[1]

            # Use the ENU offset vector to find the obstacles UTM coords
            obs_easting = uav_easting + uav_obs_x
            obs_northing = uav_northing + uav_obs_y

            # Convert UTM to LLA
            obs_utm_pt = copy.deepcopy(self.utm_pt)
            obs_utm_pt.easting = obs_easting
            obs_utm_pt.northing = obs_northing
            obs_geo_pt = obs_utm_pt.toMsg()

            # Use the original track message
            obstacle = track

            # Change the 3D position to LLA
            obstacle.position.x = obs_geo_pt.longitude
            obstacle.position.y = obs_geo_pt.latitude
            obstacle.position.z = 0 # flat earth assumption

            obstacles.tracks.append(obstacle)

        self.pub_obstacles.publish(obstacles)
        

if __name__ == '__main__':
    rospy.init_node('dss_comms', anonymous=False)

    try:
        obj = Node()

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass