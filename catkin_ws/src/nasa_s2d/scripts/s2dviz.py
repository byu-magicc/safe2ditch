#!/usr/bin/env python
from builtins import range

import copy

import rospy
import tf
from geodesy import utm

import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from mavros_msgs.msg import HomePosition, Waypoint, WaypointList, CommandCode
from sensor_msgs.msg import NavSatFix

class S2DVIZ:
    """
    S2D Vizualization Management Node.

    Manage the input to `rviz_satellite` and send waypoint markers to
    the S2D rviz visualization.

    """
    def __init__(self):

        # Connect to tf tree
        self.listener = tf.TransformListener()

        # Keep track of the vehicle's home position
        self.home_position = None

        # fixed frame used in rviz
        self.fixed_frame = 'map'

        # current mission marker array message
        self.markers = None

        # ROS subscribers
        self.sub0 = rospy.Subscriber('mavros/home_position/home', HomePosition, self.home_cb)
        self.sub1 = rospy.Subscriber('mavros/global_position/global', NavSatFix, self.globalpos_cb)
        self.sub2 = rospy.Subscriber('mavros/mission/waypoints', WaypointList, self.wp_cb)

        # ROS publishers
        self.pub_home = rospy.Publisher('visualization/home', NavSatFix, queue_size=1, latch=True)
        self.pub_mission = rospy.Publisher('visualization/mission', MarkerArray, queue_size=1, latch=True)


    def globalpos_cb(self, msg):
        # If a home position has not yet been set
        # (which happens if the Pixhawk has not yet fully booted)
        # then temporarily use the first valid NavSatFix message as the home.
        
        # If we already have a home position, just unsubscribe and bail
        if self.home_position is not None:
            self.sub1.unregister()
            return

        # Make sure that we have valid data (for the sake of simulation)
        if msg.latitude == 0 or msg.longitude == 0:
            return

        # Send a temporary home position and unsubscribe
        self.pub_home.publish(msg)
        self.sub1.unregister()
        self.home_position = msg



    def home_cb(self, msg):
        # Latch the home position for robustness of `rviz_satellite` aerial
        # imagery plugin. Note that `mavros/home_position/home` is a latched
        # topic, but if you split a bag after the topic was originally sent,
        # then that bag will never emit a home message. To mitigate this,
        # see `globalpos_cb`, which simply latches the first valid NatSatFix
        # message it sees.

        home = NavSatFix()
        home.header.stamp = rospy.Time.now()
        home.header.frame_id = self.fixed_frame
        home.latitude = msg.geo.latitude
        home.longitude = msg.geo.longitude

        self.pub_home.publish(home)

        # save home NavSatFix
        self.home_position = home

        
    def wp_cb(self, msg):

        # Remove old mission markers so that we can update below
        if self.markers is not None:
            for marker in self.markers.markers:
                marker.action = Marker.DELETE

            self.pub_mission.publish(self.markers)
        

        markers = MarkerArray()

        for idx, wp in enumerate(msg.waypoints):

            # We only care about mission items
            if wp.command is not CommandCode.NAV_WAYPOINT:
                continue

            # Ignore mission items that are not in the proper frame
            if wp.frame is not Waypoint.FRAME_GLOBAL_REL_ALT:
                continue

            # Convert LLA offset from waypoint to home position to meters
            (x, y) = self.calculate_lla_diff(wp.x_lat, wp.y_long)

            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = self.fixed_frame

            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = wp.z_alt

            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1

            marker.scale.x = 3
            marker.scale.y = 3
            marker.scale.z = 3

            if msg.current_seq > idx:
                (r, g, b) = 0, 0, 1
            elif msg.current_seq == idx:
                (r, g, b) = 0, 1, 0
            else:
                (r, g, b) = 1, 1, 0

            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0


            markers.markers.append(marker)

        self.pub_mission.publish(markers)

        self.markers = markers


    def calculate_lla_diff(self, lat, lon):
        """
            We assume that we are operating away from the boundaries
            are completely inside one UTM zone. Otherwise, something
            like the Haversine-based LLA measurement function can be
            used.
        """
        
        utm_home = utm.fromLatLong(self.home_position.latitude,
                                    self.home_position.longitude)

        utm_wp = utm.fromLatLong(lat, lon)

        if utm_home.zone is not utm_wp.zone or utm_home.band is not utm_wp.band:
            raise NotImplementedError("Near edge of UTM zone -- should use haversine")

        # calculate difference in local coordinates -- meters
        x = utm_home.easting - utm_wp.easting
        y = utm_home.northing - utm_wp.northing

        x = utm_wp.easting - utm_home.easting
        y = utm_wp.northing - utm_home.northing

        return (x, y)
        

if __name__ == '__main__':
    rospy.init_node('s2dviz', anonymous=False)

    try:
        obj = S2DVIZ()

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass