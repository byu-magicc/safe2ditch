#!/usr/bin/env python
from builtins import range

import copy

import rospy
import tf
from geodesy import utm

import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from mavros_msgs.msg import HomePosition, Waypoint, WaypointList, CommandCode, State
from sensor_msgs.msg import NavSatFix
from nasa_s2d.msg import DitchSiteList


def blended_color(x):
    """Create an RGB triplet for a blended Red to Green color
    :param x: value from [0 1], 0 is red, 1 is green
    :return: (r, g, b)
    """
    r = 2*(1-x)
    g = 2*x
    b = 0
    return (r, g, b)


class S2DVIZ:
    """
    S2D Vizualization Management Node.

    Manage the input to `rviz_satellite` and send waypoint markers to
    the S2D rviz visualization.

    """
    def __init__(self):

        # Connect to tf tree
        self.listener = tf.TransformListener()

        # current vehicle status
        self.status = None

        # Keep track of the vehicle's home position
        self.home_position = None

        # fixed frame used in rviz
        self.fixed_frame = 'map'

        # current mission marker array message
        self.mission_markers = None
        self.mission_wplist = None

        # current ditch sites marker array message
        self.ds_markers = None

        # current path marker array message
        self.path_markers = None
        self.path_wplist = None

        # List of objects ({'handler': None, 'msg': None}) to be handled once home position is set
        self.msg_queue = []

        # ROS subscribers
        self.sub0 = rospy.Subscriber('mavros/home_position/home', HomePosition, self.home_cb)
        self.sub1 = rospy.Subscriber('mavros/global_position/global', NavSatFix, self.globalpos_cb)
        self.sub2 = rospy.Subscriber('mavros/mission/waypoints', WaypointList, self.wp_cb)
        self.sub3 = rospy.Subscriber('mavros/state', State, self.state_cb)
        self.sub4 = rospy.Subscriber('dss/path', WaypointList, self.path_cb)
        self.sub5 = rospy.Subscriber('dss/ditch_sites', DitchSiteList, self.ditchsites_cb)

        # ROS publishers
        self.pub_home = rospy.Publisher('visualization/home', NavSatFix, queue_size=5, latch=True)
        self.pub_mission = rospy.Publisher('visualization/mission', MarkerArray, queue_size=5, latch=True)
        self.pub_ditchsites = rospy.Publisher('visualization/ditch_sites', MarkerArray, queue_size=5, latch=True)
        self.pub_path = rospy.Publisher('visualization/path', MarkerArray, queue_size=5, latch=True)


    def publish_home(self, msg):
        """Publish Home

        relay function that uses the publisher to send msg. Let's us 
        service the message queue if necessary.
        """

        # save home NavSatFix
        self.home_position = msg

        # Are there any messages to service?
        if self.msg_queue:
            for q in self.msg_queue:
                q['handler'](q['msg'])

            # clear the message queue
            self.msg_queue = []

        self.pub_home.publish(msg)


    def state_cb(self, msg):
        old = self.status

        self.status = msg

        if old is not None and msg.mode != old.mode:
            if self.mission_wplist is not None:
                self.wp_cb(self.mission_wplist)

            if self.path_wplist is not None:
                self.path_cb(self.path_wplist)


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
        self.publish_home(msg)
        self.sub1.unregister()



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

        self.publish_home(home)


    def path_cb(self, msg):

        if self.should_wait(self.path_cb, msg):
            return

        # Remove old path markers so that we can update below
        if self.path_markers is not None:
            for marker in self.path_markers.markers:
                marker.action = Marker.DELETE

            self.pub_path.publish(self.path_markers)
        

        markers = MarkerArray()

        for idx, wp in enumerate(msg.waypoints):

            # Convert LLA offset from center of ditch site to home position to meters
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
            marker.color.a = 1 if self.status.mode == "GUIDED" else 0.5

            markers.markers.append(marker)

        self.pub_path.publish(markers)

        self.path_markers = markers
        self.path_wplist = msg


    def ditchsites_cb(self, msg):

        if self.should_wait(self.ditchsites_cb, msg):
            return

        # Remove old ditch site markers so that we can update below
        if self.ds_markers is not None:
            for marker in self.ds_markers.markers:
                marker.action = Marker.DELETE

            self.pub_ditchsites.publish(self.ds_markers)
        

        markers = MarkerArray()

        for idx, site in enumerate(msg.ditch_sites):

            idx += 1

            # Convert LLA offset from center of ditch site to home position to meters
            (x, y) = self.calculate_lla_diff(site.position.latitude, site.position.longitude)

            #
            # Create circle for ditch site
            #

            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = self.fixed_frame

            marker.id = idx
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = site.position.altitude

            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1

            marker.scale.x = site.radius*2
            marker.scale.y = site.radius*2
            marker.scale.z = 0.05

            r, g, b = blended_color(site.reliability/100.0)
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 0.6 if site.selected else 0.3

            markers.markers.append(marker)

            #
            # Create text with name of ditch site
            #

            txt = copy.deepcopy(marker)
            txt.id = idx*100
            txt.type = Marker.TEXT_VIEW_FACING

            txt.text = site.name
            txt.pose.position.z += txt.scale.z
            txt.scale.z = 2

            txt.color.r = 1
            txt.color.g = 1
            txt.color.b = 1
            txt.color.a = 1 if site.selected else 0.5

            markers.markers.append(txt)

        self.pub_ditchsites.publish(markers)

        self.ds_markers = markers


    def wp_cb(self, msg):

        if self.should_wait(self.wp_cb, msg):
            return

        # Remove old mission markers so that we can update below
        if self.mission_markers is not None:
            for marker in self.mission_markers.markers:
                marker.action = Marker.DELETE

            self.pub_mission.publish(self.mission_markers)


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
            marker.color.a = 1 if self.status.mode == "AUTO" else 0.5


            markers.markers.append(marker)

        self.pub_mission.publish(markers)

        self.mission_markers = markers
        self.mission_wplist = msg


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


    def should_wait(self, handler, msg):
        """Check to see if this msg should be processed

        Since we are relying on the home position to position everyhing,
        if the home position has not yet been set, then this msg should
        be processed later. Put it on the queue!

        :param handler: function callback to handle message
        :param msg: message to be handled
        :return: boolean
        """

        if self.home_position is None:

            q = {
                'handler': handler,
                'msg': msg
            }

            self.msg_queue.append(q)

            return True


        # No need to wait!
        return False
        

if __name__ == '__main__':
    rospy.init_node('s2dviz', anonymous=False)

    try:
        obj = S2DVIZ()

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass