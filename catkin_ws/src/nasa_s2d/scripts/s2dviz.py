#!/usr/bin/env python
# -*- coding: utf-8 -*-
from builtins import range

import copy, threading

import rospy
import tf
from geodesy import utm

import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PolygonStamped, Point32
from mavros_msgs.msg import HomePosition, Waypoint, WaypointList, CommandCode, State
from sensor_msgs.msg import NavSatFix, CameraInfo
from nav_msgs.msg import Odometry
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


def get_target_subs(ns="/", cb=None):
    """Get Subscribers for Moving Targets

    :param ns: namespace to find target topics in
    :param cb: ROS callback for movers
    :return: list of rospy subscribers
    """

    subs = []

    for topic in rospy.get_published_topics(namespace=ns):
        t, type = topic[0], topic[1]

        if type == 'nav_msgs/Odometry':
            subs.append(rospy.Subscriber(t, Odometry, cb, topic))

    return subs

def nice_quat(quaternion):
    Phi = tf.transformations.euler_from_quaternion(quaternion)
    Phi = np.degrees(np.array(Phi))
    return ["{:.2f}".format(x) for x in Phi]

class S2DVIZ:
    """
    S2D Vizualization Management Node.

    Manage the input to `rviz_satellite` and send waypoint markers to
    the S2D rviz visualization.

    """
    def __init__(self):

        # Connect to tf tree
        self.tf = tf.TransformListener()

        # camera information (from the initial, unscaled hardware camera)
        self.cinfo = None

        # current vehicle status
        self.status = State()

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

        # mapping from topic name to marker msg
        self.mover_markers = {}
        self.sub_movers = [] # subscribers to movers, to be found after home position is set

        # List of objects ({'handler': None, 'msg': None}) to be handled once home position is set
        self.msg_queue = []

        # ROS publishers
        self.pub_home = rospy.Publisher('visualization/home', NavSatFix, queue_size=5, latch=True)
        self.pub_mission = rospy.Publisher('visualization/mission', MarkerArray, queue_size=5, latch=True)
        self.pub_ditchsites = rospy.Publisher('visualization/ditch_sites', MarkerArray, queue_size=5, latch=True)
        self.pub_path = rospy.Publisher('visualization/path', MarkerArray, queue_size=5, latch=True)
        self.pub_movers = rospy.Publisher('visualization/movers', MarkerArray, queue_size=1)
        self.pub_frustum = rospy.Publisher('visualization/frustum', PolygonStamped, queue_size=1)

        # ROS subscribers -- be careful! Some of these are referenced below.
        # I probably shouldn't have used numbers (sub*) ¯\_(ツ)_/¯
        self.sub0 = rospy.Subscriber('mavros/home_position/home', HomePosition, self.home_cb)
        self.sub1 = rospy.Subscriber('mavros/global_position/global', NavSatFix, self.globalpos_cb)
        self.sub2 = rospy.Subscriber('mavros/mission/waypoints', WaypointList, self.wp_cb)
        self.sub3 = rospy.Subscriber('mavros/state', State, self.state_cb)
        self.sub4 = rospy.Subscriber('dss/path', WaypointList, self.path_cb)
        self.sub5 = rospy.Subscriber('dss/ditch_sites', DitchSiteList, self.ditchsites_cb)
        self.sub6 = rospy.Subscriber('camera/camera_info', CameraInfo, self.cinfo_cb)


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

        # Try to subscribe to any movers out there
        if not self.sub_movers:
            self.sub_movers = get_target_subs(ns="/targets", cb=self.movers_cb)

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


    def cinfo_cb(self, msg):
        # Just receive a single camera info msg and unregister the pub
        self.cinfo = msg
        # self.sub6.unregister()

        p_ned = self.geolocate_frustum()

        R_ned_to_enu = np.array([[0,1,0],
                                 [1,0,0],
                                 [0,0,-1]
                                ], dtype=np.float64)

        # convert to ENU
        p_enu = R_ned_to_enu.dot(p_ned)

        frustum = PolygonStamped()
        frustum.header.stamp = rospy.Time.now()
        frustum.header.frame_id = self.fixed_frame

        for i in range(p_enu.shape[1]):
            pt = Point32()
            pt.x = p_enu[0,i]
            pt.y = p_enu[1,i]
            pt.z = p_enu[2,i]
            frustum.polygon.points.append(pt)

        self.pub_frustum.publish(frustum)


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

            marker.scale.x = 2
            marker.scale.y = 2
            marker.scale.z = 2

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
            marker.color.a = 0.7 if site.selected else 0.3

            markers.markers.append(marker)

            #
            # Create text with name of ditch site
            #

            txt = copy.deepcopy(marker)
            txt.id = idx*100
            txt.type = Marker.TEXT_VIEW_FACING

            txt.text = site.name
            txt.pose.position.z += txt.scale.z
            txt.scale.z = 4

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
            marker.color.a = 1 if self.status.mode == "AUTO" else 0.3


            markers.markers.append(marker)

        self.pub_mission.publish(markers)

        self.mission_markers = markers
        self.mission_wplist = msg


    def movers_cb(self, msg, args):
        """
        """

        # Extract the topic argument
        topic = args[0]

        # If this mover is not in the list of markers, make a new one
        if topic not in self.mover_markers:
            marker = Marker()

            marker.header.frame_id = self.fixed_frame

            marker.id = len(self.mover_markers)
            # marker.ns = topic
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1

            marker.scale.x = 2
            marker.scale.y = 2
            marker.scale.z = 0.025

            marker.color.r = 1
            marker.color.g = 0
            marker.color.b = 0
            marker.color.a = 0.7

            # If we don't get an update in n second, delete me!
            marker.lifetime = rospy.Duration.from_sec(2)

            # Add it to the dict
            self.mover_markers.update({topic: marker})


        # Otherwise, update the marker time and position (including new ones!)
        if topic in self.mover_markers:
            self.mover_markers[topic].header.stamp = rospy.Time.now()

            self.mover_markers[topic].pose.position.x = msg.pose.pose.position.x
            self.mover_markers[topic].pose.position.y = msg.pose.pose.position.y
            self.mover_markers[topic].pose.position.z = msg.pose.pose.position.z


        # Publish the marker for this mover so that it's lifetime is extended
        markers = MarkerArray()
        markers.markers.append(self.mover_markers[topic])
        try:
            self.pub_movers.publish(markers)
        except:
            pass


    def calculate_lla_diff(self, lat, lon):
        """
            We assume that we are operating away from the boundaries
            are completely inside one UTM zone. Otherwise, something
            like the Haversine-based LLA measurement function can be
            used.
            https://answers.ros.org/question/50763/need-help-converting-lat-long-coordinates-into-meters/?answer=243041#post-id-243041
        """
        
        utm_home = utm.fromLatLong(self.home_position.latitude,
                                    self.home_position.longitude)

        utm_wp = utm.fromLatLong(lat, lon)

        if utm_home.zone is not utm_wp.zone or utm_home.band is not utm_wp.band:
            rospy.logwarn("Near edge of UTM zone -- should use haversine")

        # calculate difference in local coordinates -- meters
        x = utm_home.easting - utm_wp.easting
        y = utm_home.northing - utm_wp.northing

        x = utm_wp.easting - utm_home.easting
        y = utm_wp.northing - utm_home.northing

        return (x, y)

    def geolocate_frustum(self):
        """Geolocate Camera Frustum
        """
        if self.cinfo is None:
            return np.zeros((3,1))

        if not self.tf.frameExists("map_ned") or not self.tf.frameExists("camera"):
            return np.zeros((3,1))

        try:
            position, quaternion = self.tf.lookupTransform("map_ned", "camera", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return np.zeros((3,1))


        #
        # Fiddling with rotation between body and gimbal (i.e., 48-degree mounted camera)
        #

        # print
        # _, q_c2v = self.tf.lookupTransform("map_ned", "camera", rospy.Time(0))
        # print("R_c_to_v: {}".format(nice_quat(q_c2v)))

        # _, q_b2v = self.tf.lookupTransform("map_ned", "base_link_ned", rospy.Time(0))
        # print("R_b_to_v: {}".format(nice_quat(q_b2v)))

        # _, q_g2b = self.tf.lookupTransform("base_link_ned", "gimbal", rospy.Time(0))
        # # axes may be wrong, but since we are only rotating about one axis, it doesn't matter
        # q_g2b = tf.transformations.quaternion_from_euler(0, np.radians(-48), 0, axes='sxyz')
        # print("R_g_to_b: {}".format(nice_quat(q_g2b)))

        # _, q_c2g = self.tf.lookupTransform("gimbal", "camera", rospy.Time(0))
        # print("R_c_to_g: {}".format(nice_quat(q_c2g)))


        # q_final = tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(q_b2v,q_g2b), q_c2g)
        # print("My R_c_to_v: {}".format(nice_quat(q_final)))
        # print

        # quaternion = q_final

        #
        # Define normalized image plane (nip) points we are interested in
        #

        # extract from camera calibration matrix
        fx_px = self.cinfo.K[0]
        cx_px = self.cinfo.K[2]
        fy_px = self.cinfo.K[4]
        cy_px = self.cinfo.K[5]

        # pixel corners of the image
        p_px = np.array(
            [[0,0],
             [self.cinfo.width,0],
             [self.cinfo.width,self.cinfo.height],
             [0,self.cinfo.height]
            ], dtype=np.float64).T

        # convert to homogeneous normalized image plane coordinates (camera frame)
        XX = (p_px[0,:] - cx_px)/fx_px
        YY = (p_px[1,:] - cy_px)/fy_px
        p_im_h = np.vstack( (XX, YY, np.ones((1,p_px.shape[1]))) )

        # construct normalized line-of-sight vectors (camera frame)
        ellhat = p_im_h / np.linalg.norm(p_im_h, axis=0)

        #
        # Geolocate!
        #

        # rotation matrix from quaternion
        R_c_to_v = tf.transformations.quaternion_matrix(quaternion)[0:3,0:3]

        # rotate from camera frame into vehicle frame
        ellhat_v = R_c_to_v.dot(ellhat)

        # calculate cosine between k_v and ellhat_v
        cos_psi = ellhat_v[2]

        # UAV height AGL
        h = -position[2]

        # 1xN vector of ranges
        L = h / cos_psi

        # make an estimate of the full LOS vector
        ell_v = L*ellhat_v

        # make position a numpy column vector
        p_uav = np.array([position], dtype=np.float64).T

        # location of pixel points given a flat earth
        p_w = p_uav + ell_v

        return p_w


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