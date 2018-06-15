#!/usr/bin/env python
import copy

import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import HomePosition

from geodesy import utm
""" Geodesy Package
Note that we should technically not use this package, as it
converts WGS84 (LLA) to UTM (a local ENU chart). If distances
were large or on the edge of a UTM Zone/Band, there would be considerable
error by working with easting and northing (in meters)
and then converting to LLA.

But, gotta graduate somehow, right?
"""

class MoverLLA(object):
    """Mover LLA

        ROS class that converts simulation movers odometry to
        latitude, longitude, altitude (LLA).

        Doing this creates the same interface for hardware
        (using RTK as ground truth) or in simulation.
    """
    def __init__(self):
        super(MoverLLA, self).__init__()

        # origin, from LLA to UTM
        self.origin = None

        # ENU coordinates, from odometry
        self.easting = None
        self.northing = None
        
        # ROS connections
        self.sub_odom = rospy.Subscriber('state', Odometry, self.odom_cb)
        self.sub_origin = rospy.Subscriber('/mavros/home_position/home', HomePosition, self.home_cb)

        self.pub_fix = rospy.Publisher('fix', NavSatFix, queue_size=1)


    def odom_cb(self, msg):
        self.easting = msg.pose.pose.position.x
        self.northing = msg.pose.pose.position.y


    def home_cb(self, msg):
        self.origin = utm.fromLatLong(latitude=msg.geo.latitude, longitude=msg.geo.longitude)


    def publish(self):
        # If we don't have all the pieces, bail
        if self.origin is None or self.easting is None or self.northing is None:
            return

        # Create a temporary UTM object to add to
        pt = copy.deepcopy(self.origin)

        # Add the current easting and northing to the origin
        pt.easting += self.easting
        pt.northing += self.northing

        # Create a "fix" of the current target's LLA
        msg = NavSatFix()
        msg.header.stamp = rospy.Time.now()
        msg.latitude = pt.toMsg().latitude
        msg.longitude = pt.toMsg().longitude
        self.pub_fix.publish(msg)


if __name__ == '__main__':
    rospy.init_node('mover_lla', anonymous=True)

    pub_rate = rospy.get_param('~pub_rate', 5)
    
    try:
        mover = MoverLLA()

        rate = rospy.Rate(pub_rate)
        while not rospy.is_shutdown():
            # Publish a fix
            mover.publish()

            rate.sleep()

    except rospy.ROSInterruptException:
        pass