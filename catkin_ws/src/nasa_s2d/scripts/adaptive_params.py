#!/usr/bin/env python

import rospy, rosbag
import dynamic_reconfigure.client

import numpy as np

from nav_msgs.msg import Odometry

class AdaptiveParams:
    """ROS Node that adaptively sends parameter
    updates to visual_mtt based on MAV state"""
    def __init__(self):

        self.sub_odom = rospy.Subscriber('odom', Odometry, self.odom_cb)

        # initialize to -1 so that the first run params are set
        self.counter = -1
        
    def odom_cb(self, msg):

        # Make sure that the altitude is sensible
        z = np.abs(msg.pose.pose.position.z)
        if z == 0:
            return

        # Throttle the service calls
        self.counter += 1
        if np.mod(self.counter, 20):
            return

        # =====================================================================
        # Recursive-RANSAC Parameters

        size_of_object = 1.1 # meters
        dist_to_merge = size_of_object*1.3 # meters

        params = {
                    'tauR': size_of_object/z,
                    'tauR_RANSAC': size_of_object/z,
                    'tau_xpos_abs_diff': dist_to_merge/z,
                    'tau_ypos_abs_diff': dist_to_merge/z,
                 }

        self.update_params('rransac', params)

        # =====================================================================
        # Visual Measurement Frontent (vmtt) Parameters

        min_vel = 0.025 # m/s
        max_vel = 0.6 # m/s

        params = {
                    'minimum_feature_velocity': min_vel/z,
                    'maximum_feature_velocity': max_vel/z,
                    # 'downsize_scale': self.get_downsize_scale(z),
                 }
        
        self.update_params('visual_frontend', params)


    def update_params(self, client_name, params):
        # Dynamic Reconfigure client
        client = dynamic_reconfigure.client.Client(client_name)

        # Update the parameters
        config = client.update_configuration(params)


    def get_downsize_scale(self, h):
        """
        Gain scheduling approach to the downsize scale, based on altitude.

        This helps get tracks at high altitudes, but the values of the
        measurements jump around in the measurement space -- therefore
        it hurts data association and track continuity.
        """

        if h < 20:
            scale = 0.6

        elif h < 60:
            scale = 0.7

        elif h < 100:
            scale = 0.8

        elif h < 140:
            scale = 1.0

        return scale

        

if __name__ == '__main__':
    rospy.init_node('adaptive_params', anonymous=False)

    try:
        obj = AdaptiveParams()

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass