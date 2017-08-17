#!/usr/bin/env python

import rospy, rosbag
import dynamic_reconfigure.client

import numpy as np

from matplotlib import pyplot as plt
# from matplotlib import animation

from std_srvs.srv import Trigger
from visual_mtt.msg import Tracks, Track

class LifetimeCounter:
    """ROS Node that listens to R-RANSAC tracks and
    creates a bar chatrt of each track and its lifetime counter"""
    def __init__(self):

        self.sub_tracks = rospy.Subscriber('tracks', Tracks, self.tracks_cb)

        # ROS service to show matplotlib histogram
        self.srv_plot = rospy.Service('~show_lifetime_barchart', Trigger, self.plot_srv)

        # Dictionary to store lifetimes for each id with the following form:
        #   {
        #       ...
        #       47: 10
        #       ...
        #   }
        self.track_lifetimes = {}

        
    def tracks_cb(self, msg):
        for track in msg.tracks:
            self.track_lifetimes[track.id] = track.lifetime


    def plot_srv(self, req):
        self.show_barchart()


    def show_barchart(self):
        # Create a list of track_ids and a list of the corresponding track's lifetime
        tmp = zip(*[(track_id, self.track_lifetimes[track_id]) for track_id in self.track_lifetimes])
        ids, Ts = tmp[0], tmp[1]


        fig, ax = plt.subplots()

        width = 0.9

        bars = ax.bar(np.array(ids) - width/2, Ts, width, color='blue', alpha=0.4)

        plt.xlabel('Track ID')
        plt.ylabel('Lifetime')
        plt.title('Lifetime of Each R-RANSAC Track')
        plt.grid(True)

        plt.show()
        plt.close()
        

if __name__ == '__main__':
    rospy.init_node('lifetime_counter', anonymous=False)

    try:
        obj = LifetimeCounter()

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass