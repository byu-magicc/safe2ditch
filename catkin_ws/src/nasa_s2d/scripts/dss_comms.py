#!/usr/bin/env python
from builtins import range

import rospy

import numpy as np

import zmq

class DSSComms:
    """
    Ditch Site Selector Communications handler.
    """

    # Topic IDs for message types
    TOPIC_INTRUDER = 1000

    def __init__(self, port):

        self.port = port
        self.socket = zmq.Context().socket(zmq.PUB)
        self.socket.bind("tcp://*:{}".format(self.port))
        print("Socket on port {} bind complete".format(self.port))


    def send_intruders(self, lat, lon):
        # message contains number of intruders followed by lat/lon for each
        message = ""
        # for i in range(len(self.intruder_lat)):
        #     message = message + "," + str(self.intruder_lat[i]) + "," + str(self.intruder_lon[i])

        self.socket.send("{},{}".format(TOPIC_INTRUDER, message))


class Node:
    """
    Wrapper for communications between MAVProxy DSS and ROS
    """
    def __init__(self):

        # Create DSS Communications object
        self.comms = DSSComms(5556)

        # ROS subscribers
        # self.sub0 = rospy.Subscriber('tracks3d', Tracks, self.tracks_cb)

        
    def tracks_cb(self, msg):
        pass
        

if __name__ == '__main__':
    rospy.init_node('dss_comms', anonymous=False)

    try:
        obj = Node()

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass