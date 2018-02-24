#!/usr/bin/env python
from builtins import range

import rospy

import numpy as np
import zmq

from visual_mtt.msg import Tracks

class DSSComms:
    """
    Ditch Site Selector Communications handler.
    """

    # Topic IDs for message types
    TOPIC_INTRUDER = 1000

    def __init__(self, port):

        self.port = port

        # Create ZeroMQ socket connection
        self.socket = zmq.Context().socket(zmq.PUB)
        self.socket.bind("tcp://*:{}".format(self.port))


    def send_intruders(self, intruders):
        """
        send_intruders(intruders)

        :param: intruders
                List of tuples with lat/lon of each intruder.
                e.g., intruders=[(lat,lon), (lat,lon), ...]
        """

        # message contains number of intruders followed by lat/lon for each
        message = ""
        for intruder in intruders:
            message = "{},{},{}".format(message, intruder[0], intruder[1])

        self.socket.send("{}{}".format(DSSComms.TOPIC_INTRUDER, message))


class Node:
    """
    Wrapper for communications between MAVProxy DSS and ROS
    """
    def __init__(self):

        # Create DSS Communications object
        self.comms = DSSComms(5556)

        # ROS subscribers
        self.sub0 = rospy.Subscriber('obstacles', Tracks, self.obstacles_cb)

        
    def obstacles_cb(self, msg):
        # If there are no tracks to process, just bail!
        if not msg.tracks:
            return

        intruders = []

        # Extract lat/lon from tracks
        for track in msg.tracks:
            intruder = (track.position.y, track.position.x)
            intruders.append(intruder)

        self.comms.send_intruders(intruders)
        

if __name__ == '__main__':
    rospy.init_node('dss_comms', anonymous=False)

    try:
        obj = Node()

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass