#!/usr/bin/env python
import os, sys
import argparse

import rospy

import dss

from mavros_msgs.msg import RCIn, State, GlobalPositionTarget
from mavros_msgs.srv import SetMode
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped

class ROSInterface(dss.core.AbstractInterface):
    """ROS Interface for DSS to Autopilot Communications
    """
    def __init__(self):
        super(ROSInterface, self).__init__()
        
        # Create a ROS node
        rospy.init_node('dss', anonymous=False)

        # ROS subscribers
        self.sub_global = rospy.Subscriber('mavros/global_position/global', NavSatFix, self.globalpos_cb)
        self.sub_state = rospy.Subscriber('mavros/state', State, self.state_cb)
        self.sub_vel = rospy.Subscriber('mavros/local_position/velocity', TwistStamped, self.velocity_cb)
        self.sub_rcin = rospy.Subscriber('mavros/rc/in', RCIn, self.rcin_cb)

        # ROS publishers
        # self.pub_home = rospy.Publisher('visualization/home', NavSatFix, queue_size=1, latch=True)
        # self.pub_mission = rospy.Publisher('visualization/mission', MarkerArray, queue_size=1, latch=True)

        # Data store -- most recent data provided by ROS
        self.data = {
            'lat': 0,
            'lon': 0,
            'alt': 0,
            'vx': 0,
            'vy': 0,
            'vz': 0,
            'state': State()
        }


    def globalpos_cb(self, msg):
        """Receive vehicle GPS information
        """
        self.data['lat'] = msg.latitude
        self.data['lon'] = msg.longitude
        self.data['alt'] = msg.altitude # [m] Positive is above the WGS84 ellipsoid


    def state_cb(self, msg):
        """Receive heartbeat/status messages
        """
        self.data['state'] = msg


    def velocity_cb(self, msg):
        """Receive velocity information in the body frame (FLU)
        """
        self.data['vx'] = msg.twist.linear.x
        self.data['vy'] = msg.twist.linear.y
        self.data['vz'] = msg.twist.linear.z


    def rcin_cb(self, msg):
        """Receive RC inputs that the FCU got from the Tx
        Expressed in raw milliseconds.
        """
        
        # TODO: Determine if S2D is engaged or not.
        pass

    ###########################################################################
    ##                   Interface Methods to be Overridden                  ##
    ###########################################################################

    def _position_lla(self):
        return (self.data['lat'], self.data['lon'], self.data['alt'])


    def _velocity(self):
        return (self.data['vx'], self.data['vy'], self.data['vz'])



def main():

    # Create an argument parser to get the desired CWD for this node
    parser = argparse.ArgumentParser(description='Ditch Site Selection Node')
    parser.add_argument('--dir', dest='dir', default='.', help='working directory to use')
    args, unknown = parser.parse_known_args() # ignores unrecognized args

    # Change CWD to what the user gave as input
    os.chdir(os.path.abspath(args.dir))

    # We assume that there is a file in the CWD called `config.json`
    config_file = 'config.json'

    # Load the configuration parameters for the DSS
    parser = dss.parsers.JSONConfig(config_file)
    params = dss.parameters.HW(parser)

    # Setup the ROS DSS Interface
    intf = ROSInterface()

    # Create a DSS manager that uses the ROS interface
    manager = dss.core.Manager(params, intf)

    try:
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():

            # Run the DSS manager to check if Safe2Ditch is engaged
            manager.run(blocking=False)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass



if __name__ == '__main__':
    main()