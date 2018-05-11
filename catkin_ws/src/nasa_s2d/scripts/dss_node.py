#!/usr/bin/env python
import os, sys
import logging, argparse

import rospy

import dss

from mavros_msgs.msg import RCIn, State, GlobalPositionTarget
from mavros_msgs.srv import SetMode
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from nasa_s2d.msg import DitchSiteList, DitchSite


class ConnectPythonLoggingToROS(logging.Handler):
    """Connect the standard Python logging module to ROS loggin

    See https://gist.github.com/nzjrs/8712011.
    """

    MAP = {
        logging.DEBUG:    rospy.logdebug,
        logging.INFO:     rospy.loginfo,
        logging.WARNING:  rospy.logwarn,
        logging.ERROR:    rospy.logerr,
        logging.CRITICAL: rospy.logfatal
    }

    def emit(self, record):
        try:
            self.MAP[record.levelno]("%s: %s" % (record.name, record.msg))
        except KeyError:
            rospy.logerr("unknown log level %s LOG: %s: %s" % (record.levelno, record.name, record.msg))



# reconnect logging calls which are children of this to the ros log system
logging.getLogger('dss').addHandler(ConnectPythonLoggingToROS())
# logs sent to children of dss with a level >= this will be redirected to ROS
logging.getLogger('dss').setLevel(logging.DEBUG)



class ROSInterface(dss.core.AbstractInterface):
    """ROS Interface for DSS to Autopilot Communications
    """
    def __init__(self, params):
        super(ROSInterface, self).__init__()
        
        # Create a ROS node
        rospy.init_node('dss', anonymous=False)

        # ROS subscribers
        self.sub_global = rospy.Subscriber('mavros/global_position/global', NavSatFix, self.globalpos_cb)
        self.sub_relalt = rospy.Subscriber('mavros/global_position/rel_alt', Float64, self.relalt_cb)
        self.sub_state = rospy.Subscriber('mavros/state', State, self.state_cb)
        self.sub_vel = rospy.Subscriber('mavros/local_position/velocity', TwistStamped, self.velocity_cb)
        self.sub_rcin = rospy.Subscriber('mavros/rc/in', RCIn, self.rcin_cb)

        # ROS publishers
        self.pub_setpos = rospy.Publisher('mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=1, latch=True)
        self.pub_ditchsite = rospy.Publisher('ditch_sites', DitchSiteList, queue_size=1, latch=True)

        # store configuration parameters
        self.params = params

        # let the ROS network know about the potential ditch sites
        self.publish_ditch_sites(self.params.ditch_site_package)

        self.reset()


    def globalpos_cb(self, msg):
        """Receive vehicle GPS information
        """
        self._data['lat'] = msg.latitude
        self._data['lon'] = msg.longitude
        self._data['alt'] = msg.altitude # [m] absolute, above sea level (ASL)


    def relalt_cb(self, msg):
        """Receive relative altitude above vehicle home message
        """
        self._data['rel_alt'] = msg.data


    def state_cb(self, msg):
        """Receive heartbeat/status messages

        Published with a rate of 1 Hz
        """
        self.state = msg


    def velocity_cb(self, msg):
        """Receive velocity information in the body frame (FLU)
        """
        self._data['vx'] = msg.twist.linear.x
        self._data['vy'] = msg.twist.linear.y
        self._data['vz'] = msg.twist.linear.z


    def rcin_cb(self, msg):
        """Receive RC inputs that the FCU got from the Tx
        Expressed in raw microseconds.
        """

        # TODO: Determine if S2D is engaged or not.
        if msg.channels[5] > 1500:
            self._s2d_engaged = True
        else:
            self._s2d_engaged = False


    def change_mode(self, mode):
        rospy.wait_for_service('mavros/set_mode')
        try:
            set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
            resp = set_mode(0, mode)
            return resp.mode_sent
        except rospy.ServiceException as e:
            rospy.logerr("change mode failed: %s", e)


    def publish_ditch_sites(self, sites, selected_ds=None):

        msg = DitchSiteList()

        for site in sites:
            ds = DitchSite()
            ds.name = site.name
            ds.position.latitude = site.lat
            ds.position.longitude = site.lon
            ds.position.altitude = site.alt
            ds.radius = site.radius
            ds.type = site.type
            ds.reliability = site.reliability

            if selected_ds is not None and selected_ds.name == site.name:
                ds.selected = True

            msg.ditch_sites.append(ds)

        self.pub_ditchsite.publish(msg)

    ###########################################################################
    ##                   Interface Methods to be Overridden                  ##
    ###########################################################################

    def reset(self):
        super(ROSInterface, self).reset()

        # a place to store the latest vehicle status message from mavros
        self.state = State()


    def set_path(self, path):

        self.publish_ditch_sites(self.params.ditch_site_package, path[-1])


    def set_guidance_waypoint(self, wp):
        
        if self.state.mode != 'GUIDED':
            self.change_mode('GUIDED')

        msg = GlobalPositionTarget()

        msg.header.stamp = rospy.Time.now()

        # Use LLA with altitude being above the home position (assumes flat earth!)
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        
        # Ignore everything but LLA (4088)
        msg.type_mask = 0b0000111111111000

        # Set LLA to go to
        msg.latitude = wp.lat
        msg.longitude = wp.lon
        msg.altitude = wp.alt

        self.pub_setpos.publish(msg)





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
    intf = ROSInterface(params)

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