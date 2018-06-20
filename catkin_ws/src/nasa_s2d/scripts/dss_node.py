#!/usr/bin/env python
import os, sys
import logging, commands

import rospy

import dss

from visual_mtt.msg import Tracks
from mavros_msgs.msg import RCIn, State, GlobalPositionTarget, WaypointList, Waypoint
from mavros_msgs.srv import SetMode, CommandLong, CommandLongRequest, WaypointPull
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64, String
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



class ROSInterface(dss.interfaces.AbstractInterface):
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
        self.pub_ditchsite = rospy.Publisher('dss/ditch_sites', DitchSiteList, queue_size=1, latch=True)
        self.pub_path = rospy.Publisher('dss/path', WaypointList, queue_size=1, latch=True)
        self.pub_git = rospy.Publisher('dss/git_info', String, queue_size=1, latch=True)

        # store configuration parameters
        self.params = params

        # let the ROS network know about the potential ditch sites
        self.publish_ditch_sites(self.params.ditch_site_package)

        self.reset()


    def send_git_info(self):
        """Publish git info

        Publishes the git info for this project so that it is captured in the ros bag
        """
        git = commands.getoutput('git rev-parse HEAD && git status && git submodule foreach --recursive git status && git submodule foreach --recursive git diff')

        msg = String()
        msg.data = git
        self.pub_git.publish(msg)


    def globalpos_cb(self, msg):
        """Receive vehicle GPS information
        """
        self._lock.acquire()
        self._data['lat'] = msg.latitude
        self._data['lon'] = msg.longitude
        self._data['alt'] = msg.altitude # [m] absolute, above sea level (ASL)
        self._lock.release()


    def relalt_cb(self, msg):
        """Receive relative altitude above vehicle home message
        """
        self._lock.acquire()
        self._data['rel_alt'] = msg.data
        self._lock.release()


    def state_cb(self, msg):
        """Receive heartbeat/status messages

        Published with a rate of 1 Hz
        """

        # Upon first arming, publish the ditch sites
        if not self.state.armed and msg.armed:
            # let the ROS network know about the potential ditch sites
            self.publish_ditch_sites(self.params.ditch_site_package)

            # Also re-publish waypoints
            self.pull_waypoints()

            # Send git version info to be recorded
            self.send_git_info()

        self.state = msg


    def velocity_cb(self, msg):
        """Receive velocity information in the body frame (FLU)
        """
        self._lock.acquire()
        self._data['vx'] = msg.twist.linear.x
        self._data['vy'] = msg.twist.linear.y
        self._data['vz'] = msg.twist.linear.z
        self._lock.release()


    def rcin_cb(self, msg):
        """Receive RC inputs that the FCU got from the Tx
        Expressed in raw microseconds.
        """

        self._lock.acquire()

        if msg.channels[5] > 1500:
            self._s2d_engaged = True
        else:
            self._s2d_engaged = False

        self._lock.release()


    def change_mode(self, mode):
        """Request to change flight mode
        Use mavros service call to change flight mode to GUIDED.
        As a safety precaution, only change mode to GUIDED
        if the current flight mode is AUTO.
        """

        if self.state.mode != 'AUTO':
            rospy.logwarn('[Safety] Will not change mode from {} to GUIDED for DSS'.format(self.state.mode))
            return False

        # rospy.wait_for_service('mavros/set_mode')
        try:
            set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
            resp = set_mode(0, mode)
            return resp.mode_sent
        except rospy.ServiceException as e:
            rospy.logerr("change mode failed: %s", e)


    def do_set_roi(self, lat, lon, alt):
        """Send DO_SET_ROI MAVLink Command
        """
        req = CommandLongRequest()
        req.command = 201 # DO_SET_ROI, used in APM:copter
        req.param5 = lat
        req.param6 = lon
        req.param7 = alt

        # rospy.wait_for_service('mavros/cmd/command')
        try:
            command_long = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
            resp = command_long(req)
            rospy.loginfo("Ack: DO_SET_ROI: {}, {}, {}".format(lat, lon, alt))
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr("DO_SET_ROI failed: %s", e)


    def pull_waypoints(self):
        """Pull mission waypoints
        """
        # rospy.wait_for_service('mavros/mission/pull')
        try:
            pull = rospy.ServiceProxy('mavros/mission/pull', WaypointPull)
            resp = pull()
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr("Waypoint pull failed: %s", e)


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


    def ditch_aborted(self):
        # Clear the path and the selected ditch site
        self.set_path([], None)

        # Command the vehicle to hold its current position
        wp = dss.helpers.Waypoint(*self.position_lla())
        self.set_guidance_waypoint(wp)

        # msg = GlobalPositionTarget()

        # msg.header.stamp = rospy.Time.now()

        # # Use LLA with altitude being above the home position (assumes flat earth!)
        # msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        
        # # Ignore everything but LLA (4088)
        # msg.type_mask = 0b0000111111000111

        # # Set zero velocity to stop moving
        # msg.velocity.x = 0
        # msg.velocity.y = 0
        # msg.velocity.z = 0

        # self.pub_setpos.publish(msg)


    def set_path(self, path, current):
        # We are assuming that the selected ditch site is
        # always the last waypoint in the path list.
        selected_ds = path[-1] if path else None
        self.publish_ditch_sites(self.params.ditch_site_package, selected_ds)

        msg = WaypointList()

        for idx, waypoint in enumerate(path):

            wp = Waypoint()
            wp.x_lat = waypoint.lat
            wp.y_long = waypoint.lon
            wp.z_alt = waypoint.alt

            wp.is_current = waypoint.name == current.name
            if wp.is_current:
                msg.current_seq = idx

            msg.waypoints.append(wp)

        self.pub_path.publish(msg)


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


    def set_ditch_site(self, ditch_site):

        if self.state.mode != 'GUIDED':
            self.change_mode('GUIDED')

        rospy.loginfo("Attempting to DO_SET_ROI at: {}".format(ditch_site))
        self.do_set_roi(ditch_site.lat, ditch_site.lon, ditch_site.alt)


class ROSVisionInterface(dss.interfaces.AbstractVisionInterface):
    """ROS Interface for vision communications
    """
    def __init__(self):
        super(ROSVisionInterface, self).__init__()

        # ROS subscribers
        self.sub0 = rospy.Subscriber('obstacles', Tracks, self.obstacles_cb)


    def obstacles_cb(self, msg):
        # If there are no tracks to process, just bail!
        if not msg.tracks:
            return

        # list of dss.helpers.Intruder objects
        intruders = []

        # Extract lat/lon from tracks
        for track in msg.tracks:
            intruder = dss.helpers.Intruder(track.id, track.position.y, track.position.x)
            intruders.append(intruder)

        self._set_intruders(intruders)            


def main():

    # Use rosparam server to find where the CWD should be set to
    # Note: it is not necessary to have a node already initialized
    desired_cwd = rospy.get_param('dss/config_dir', '.')

    # Change CWD to what the user gave through rosparam
    os.chdir(os.path.abspath(desired_cwd))

    # We assume that there is a file in the CWD called `config.json`
    config_file = rospy.get_param('dss/config_file', 'config.json')

    # Load the configuration parameters for the DSS
    parser = dss.parsers.JSONConfig(config_file)
    params = dss.parameters.HW(parser)

    # Setup the ROS DSS Interface
    intf = ROSInterface(params)

    # Setup the vision interface
    vintf = ROSVisionInterface()

    # Create a DSS manager that uses the ROS interface
    manager = dss.core.Manager(params, intf, vintf)

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