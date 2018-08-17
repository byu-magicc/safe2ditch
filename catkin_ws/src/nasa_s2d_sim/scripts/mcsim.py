#!/usr/bin/env python
from __future__ import print_function
from builtins import range

import sys, argparse, socket
import time, datetime, pickle
import os, subprocess, signal
import json, datetime

import rospy, rosbag, rosnode
import dynamic_reconfigure.client

import numpy as np

from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandLong, CommandLongRequest, CommandBool, ParamSet
from mavros_msgs.msg import OverrideRCIn, ParamValue, State
from nasa_s2d.msg import DitchSiteList


class ROSLauncher:
    """ROSLauncher

        The object that actually interfaces with the roslaunch shell command.
    """
    def __init__(self, pkg, file, flags=None):

        self.pkg = pkg
        self.file = file
        self.flags = flags if flags is not None else ''

        # Store the roslaunch process
        self.process = None

        # Hide the roslaunch output
        self.squelch = False


    def run(self):
        """Launch

            Create a subprocess that runs `roslaunch nasa_s2d_sim mcsim.launch`

            See: https://stackoverflow.com/a/4791612/2392520
        """
        try:
            cmd = 'roslaunch {} {} {}'.format(self.pkg, self.file, self.flags)
            if self.squelch:
                cmd += ' > /dev/null 2>&1'
            # print("Running command: {}".format(cmd))
            self.process = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)

        except Exception as e:
            print("could not run roslaunch command")
            print(e)


    def stop(self):
        # Send a kill signal to all the process groups
        os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)

        # Wait a second to let roslaunch cleanly exit
        time.sleep(1.5)


class ROSBagRecorder:
    """docstring for ROSBagRecorder"""
    def __init__(self):

        # Store the rosbag process
        self.process = None

        # Hide the rosbag output
        self.squelch = True

        # self.topics = '-a -x "/camera/(.*)|/tracks_video(.*)|/hud(.*)|/rviz(.*)|/visualization(.*)" /camera/image_raw/compressed /camera/camera_info /tracks_video/compressed'
        self.topics = '-a -x "/camera/(.*)|/tracks_video(.*)|/hud(.*)|/rviz(.*)|/visualization(.*)|/gazebo(.*)|/iris(.*)|/mavlink/from|/mavros/battery|/rransac(.*)|/visual_frontend(.*)|/mavros/vfr_hud|/clock" /hud/image_raw/compressed'


    def record(self, bagname):
        """Launch

            Create a subprocess that runs `roslaunch nasa_s2d_sim mcsim.launch`

            See: https://stackoverflow.com/a/4791612/2392520
        """
        try:
            cmd = 'rosbag record {} -O {}'.format(self.topics, bagname)
            if self.squelch:
                cmd += ' > /dev/null 2>&1'
            # print("Running command: {}".format(cmd))
            self.process = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)

        except Exception as e:
            print("could not run rosbag record command")
            print(e)


    def stop(self):
        if self.process is None:
            return

        # Send a kill signal to all the process groups
        os.killpg(os.getpgid(self.process.pid), signal.SIGINT)

        # Wait a second to let roslaunch cleanly exit
        time.sleep(1.5)
        


class MAVROS:
    """docstring for MAVROS"""
    def __init__(self):
        pass


    @staticmethod
    def change_mode(mode):
        """Request to change flight mode
        Use mavros service call to change flight mode to GUIDED, AUTO, etc
        """

        # rospy.wait_for_service('mavros/set_mode')
        try:
            set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
            resp = set_mode(0, mode)
            return resp.mode_sent
        except rospy.ServiceException as e:
            rospy.logerr("change mode failed: %s", e)


    @staticmethod
    def takeoff(alt):
        """Send MAV_CMD_NAV_TAKEOFF MAVLink Command
        """
        req = CommandLongRequest()
        req.command = 22 # MAV_CMD_NAV_TAKEOFF, used in common
        req.param7 = alt

        try:
            command_long = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
            resp = command_long(req)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr("MAV_CMD_NAV_TAKEOFF failed: %s", e)

    @staticmethod
    def arm():
        """throttle arm
        """
        state = True
        try:
            arm = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            resp = arm(state)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr("MAV_CMD_NAV_TAKEOFF failed: %s", e)


    @staticmethod
    def set_sysid_mygcs(val):
        pv = ParamValue(integer=val, real=0.0)
        try:
            set = rospy.ServiceProxy('mavros/param/set', ParamSet)
            resp = set(param_id='SYSID_MYGCS', value=pv)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr("SYSID_MYGCS set failed: %s", e)

    @staticmethod
    def build_rc_override_ch6(value):
        # make sure that the correct SYSID_MYGCS is set:
        MAVROS.set_sysid_mygcs(1)

        rc = OverrideRCIn()
        rc.channels[0] = OverrideRCIn.CHAN_NOCHANGE
        rc.channels[1] = OverrideRCIn.CHAN_NOCHANGE
        rc.channels[2] = OverrideRCIn.CHAN_NOCHANGE
        rc.channels[3] = OverrideRCIn.CHAN_NOCHANGE
        rc.channels[4] = OverrideRCIn.CHAN_NOCHANGE
        rc.channels[5] = value
        rc.channels[6] = OverrideRCIn.CHAN_NOCHANGE
        rc.channels[7] = OverrideRCIn.CHAN_NOCHANGE

        return rc
        


class Simulation:
    """docstring for Simulation"""
    def __init__(self, num_targets, m, end_ds):

        self.num_targets = num_targets
        self.m = m

        # keep track of current ditch site and
        # the ditch site we would like to end on
        self.current_ds = None
        self.end_ds = end_ds
        self.reroutes = 0
    
        # Is the simulation complete?
        self.sim_done = False

        # have we started flying?
        self.flying = False

        # Are we in AUTO mode to start the mission?
        self.mission_started = False

        # this ROS timer will be given a random
        # period for engaging Safe2Ditch
        self.timer = None

        # latest MAVROS heartbeat/state message
        self.heartbeat = State()

        # Create a rosbag recorder
        self.bag = ROSBagRecorder()

        # what time did the sim start?
        self.time_started = None

        # What is an acceptable amount of time to wait
        # before we've started flying? If this time passes,
        # then something is likely to be wrong. (secs)
        self.MAX_WAIT_TIME = 80

        # keep track of target launches so we can kill them
        self.target_launches = []


    def start(self):
        # Run roslaunch and start a roscore
        launcher = ROSLauncher("nasa_s2d_sim", "mcsim.launch", "viz:=false")
        launcher.run()

        rospy.init_node('mcsim', anonymous=False)

        # make sure to stamp when we started
        self.time_started = time.time()

        try:
            #
            # setup the monte carlo simualtion node
            #

            self.spawn_targets()
            
            # increase compression for smaller bag files
            client = dynamic_reconfigure.client.Client("/hud/image_raw/compressed", timeout=5)
            client.update_configuration({'jpeg_quality':10})

            # When this topic contains data, we know that the EKF is using GPS and we are ready to fly!
            self.sub_pose = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pose_cb)

            # subscribe to the ditch sites so we know where we are headed. This informs the termination of the sim
            self.sub_ditch_sites = rospy.Subscriber('dss/ditch_sites', DitchSiteList, self.ditchsites_cb)

            # mavlink heartbeat
            self.sub_heartbeat = rospy.Subscriber('mavros/state', State, self.heartbeat_cb)

            # Connect to rc override topic to engage Safe2Ditch using channel 6
            self.rc_pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=1)

            sim_error = False

            rate = rospy.Rate(1)
            while not rospy.is_shutdown() and not self.sim_done and not sim_error:

                ##
                ## Simulation heartbeat
                ##

                if '/gazebo' not in rosnode.get_node_names():
                    # something went wrong and this iteration
                    # needs to be re-ran.
                   sim_error = True

                if not self.flying and time.time() - self.time_started > self.MAX_WAIT_TIME:
                    sim_error = True

                # If anything goes wrong during flight, bail
                if self.flying:
                    if not self.heartbeat.connected:
                        sim_error = True

                    if self.heartbeat.mode not in ['AUTO', 'GUIDED']:
                        sim_error = True

                rate.sleep()
        except rospy.ROSInterruptException:
            sim_error = True
        finally:
            self.bag.stop()
            launcher.stop()

            map(lambda x: x.stop(), self.target_launches)

            rospy.signal_shutdown("monte carlo iteration complete")

            return not sim_error

        return False # we shouldn't have gotten here...


    def spawn_targets(self):
        for n in range(self.num_targets):
            # build mover name (used as namespace)
            name = "target{}".format(n+1)

            # randomly choose velocity
            v = np.random.uniform(0.5, 2.5) # typical human walking speed is 1.4 m/s

            launcher = ROSLauncher("nasa_s2d_sim", "movers_mcsim.launch", "mover_name:={} velocity:={}".format(name, v))
            launcher.run()

            # add it to the list so it can be stopped later
            self.target_launches.append(launcher)


    def heartbeat_cb(self, msg):
        self.heartbeat = msg


    def ditchsites_cb(self, msg):

        for ds in msg.ditch_sites:
            if ds.selected:

                # Keep track of the number of reroutes
                if self.current_ds != ds.name:
                    self.reroutes += 1

                # Once we reroute to the ending ditch site, we can close the simulation
                if ds.name == self.end_ds:
                    self.sim_done = True

                # as a precaution, lets also end if we reroute more than 3 times
                if self.reroutes > 3:
                    self.sim_done = True

                self.current_ds = ds.name


    def pose_cb(self, msg):
        if not self.flying:
            self.flying = True
            self.start_flying()

        # TODO: randomize altitude?
        if msg.pose.position.z > 20 and not self.mission_started:
            self.mission_started = True
            MAVROS.change_mode('AUTO')

            T_engage = np.random.uniform(20, 60)
            self.timer = rospy.Timer(rospy.Duration(T_engage), self.timer_cb)


    def start_flying(self):
        # start recording a rosbag
        today = datetime.date.today().strftime('%d%b%Y')
        self.bag.record('mcsim_{}_t{}_m{}.bag'.format(today,self.num_targets,self.m))

        MAVROS.change_mode('GUIDED')
        MAVROS.arm()
        MAVROS.takeoff(120)


    def timer_cb(self, event):
        self.rc_pub.publish(MAVROS.build_rc_override_ch6(1600))
        self.timer.shutdown()


   
if __name__ == '__main__':

    if len(sys.argv[1:]) != 3:
        print("Use the right number of args!")
        sys.exit(1)

    num_targets = sys.argv[1]
    m = sys.argv[2]
    end_ds = sys.argv[3]

    sim = Simulation(num_targets, m, end_ds)

    completed = sim.start()

    sys.exit(0 if completed else 1)