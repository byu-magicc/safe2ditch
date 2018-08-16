#!/usr/bin/env python
from __future__ import print_function
from builtins import range

import sys, argparse, socket
import time, datetime, pickle
import os, subprocess, signal
import json

import rospy, rosbag, rosnode
import dynamic_reconfigure.client

import numpy as np

from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandLong, CommandLongRequest, CommandBool, ParamSet
from mavros_msgs.msg import OverrideRCIn, ParamValue
from nasa_s2d.msg import DitchSiteList


class ROSLauncher:
    """ROSLauncher

        The object that actually interfaces with the roslaunch shell command.
    """
    def __init__(self, flags=None):

        self.flags = flags

        # Store the roslaunch process
        self.process = None

        # Hide the roslaunch output
        self.squelch = True


    def run(self):
        """Launch

            Create a subprocess that runs `roslaunch nasa_s2d_sim mcsim.launch`

            See: https://stackoverflow.com/a/4791612/2392520
        """
        try:
            cmd = 'roslaunch nasa_s2d_sim mcsim.launch {}'.format(self.flags if self.flags is not None else '')
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
        self.topics = '-a -x "/camera/(.*)|/tracks_video(.*)|/hud(.*)|/rviz(.*)|/visualization(.*)|/gazebo(.*)|/iris(.*)|/mavlink/from|/mavros/battery|/rransac(.*)|/visual_frontend(.*)|/mavros/vfr_hud" /hud/image_raw/compressed'


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
    def ch6(value):
        # make sure that the correct SYSID_MYGCS is set:
        MAVROS.set_sysid_mygcs(1)

        pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=1)

        rc = OverrideRCIn()
        rc.channels[0] = OverrideRCIn.CHAN_NOCHANGE
        rc.channels[1] = OverrideRCIn.CHAN_NOCHANGE
        rc.channels[2] = OverrideRCIn.CHAN_NOCHANGE
        rc.channels[3] = OverrideRCIn.CHAN_NOCHANGE
        rc.channels[4] = OverrideRCIn.CHAN_NOCHANGE
        rc.channels[5] = value
        rc.channels[6] = OverrideRCIn.CHAN_NOCHANGE
        rc.channels[7] = OverrideRCIn.CHAN_NOCHANGE

        pub.publish(rc)

        


class Simulation:
    """docstring for Simulation"""
    def __init__(self, num_targets, m, end_ds):

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

        # Create a rosbag recorder
        self.bag = ROSBagRecorder()


    def start(self):
        # Run roslaunch and start a roscore
        launcher = ROSLauncher()
        launcher.run()

        rospy.init_node('mcsim', anonymous=False)

        try:
            #
            # setup the monte carlo simualtion node
            #
            
            # increase compression for smaller bag files
            client = dynamic_reconfigure.client.Client("/hud/image_raw/compressed", timeout=5)
            client.update_configuration({'jpeg_quality':10})

            # When this topic contains data, we know that the EKF is using GPS and we are ready to fly!
            self.sub_pose = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pose_cb)

            # subscribe to the ditch sites so we know where we are headed. This informs the termination of the sim
            self.sub_ditch_sites = rospy.Subscriber('dss/ditch_sites', DitchSiteList, self.ditchsites_cb)

            rate = rospy.Rate(1)
            while not rospy.is_shutdown() and not self.sim_done:

                ##
                ## Simulation heartbeat
                ##

                if '/gazebo' not in rosnode.get_node_names():
                    # something went wrong and this iteration
                    # needs to be re-ran.
                    return False

                rate.sleep()
        except rospy.ROSInterruptException:
            sys.exit(0)

        if self.sim_done:
            time.sleep(2)

        launcher.stop()
        self.bag.stop()

        rospy.signal_shutdown("monte carlo iteration complete")

        return True

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
        if msg.pose.position.z > 10 and not self.mission_started:
            self.mission_started = True
            MAVROS.change_mode('AUTO')

            self.timer = rospy.Timer(rospy.Duration(10), self.timer_cb)


    def start_flying(self):
        # start recording a rosbag
        self.bag.record('parker_test2_aug15.bag')

        MAVROS.change_mode('GUIDED')
        MAVROS.arm()
        MAVROS.takeoff(120)


    def timer_cb(self, event):
        MAVROS.ch6(1600)
        self.timer.shutdown()

      

class MCSim:
    """The simulation executive that manages Safe2Ditch monte carlo (MC) simulations"""
    def __init__(self, **kwargs):
        # list of different number of targets
        self.num_targets_list = kwargs['num_targets_list']

        # how many Monte Carlo iteration per num targets?
        self.M = kwargs['M']

        # terminate when the DSS reroutes to this one
        self.end_ds = kwargs['ending_ditch_site']
        
    def run(self):

        # MC iteration counter for this num targets
        m = 1

        # which index of the num_targets_list are we currently simulating?
        ntarget_idx = 0

        num_targets = self.num_targets_list[ntarget_idx]

        while m <= self.M:

            # =================================================================

            sim = Simulation(num_targets, m, self.end_ds)

            print("Starting simulation {}/{}...".format(ntarget_idx,m), end=''); sys.stdout.flush()
            completed = sim.start()

            if completed:
                print("complete.")
            else:
                print("failed!")

            time.sleep(2)

            # =================================================================

            #
            # Hack for reinitializing node (https://github.com/ros/ros_comm/issues/185)
            #
            
            rospy.client._init_node_args = None
            rospy.core._shutdown_flag = False
            rospy.core._in_shutdown = False

            if not completed:
                print("Re-simulating iteration {}/{}".format(ntarget_idx, m))
                m -= 1

            m += 1



   
if __name__ == '__main__':

    options = {
        'M': 2,
        'num_targets_list': list(range(10)),
        'ending_ditch_site': '23681_70' # terminate when the DSS reroutes to this one
    }

    # setup the simulation executive
    simexec = MCSim(**options)

    simexec.run()
