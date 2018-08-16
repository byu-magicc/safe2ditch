#!/usr/bin/env python

import sys, argparse, socket
import time, datetime, pickle
import os, subprocess, signal
import json

import rospy, rosbag
import dynamic_reconfigure.client

from colorama import init; init(autoreset=True)
from colorama import Fore, Style

import numpy as np
import matplotlib.pyplot as plt
plt.style.use('ggplot')

# PDFs don't seem to like Type 3 fonts, this will use Type 42
plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['ps.fonttype'] = 42

from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandLong, CommandLongRequest, CommandBool


class ROSLauncher:
    """ROSLauncher

        The object that actually interfaces with the roslaunch shell command.
    """
    def __init__(self, flags=None):

        self.flags = flags

        # Store the roslaunch process
        self.process = None

        # Hide the roslaunch output
        self.squelch = False


    def run(self):
        """Launch

            Create a subprocess that runs `roslaunch nasa_s2d_sim mcsim.launch`

            See: https://stackoverflow.com/a/4791612/2392520
        """
        if not rospy.is_shutdown():
            cmd = 'roslaunch nasa_s2d_sim mcsim.launch {}'.format(self.flags if self.flags is not None else '')
            if self.squelch:
                cmd += ' > /dev/null 2>&1'
            print("Running command: {}".format(cmd))
            self.process = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)

        else:
            print("rospy shutting down, could not run roslaunch command")


    def stop(self):
        # Send a kill signal to all the process groups
        os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)

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

        


class Simulation:
    """docstring for Simulation"""
    def __init__(self):
    
        # has the simulation been officially started?
        self.started = False

        # Run roslaunch and start a roscore
        launcher = ROSLauncher()
        launcher.run()

        rospy.init_node('mcsim', anonymous=False)

        try:
            # setup the monte carlo simualtion node
            self.setup()

            rate = rospy.Rate(1)
            while not rospy.is_shutdown():
                rate.sleep()
        except rospy.ROSInterruptException:
            pass

        launcher.stop()


    def setup(self):
        self.sub_pose = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pose_cb)


    def pose_cb(self, msg):
        if not self.started:
            self.start_simulation()
            self.started = True


    def start_simulation(self):
        MAVROS.change_mode('GUIDED')
        MAVROS.arm()
        MAVROS.takeoff(120)





        

class MCSim:
    """The simulation executive that manages Safe2Ditch monte carlo (MC) simulations"""
    def __init__(self):
        pass
        
    def run(self):
        sim = Simulation()




   
if __name__ == '__main__':
    # setup the simulation executive
    simexec = MCSim()

    simexec.run()
