#!/usr/bin/env python
from __future__ import print_function
from builtins import range

import sys, argparse, socket
import time, datetime, pickle
import os, subprocess, signal
import json

import numpy as np

class SimulationLauncher:
    """SimulationLauncher

        The object that actually interfaces with the roslaunch shell command.
    """
    def __init__(self, num_targets, m, end_ds):

        self.flags = "{} {} {}".format(num_targets, m, end_ds)

        # Store the roslaunch process
        self.process = None

        # Hide the roslaunch output
        self.squelch = False


    def start(self):
        """Launch

            Create a subprocess that runs `roslaunch nasa_s2d_sim mcsim.launch`

            See: https://stackoverflow.com/a/4791612/2392520
        """
        try:
            cmd = 'python2 mcsim.py {}'.format(self.flags)
            if self.squelch:
                cmd += ' > /dev/null 2>&1'
            # print("Running command: {}".format(cmd))
            self.process = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)

            try:
                self.process.wait()
            except KeyboardInterrupt:
                try:
                   self.stop()
                except OSError:
                   pass
                self.process.wait()

                print("exiting!")
                sys.exit(1)


            return self.process.returncode == 0

        except Exception as e:
            print("could not run simulation command")
            print(e)


    def stop(self):
        # Send a kill signal to all the process groups
        os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)

        # Wait a second to let roslaunch cleanly exit
        time.sleep(1.5)

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

        for num_targets in self.num_targets_list:

            # MC iteration counter for this num targets
            m = 1

            while m <= self.M:

                # =================================================================

                sim = SimulationLauncher(num_targets, m, self.end_ds)

                print("Starting simulation t{}_m{}...".format(num_targets,m), end=''); sys.stdout.flush()
                completed = sim.start()

                if completed:
                    print("complete.")
                else:
                    print("failed!")

                # time.sleep(2)

                # =================================================================

                #
                # Hack for reinitializing node (https://github.com/ros/ros_comm/issues/185)
                #

                if not completed:
                    print("Re-simulating iteration t{}_m{}".format(num_targets, m))
                    m -= 1

                m += 1



   
if __name__ == '__main__':

    options = {
        'M': 100,
        'num_targets_list': list(range(1,5)),
        'ending_ditch_site': '23681_70' # terminate when the DSS reroutes to this one
    }

    # setup the simulation executive
    simexec = MCSim(**options)

    simexec.run()
