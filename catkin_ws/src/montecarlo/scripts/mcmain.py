#!/usr/bin/env python
from __future__ import print_function
from builtins import range

import sys, argparse, socket
import time, datetime, pickle
import os, subprocess, signal
import json, re

import numpy as np

class SimulationLauncher:
    """SimulationLauncher

        The object that actually interfaces with the roslaunch shell command.
    """
    def __init__(self, num_targets, m, viz):

        self.flags = "{} {} {}".format(num_targets, m, 1 if viz else 0)

        # Store the roslaunch process
        self.process = None

        # Hide the roslaunch output
        self.squelch = True


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


            return self.process.returncode == 0, self.process.returncode

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

        # Should we show the rviz?
        self.viz = kwargs['viz']

        # sometimes you might want to resume were you left off
        if 'start' in kwargs:
            t, m = kwargs['start']

            if m <= self.M:
                self.start_m = m

            # Make sure that the requested start target is in the list
            if t in self.num_targets_list:
                # find the index of this item in the list
                idx = self.num_targets_list.index(t)

                # Chop off everything before that in the list
                self.num_targets_list = self.num_targets_list[idx:]
        else:
            self.start_m = None

        
    def run(self):

        for num_targets in self.num_targets_list:

            # MC iteration counter for this num targets
            m = 1

            # Check if we should start this one time at a given m
            if self.start_m is not None:
                m = self.start_m
                self.start_m = None

            while m <= self.M:

                # =================================================================

                sim = SimulationLauncher(num_targets, m, self.viz)

                print("Starting simulation t{}_m{}...".format(num_targets,m), end=''); sys.stdout.flush()
                completed, reason = sim.start()

                if completed:
                    print("complete.")
                else:
                    print("failed! ({})".format(reason))

                # =================================================================

                if not completed:
                    print("Re-simulating iteration t{}_m{}".format(num_targets, m))
                    m -= 1

                m += 1



   
if __name__ == '__main__':

    options = {
        'M': 100,
        'num_targets_list': list(range(1,7)),
        'viz': False,
    }

    if len(sys.argv[1:]) >= 1 and sys.argv[1] == 'viz':
        options['viz'] = True

    if len(sys.argv[1:]) >= 2:
        t, m = re.findall(r't(\d+)_m(\d+)', sys.argv[2])[0]
        options['start'] = (int(t), int(m))

    # setup the simulation executive
    simexec = MCSim(**options)

    simexec.run()
