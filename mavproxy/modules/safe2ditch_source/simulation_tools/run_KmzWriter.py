"""
Created on Mon, Feb 27, 2017

@author: pglaab

Runs the KmzWriter.py code to generate GoogleEarth animations
"""
import sys
from simulation_tools.KmzWriter_stage2 import KmzWriter as kmz
from TriageLogic import TriageLogic as triage

class RunKmzWriter:

    def __init__(self, config_file, route_data, 
                 veh_path_file, ditch_site_data, kmz_file_name):
        self.animation_file_name = "animation" + config_file.split(".")[0] + ".kmz"
        
        self.kmz = kmz(route_data, veh_path_file, ditch_site_data, kmz_file_name)

        self.feetToMeters = 0.3048
        
    def runNoEngage(self):
        self.kmz.createAnimation()

    def run(self, listOfXs):
        # Adds a big "X" to the site denoted in the input args
        self.kmz.createAnimationWithXs(listOfXs)

    def setAnimationName(self, nameIn):
        self.animation_file_name = nameIn
        self.kmz.setAnimationName(nameIn)

    def findLocationAtTime(self, timeSec):
        # parse through path file, and use lat/lon that corresponds to time
        self.path = open(self.vehicle_path_file)
        tracks = self.path.readlines()
        flat = 0.
        flon = 0.
        falt = 0.
        for track in tracks:
            timestr, flatstr, flonstr, faltstr = track.split()
            fileTimeInSec = float(timestr)
            if fileTimeInSec < timeSec:
                # Haven't exceeded time yet; update lat/lon
                flat = float(flatstr)
                flon = float(flonstr)
                falt = float(faltstr) * self.feetToMeters

        return flat, flon, falt
        
if __name__ == "__main__":

    if (len(sys.argv) == 4):
        # use case is run_KmzWriter configFileName animationName engageTimeSec
        timeEngage = float(sys.argv[2])
        timeToCrashSec = float(sys.argv[3])
        print("timeEngage = " + sys.argv[2] + ", timeToCrash = " + sys.argv[3])

        # Separate the specified config file name from the type identifier
        config_main = str(sys.argv[1]).split(".")

        # Append the config file name to the animation file to colate
        animation_file_name = "animation_" + config_main[0] + "_eng" + sys.argv[2]+"_ttc"+sys.argv[3]+".kmz"
        
        run_writer = RunKmzWriter(str(sys.argv[1]))
        # Overwrite default animation file name to include command line args
        run_writer.setAnimationName(animation_file_name)
        
        # Call the Triage code to find out where to put the X marker for engage
        # and to compute which ditch site would be selected
        notionalSpeed = 10  # feet/sec
        
        engageLat, engageLon, engageAlt = run_writer.findLocationAtTime(timeEngage)
        print("vehicle at engage time " + sys.argv[2] + ", lat/lon/alt = " + str(engageLat) + "/" + str(engageLon) + "/" + str(engageAlt))
        
        triage = triage(sys.argv[1])
        # The returned list contains at minimum a location for where the vehicle got
        # then engage signal, and a location for the selected site
        listOfBigXs = triage.selectSiteUsingTimeLeft(timeToCrashSec, engageLat, engageLon, engageAlt, notionalSpeed)
        run_writer.run(listOfBigXs)

    elif (len(sys.argv) == 2):
        # Supply non-default config file as argument
        run_writer = RunKmzWriter(str(sys.argv[1]))
        run_writer.runNoEngage()

    elif (len(sys.argv) == 1):
        # Run with default n
        run_writer = RunKmzWriter('config_file.json', 'animation.kmz')
        run_writer.runNoEngage()

    else:
        print("number of args used not allowable")
