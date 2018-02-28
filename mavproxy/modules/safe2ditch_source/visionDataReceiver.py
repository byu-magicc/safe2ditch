# Text program for vision data receiver

import zmq
import sys
import time
import os
import thread
import math
from site_selection.waypoint import Waypoint

class VisionDataReceiver:

    def __init__(self):
        self.time_max = 60 
        self.time = 0.0
        self.time_delta = 0.5
        self.port = "5556"
        self.topic = "1000"  # id for intruder lat/lon data
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)

        self.socket.connect("tcp://localhost:%s" % self.port)
        self.socket.setsockopt(zmq.SUBSCRIBE, self.topic)

        # file to contain intruder history
        try:
            self.file = open("intruder_history.txt", 'w')
        except IOError as e:
            print "VDR: could not open intruder history output file."
            pass

        self.intruders = []

        print("subscriber connected to port" + self.port)

   # For the given location and radius, return the number of intruders inside
    def getNumberIntrudersWithin(self, ditch_site):
        # for each intruder known, compare distance between that guy and 
        # the specified lat/lon, and see if it is less than the radius given
        intruderCount = 0
        for i in range(len(self.intruders)):
            distIntruder = self.intruders[i].distance(ditch_site)

            if distIntruder < ditch_site.radius:
                # This one is inside, bump up counter before continuing
                intruderCount = intruderCount + 1

        return intruderCount

    def getIntruders():
        return self.intruders

    def run(self):
        try:
            thread.start_new_thread(self.listenStore, ())
            print "vDR run, after thread creations"
        except:
            print "Error: unable to start thread"

    def listenStore(self):
        while True:

            socket_msg = self.socket.recv()
            print "Receiver received: " + socket_msg 

            # clear the old version of the intruder lat/lon list in prep
            # for storage of new values
            for i in range(0, len(self.intruders)):
                self.intruders.pop()

            parts = socket_msg.split(",")
            topic, lat_lons = parts[0], parts[1:]
            for i in range(0, len(lat_lons)/2):
                new_intruder = Waypoint(math.radians(float(lat_lons[i*2])), 
                                        math.radians(float(lat_lons[i*2+1])), 0.0)
                self.intruders.append(new_intruder)

            self.file.write(str(time.time()) + " ")
            for i in range(len(self.intruders)):
                self.file.write("{:0.6f} {:1.6f} {:2.6f} ".format(math.degrees(self.intruders[i].lat),
                                                                 math.degrees(self.intruders[i].lon), 
                                                                 self.intruders[i].alt))
            self.file.write("\n")
               
               

if __name__ == '__main__':

    receiver = VisionDataReceiver()
    receiver.run()
        
