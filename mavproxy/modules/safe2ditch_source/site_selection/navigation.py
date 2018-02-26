# -*- coding: utf-8 -*-
"""
Created on Thu Feb 1 08:47:56 2018

@author: pglaab
"""

""" Class for determining navigation commands based on the route 

    selected and the location of the vehicle"""
   
from waypoint import Waypoint


class Navigation:
    
    def __init__(self):
        self.dsIndex = 1       # ditch site index
        self.todIndex = 0      # top of descent point index
        self.pastTod = False   # vehicle passed TOD point
        self.dsLast = ""      # ditch site name given on last entry

        # this should be dynamically calculated using frame rate later
        self.distTolerance = 20.0 
     
    def resetDefaults(self):
        self.pastTod = False
        self.dsLast = "unset"

    def get_guidance_waypoint(self, flight_plan, current_location):
        
        # returns a waypoint (lat/lon/alt) that is the current point to 
        # steer to, to have vehicle follow the desired route with any 
        # route constraints considered

        # this method only uses the first two waypoints in the flight plan,
        # the top of descent point (index 1) and the ditch site (index 0).
        # output a warning if more than 2 waypoints are on the list sent
        if (len(flight_plan) > 2):
            print("Warning: Navigation::get_guidance_waypoint received a ")
            print("path with more than two points. Ignoring extras.")

        # Check if ditch site has changed, which is reset condition
        if (flight_plan[self.dsIndex].name != self.dsLast):
            self.pastTod = False
            self.dsLast = flight_plan[self.dsIndex].name

        # Check if vehicle has reached TOD, so should now proceed to ditch site
        if (not self.pastTod):
            if (flight_plan[self.todIndex].distance(current_location) < self.distTolerance):
                self.pastTod = True
                print "nav: vehicle past TOD"

        # If past TOD point, return the ditch site waypoint. Otherwise, return
        # the TOD waypoint
        if (self.pastTod):
            return flight_plan[self.dsIndex]
        else:
            return flight_plan[self.todIndex]
            
                
    
    

