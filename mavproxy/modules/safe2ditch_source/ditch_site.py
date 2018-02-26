# -*- coding: utf-8 -*-
"""
Created on Fri Jan 12 11:18:57 2018

@author: nnmonroe
"""
from site_selection.waypoint import Waypoint

class DitchSite(Waypoint):
    
    def __init__(self, name='', lat=0.0, lon=0.0, alt=0.0, 
                 wp_type=0, radius=0.0, reliability=0.0,
                 desirability=0.0, range_factor=0.0, 
                 radius_factor=0.0, intruder_factor=0.0, wp_range=0.0):
        
        #super().__init__(lat, lon, alt)
        self.name = str(name)
        self.type = int(wp_type)
        self.radius = float(radius)
        self.range = float(wp_range)
        self.reliability = float(reliability)
        self.desirability = float(desirability) 
        self.range_factor = float(range_factor)
        self.radius_factor = float(radius_factor)
        self.intruder_factor = float(intruder_factor)
        
        if self.range_factor is None:
            self.range_factor = 0.0
            
        if self.radius_factor is None:
            self.radius_factor = 0.0
            
        if self.intruder_factor is None:
            self.intruder_factor = 0.0

        if self.desirability is None:
            self.desirability = 0.0
            
