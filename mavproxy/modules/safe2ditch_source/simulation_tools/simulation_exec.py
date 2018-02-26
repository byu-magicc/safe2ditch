# -*- coding: utf-8 -*-
"""
Created on Mon Feb  6 13:21:20 2017

@author: nnmonroe
"""

"""
Structure of the config file (json)

   root: configuration
   branches:
           path_def (number of waypoints needed to reach destination):
                       latitude
                       longitude
                       altitude
                  
           perf_def:
                   max_climb_rate
                   max_speed
                   max_descent_rate
           time_specs:
                   engage_time
                   time_interval
           sites (number of sites in database):
                        name
                        latitude
                        longitude
                        altitude
                        
    max_climb_rate, max_speed, and max_descent_rate are in ft/s

"""                  

class SimulationExec:
    
    def __init__(self, data):
        self.max_climb_rate = float(data['perf_def']['max_climb_rate_FPS'])
        self.max_speed = float(data['perf_def']['max_speed_FPS'])
        self.max_descent_rate = float(data['perf_def']['max_descent_rate_FPS'])
        self.delta_time = float(data['time_specs']['time_interval'])
                                         
    
    
    
    
    
    
    
