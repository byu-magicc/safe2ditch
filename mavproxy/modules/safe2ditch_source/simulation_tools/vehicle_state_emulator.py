# -*- coding: utf-8 -*-
"""
Created on Mon Feb  6 14:21:56 2017

@author: nnmonroe
"""

import os
import time
from math import degrees 
from simulation_tools.simulation_exec import SimulationExec
from waypoint import Waypoint

"""
    Takes the performance parameters given from the config file for distance
    and bearing calculations
    
    performance parameters:
        
        max climb rate
        max speed
        max descent rate
    
    
    The performance parameters are in ft/s 
"""

class VehicleStateEmulator:
    
    LOW = 0
    BEG_OF_PATH = 0
    TOLERANCE = 0.00001
    
    def __init__(self, output_file, config_file_data, 
                 route_file_data, wp_dist_tolerance):
        self.path = route_file_data
        self.se = SimulationExec(config_file_data)
        self.wp_dist_tolerance = wp_dist_tolerance
        self.veh_seg_dist_traveled = self.se.delta_time * self.se.max_speed
        self.veh_cur_wp = Waypoint()
        self.veh_next_wp = Waypoint()
        # sets up the initial lat/lon/alt, distance, and bearing values needed
        self.update_cur_veh_wp()
        # gets rid of the first waypoint in the list after the values are saved
        self.reduce_list() 
        self.update_next_veh_wp()
        self.bearing = self.veh_cur_wp.bearing(self.veh_next_wp)
        self.distance = self.veh_cur_wp.distance(self.veh_next_wp)
        self.climb_rate = self.get_climb_rate(self.veh_cur_wp.alt, 
                                              self.veh_next_wp.alt, 
                                              self.distance)
        self.descent_rate = self.get_climb_rate(self.veh_cur_wp.alt, 
                                                self.veh_next_wp.alt, 
                                                self.distance)
        self.file = self.open_file(output_file)
        self.commanded_speed = 0.
        self.sim_time = 0.
        self.running = True
        
    @staticmethod
    def limit(low, high, value):
        
        return max(low, min(high, value))
    
    @staticmethod
    def open_file(output_file):
        
        # returns the file object so that it can be closed later
        if os.path.isfile(output_file):
            try:
                file = open(output_file, 'w')
            except IOError as e:
                pass
        else:
            file = open(output_file, 'a')
                
        return file
        
    def close_file(cls, file_name):
        cls.file.close()
    
    def get_path_lat(self):
        # returns the latitude residing at the desired index
        return self.path[0].lat
     
    def get_path_lon(self):
        # returns the longitude residing at the desired index
        return self.path[0].lon
    
    def get_path_alt(self):
        # returns the altitude residing at the desired index
        return self.path[0].alt 
    
    def get_climb_rate(self, alt_cur, alt_next, distance):
        """ Finds the time it takes to reach the destination based on the speed
            of the vehicle. Then takes that time and divides it by height to 
            find the rate of descent """
        climb_rate = self.se.max_climb_rate
        self.commanded_speed = self.se.max_speed
        
        if distance != 0:
            time_at_max_speed = distance / self.se.max_speed
            height = alt_next - alt_cur
            climb_rate_max_ground_speed = height / time_at_max_speed
            if (abs(climb_rate_max_ground_speed) > self.se.max_climb_rate):
                time_to_climb = abs(height / climb_rate)
                self.commanded_speed = distance / time_to_climb
            else:
                climb_rate = climb_rate_max_ground_speed
        
        return climb_rate
    
    def end(self):
        """ Checks to see if there are anymore waypoints to fly to. If not,
            then prepare to land. """
        end = False
        if not self.path:
            end = True
            
        return end
   
    def reduce_list(self):
        """ Pop off the first element of the list after the 
            values have been attained. This process allows for the program to 
            only have to focus on the first element in the list """
        self.path.pop(self.BEG_OF_PATH)
    
    def prepare_for_safe2ditch_stage1(self, new_site_wps):
        """ Prepares the vehicle for crash landing by getting the ditch site,
            clearing the previous route information and inserting the new
            destination into the vehicle's path list """
        self.path = new_site_wps
        self.update_next_veh_wp()
        self.update_distance()
        self.update_bearing()
        self.update_descent_rate(self.veh_cur_wp.alt, 
                                 self.veh_next_wp.alt, 
                                 self.distance) 
        self.veh_seg_dist_traveled = self.se.delta_time * self.commanded_speed
        
    def prepare_for_safe2ditch_stage2(self, new_site_wps):
        """ Prepares the vehicle for crash landing by getting the ditch site,
            clearing the previous route information and inserting the new
            destination into the vehicle's path list """
        
        """ Set next lat lon alt to the first waypoint lat lon alt in the list"""
        self.path = new_site_wps
        self.update_next_veh_wp()
        self.reduce_list()
        self.update_distance()
        self.update_bearing()
        self.update_descent_rate(self.veh_cur_wp.alt, 
                                 self.veh_next_wp.alt, 
                                 self.distance)   
        self.veh_seg_dist_traveled = self.se.delta_time * self.commanded_speed
    
    def update_veh_attitude(self, guid_wp_in):
        # align vehicle's nose and related parameters to guidance waypoint
        self.update_distance()
        self.update_bearing()
        self.update_descent_rate(self.veh_cur_wp.alt, 
                                 self.veh_next_wp.alt, 
                                 self.distance)   

    def update_bearing(self):
        self.bearing = self.veh_cur_wp.bearing(self.veh_next_wp)
        
    def update_distance(self):
        self.distance = self.veh_cur_wp.distance(self.veh_next_wp)
     
    def update_cur_veh_wp(self):
        self.veh_cur_wp.lat = self.get_path_lat()
        self.veh_cur_wp.lon = self.get_path_lon()
        self.veh_cur_wp.alt = self.get_path_alt()
        
    def update_guidance_point(self, guid_wp_in):
        self.veh_next_wp.lat = guid_wp_in.lat
        self.veh_next_wp.lon = guid_wp_in.lon
        self.veh_next_wp.alt = guid_wp_in.alt

        # realign vehicle nose to guidance point
        self.update_veh_attitude(self.veh_next_wp)

    def update_next_veh_wp(self):
        """ Updates the comparison variables to the next waypoint's lat/lon/alt
            values """
        self.veh_next_wp.lat = self.get_path_lat()
        self.veh_next_wp.lon = self.get_path_lon()
        self.veh_next_wp.alt = self.get_path_alt()
        
    def update_climb_rate(self, alt_cur, alt_next, distance):
        """Updates the climb rate when the vehicle reaches the destination 
           of each of its segments""" 
        self.climb_rate = self.get_climb_rate(alt_cur, alt_next, distance)
        
    def update_descent_rate(self, alt_cur, alt_next, distance):
        """ Updates the descent rate when the vehicle reaches the destination
            of each of its segments """
        self.descent_rate = -1 * self.get_climb_rate(alt_cur, alt_next, distance)
        
    def update_target_wp(self):
         """ Resets the vehicle's information when it reaches the destination 
             of each of its segments """
         self.update_next_veh_wp()
         self.update_bearing()
         self.update_distance()
         self.update_climb_rate(self.veh_cur_wp.alt, 
                                self.veh_next_wp.alt, 
                                self.distance)
         self.update_descent_rate(self.veh_cur_wp.alt, 
                                  self.veh_next_wp.alt, 
                                  self.distance)   
         self.veh_seg_dist_traveled = self.se.delta_time * self.commanded_speed
        
    def calculate_segments(self, max_tolerance):
        """ Moves the vehicle and updates the lat/lon/alt values as well as 
            the bearing, distance, simulation time """
        self.veh_seg_dist_traveled = self.limit(self.LOW,
                                                self.distance,
                                                self.veh_seg_dist_traveled)
        new_lat = self.veh_cur_wp.calc_new_lat(self.bearing, 
                                               self.veh_seg_dist_traveled)
        new_lon = self.veh_cur_wp.calc_new_lon(new_lat,
                                               self.bearing,
                                               self.veh_seg_dist_traveled)
        new_alt = self.new_altitude(self.veh_cur_wp.alt, self.veh_next_wp.alt)
        
        self.veh_cur_wp = Waypoint(new_lat, new_lon, new_alt)
        self.update_bearing()
        self.update_distance()
        self.update_time()
        
    def compute_veh_state(self):
        # Find the ranges of distance tolerance for the vehicle
        dist_tolerance = self.TOLERANCE + self.wp_dist_tolerance
        """ If the end of the list has not been reached, continue calculating 
            the distance segments and popping off waypoints off of the
            path list, but if the end of the list has been reached, then
            continue calculating the distance segments until the vehicle has 
            reached the final destination and is at the correct altitude """
        self.calculate_segments(dist_tolerance)
        if (dist_tolerance >= self.distance and 
            self.veh_cur_wp.alt <= self.veh_next_wp.alt):
            if not self.end():
                self.update_target_wp()
                self.reduce_list()
            else:
                self.running = False

    def new_altitude(self, old_alt, WP_alt2):
        """ If the altitude is lower than the waypoint altitude necessary,
            then use the max climb rate to ascend or else, use the 
            predetermined descent rate. Since the descent rate is partly 
            calculated using the change in altitude, if there is no change, 
            the rate will be zero which will allow the vehicle to maintain
            its current altitude """
        if old_alt < WP_alt2:
            climb_rate = self.limit(self.LOW,
                                    self.se.max_climb_rate,
                                    self.climb_rate)
            new_alt = min(WP_alt2, (old_alt + (climb_rate * self.se.delta_time)))
        
        else:
            descent_rate = self.limit(self.se.max_descent_rate, 
                                      self.LOW, 
                                      self.descent_rate)
            new_alt = old_alt + (descent_rate * self.se.delta_time)
            if new_alt < WP_alt2:
                new_alt = WP_alt2
        
        return float(new_alt)
    
    def update_time(self):
        # used to provide the total time needed for the simulation
        self.sim_time += self.se.delta_time

        print("Time: ", self.sim_time)

        # slow the simulation to run at wall clock time
        time.sleep(self.se.delta_time)
    
    def output_veh_state(self, file_name):
        """ Send the vehicle's lat, long, and alt to a text file for the delta
            time period """
        lat = degrees(self.veh_cur_wp.lat)
        lon = degrees(self.veh_cur_wp.lon)
        self.file.write("{:0.2f} {:1.6f} {:2.6f} {:3.6f} {:4.6f} \n".format(
            time.time(), self.sim_time, lat, lon, self.veh_cur_wp.alt))
