# -*- coding: utf-8 -*-
"""
Created on Thu Feb  9 17:51:41 2017

@author: nnmonroe
"""
from simulation_tools.vehicle_state_emulator import VehicleStateEmulator as vse
from site_selection.geolocation import Geolocation
from site_selection.triage_logic import TriageLogic
from site_selection.navigation import Navigation

import time

class RunSimulation:
    
    def __init__(self, engage_time, config_file_data, route_file_data, 
                 ditch_site_data, output_file, WP_dist_tolerance, other_file):
        self.engage_time = float(engage_time)
        self.output_file = output_file
        self.vse = vse(self.output_file, config_file_data, 
                       route_file_data, WP_dist_tolerance)
        self.geo_loc = Geolocation(ditch_site_data)
        self.triage = TriageLogic(ditch_site_data, other_file)
        self.navigation = Navigation()
        self.has_been_engaged = False
        self.list_of_big_Xs = []
        
    def engage_safe2ditch(self, sim_time, engage_time):
        """ Notifies the program when safe2ditch needs to be engaged """
        engage_safe2ditch = False
        
        """ If the simulation time is greater than or equal to the time 
            selected for Safe2Ditch to be engaged, as well as Safe2Ditch
            not having been engaged already, engage Safe2Ditch """
        if sim_time >= engage_time:
            engage_safe2ditch = True
        
        return engage_safe2ditch
    
    def run_stage1(self):
        """ This will run the first stage of the simulator, which will 
            pick the closest ditch site from where Safe2Ditch was engaged """
        has_been_engaged = False
        self.vse.output_veh_state(self.output_file)
        
        while self.vse.running: 
            
           if self.engage_safe2ditch(self.vse.sim_time, self.engage_time) and not has_been_engaged:
                ditch_site = self.geo_loc.pick_site(self.vse.veh_cur_wp)
                self.vse.prepare_for_safe2ditch_stage1(ditch_site)
                has_been_engaged = True
           self.vse.compute_veh_state()
           self.vse.output_veh_state(self.output_file)
        
        self.vse.close_file(self.output_file)
        print('\nTotal simulation time: ', '{:0.4f}'. format(self.vse.sim_time))
        
    def run_stage2(self, time_to_crash):
        """ This will run the second stage of the simulator, which will pick
            the best site based on the highest desirability number """    
        self.vse.output_veh_state(self.output_file)
        has_been_engaged = False
        
        while self.vse.running:
            if self.engage_safe2ditch(self.vse.sim_time, self.engage_time):
                new_ditch_site_wps = self.triage.select_new_waypoint(time_to_crash, 
                                                                     self.vse.commanded_speed,
                                                                     self.vse.veh_cur_wp)

                self.list_of_big_Xs = self.triage.select_site(time_to_crash, 
                                                              self.vse.commanded_speed,
                                                              self.vse.veh_cur_wp)

                guidance_waypoint = self.navigation.get_guidance_waypoint(new_ditch_site_wps, 
                                                                          self.vse.veh_cur_wp)

                self.vse.update_guidance_point(guidance_waypoint)
                has_been_engaged = True
                
            self.vse.compute_veh_state()
            self.vse.output_veh_state(self.output_file)

        self.vse.close_file(self.output_file)
        self.triage.output_best_site_for_ralley(time_to_crash,
                                                self.vse.commanded_speed,
                                                self.vse.veh_cur_wp)
       
        print('\nTotal simulation time: ', '{:0.4f}'. format(self.vse.sim_time))
        
        
    
