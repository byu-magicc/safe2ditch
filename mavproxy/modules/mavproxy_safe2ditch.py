#!/usr/bin/env python
'''safe2ditch command handling'''

from pymavlink import mavwp
from pymavlink import mavutil
import time,os
import math

from safe2ditch_source.site_selection.triage_logic import TriageLogic
from safe2ditch_source.site_selection.navigation import Navigation
from safe2ditch_source.config_file_parser import ConfigFileParser
from safe2ditch_source.site_selection.waypoint import Waypoint

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.mavproxy_mode import ModeModule

class Safe2ditchModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(Safe2ditchModule, self).__init__(mpstate, "safe2ditch", "safe2ditch handling")
        self.mode_cmdr = ModeModule(mpstate)
        self.max_maneuvers_exceeded = False
        self.max_maneuvers = 20
        self.num_site_selected = 0
        self.lat = 37.0
        self.lon = -76.0
        self.heading = 0.0
        self.alt = 0.0
        self.speed_fps = 0.0
        self.previous_mode = "unset"
        self.simulated_meters_2_go = 500.0
        self.this_mode = "unset"
        self.path_to_site = []
        self.meters_to_feet = 3.28084
        self.feet_to_meters = 1.0/self.meters_to_feet

        # default guidance waypoint is in bounds of CERTAIN 1
        self.safe_latitude = math.radians(37.10319)
        self.safe_longitude = math.radians(-76.385285)
        self.guidance_waypoint = Waypoint(self.safe_latitude, self.safe_longitude)
        self.guid_wp_past = Waypoint(self.safe_latitude, self.safe_longitude)
        self.guid_wp_past_name = "unset"
        self.veh_cur_wp = Waypoint(self.safe_latitude, self.safe_longitude)
        self.best_site = {'latitude': 37.10319,
                         'longitude': -76.385285,
                         'altitude': 10.0}

        # Specify the flight data directory
        data_dir = '/home/plusk01/dev/safe2ditch/s2d_config/'

        self.s2d_log_file = open(os.path.join(data_dir, 's2dEvents.txt'), mode='a')
        self.s2d_log_file.write("\n\n!! New Log !!\n\n")
        self.parser = ConfigFileParser(os.path.join(data_dir, 'config_file.json'))
        self.triage = TriageLogic(self.parser.ditch_site_package, os.path.join(data_dir, 'path.txt'))
        self.navigation = Navigation()
        self.reset_safe2ditch()

        print "safe2ditch loaded"

    def reset_safe2ditch(self):
        self.enabled = False
        self.s2d_engaged = False
        self.time_auto_eng = 3502400366.0
        self.requests_for_guided = 0
        self.guided_requested = False
        self.max_attempts = 4
        self.guide_lat = 37.3
        self.guide_lon = -76.3
        self.guide_alt = 100.0
        self.guidance_wpt_set = False
        self.guidance_waypoint
        self.meters2go = 9999999.9
        self.meters_tolerance = 1.0
        self.arrived = False
        self.time_mode_change_max = 3.0
        self.s2d_engage_wait = 10
        self.simulated_meters_2_go = 500.0
        self.land_requested = False
        self.time_land_req = 3502400366.0
        self.requests_for_land = 0
        self.ditch_site_found = False
        self.guidance_wpt_set = False
        self.guid_wpt_set_req = 0
        self.snap_time_in_guided = False
        self.time_guided_eng = 3502400366.0
        self.wait_in_guidance = 2
        self.time_in_guided = 0
        self.guidance_waypoint.reset_lat_lon(self.safe_latitude, self.safe_longitude)

        print("completed reset")
        
    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        type = m.get_type()

        if type == 'HEARTBEAT':
            self.previous_mode = self.this_mode
            self.this_mode = self.master.flightmode
            
            self.s2d_log_file.write("this mode: {}\tprevious mode: {}\n".format(self.this_mode, self.previous_mode))
            if self.this_mode == "STABILIZE" and self.previous_mode != "STABILIZE":
                # Pilot requesting command back, reset
                self.reset_safe2ditch()
            elif self.this_mode == "AUTO":
                if self.previous_mode == "STABILIZE":
                    self.s2d_engaged = True
                    self.time_auto_eng = time.time()
                elif self.previous_mode == "AUTO":
                    if self.s2d_engaged:
                        time_waiting_for_s2d_active = time.time() - self.time_auto_eng
                        self.s2d_log_file.write("wait time: {}\n".format(time_waiting_for_s2d_active))
                        if time_waiting_for_s2d_active > self.s2d_engage_wait:
                            self.s2d_log_file.write("wait time acheived, engaging\n")
                            if self.guided_requested == False:
                                self.mpstate.functions.process_stdin("mode GUIDED")
                                self.time_guided_req = time.time()
                                self.guided_requested = True
                                self.requests_for_guided = self.requests_for_guided + 1
                                
                            time_waiting_for_guided = time.time() - self.time_guided_req
                            if time_waiting_for_guided > self.time_mode_change_max:
                                if self.requests_for_guided <= self.max_attempts:
                                    # reset the timer for another try
                                    self.time_guided_req = time.time()
                                    self.requests_for_guided = self.requests_for_guided + 1
                                    self.mpstate.functions.process_stdin("mode GUIDED")
                                    print "mavproxy_safe2ditch requested guided mode"
                                else:
                                    # exceeded max attempts, reset s2d
                                    self.reset_safe2ditch()
                                    print("s2d GUIDED mode change attempts exceeded, resetting")
                                    # pilot must recycle through stabilize->auto to retry
                                self.s2d_log_file.write("request for guided = {}\n".format(self.requests_for_guided))
                else:
                    # transition to AUTO from any mode other than STABILIZE resets safe2ditch
                    self.reset_safe2ditch()
                    
            elif self.this_mode == "GUIDED":
                if self.s2d_engaged:
                    if self.snap_time_in_guided == False:
                        self.time_guided_eng = time.time()
                        self.snap_time_in_guided = True

                    self.time_in_guided = time.time() - self.time_guided_eng
                    #print("time in guided is {} seconds".format(self.time_in_guided))
                    if self.time_in_guided > self.wait_in_guidance:
                        
                        # update vehicle waypoint object to hold the lat/lon/alt of the vehicle
                        self.veh_cur_wp.update_lat_lon_alt(math.radians(self.lat), math.radians(self.lon), self.alt)

                        self.pick_ditch_site()
                        self.ditch_site_found = True

                        if self.ditch_site_found == True:
                            # pick guidance point each iteration because previous guidance point
                            # may sequence en route to ditch site

                            #self.guid_wp_past = self.guidance_waypoint
                            self.guid_wp_past_name = self.guidance_waypoint.name 
                            self.guidance_waypoint = self.navigation.get_guidance_waypoint(self.path_to_site, 
                                                                                           self.veh_cur_wp)
                            
                            if (self.guidance_waypoint.name == self.guid_wp_past_name):
                                # same guidance point as last pass, no need to update to copter
                                self.guidance_wpt_set = True
                            else:
                                # moving to next path waypoint, need to reset guidance wp to copter
                                self.guidance_wpt_set = False
                                self.guid_wpt_set_req = 0
                                print "mp_s2d, guid wp change"

                            if self.guidance_wpt_set == False:
                                # send 3 times, to be sure it takes
                                if self.guid_wpt_set_req < self.max_attempts:
                                    guid_lat = math.degrees(self.guidance_waypoint.lat)
                                    guid_lon = math.degrees(self.guidance_waypoint.lon)
                                    guid_alt = self.guidance_waypoint.alt * self.feet_to_meters
                                    # safety check, don't let guide to less than 3 meters alt
                                    if (self.guidance_waypoint.alt < 3.0):
                                       guid_alt = 3.0

                                    guided_cmd = "guided {} {} {}".format(guid_lat, guid_lon, guid_alt)
                                    print guided_cmd
                                    self.mpstate.functions.process_stdin(guided_cmd)
                                    self.guid_wpt_set_req = self.guid_wpt_set_req + 1
                                else:
                                    self.guidance_wpt_set = True

                            elif self.arrived == False:
                                # are we there yet?
                                self.meters2go = mp_util.gps_distance(self.lat, self.lon,
                                                                      self.guidance_waypoint.lat,
                                                                      self.guidance_waypoint.lon)
                                value4 = ('s2d loc: {} {} {} meters to go {}: '.format(self.lat,
                                                                                       self.lon,
                                                                                       self.alt,
                                                                                       self.meters2go))
                                value4_string = str(value4)
                                self.s2d_log_file.write(value4_string)
                                if self.meters2go < self.meters_tolerance:
                                    self.arrived = True;
                                    self.s2d_log_file.write('safe2ditch arrived to ditch site')

                        '''
			else:
                            # land for final step
                            if self.land_requested == False:
                                self.mpstate.functions.process_stdin("mode LAND")
                                self.time_land_req = time.time()
                                self.land_requested = True
                                self.requests_for_land = self.requests_for_land + 1
                                
                                time_waiting_for_land = time.time() - self.time_land_req
                                if time_waiting_for_land > self.time_mode_change_max:
                                    if self.requests_for_land <= self.max_attempts:
                                        # reset the timer for another try
                                        self.time_land_req = time.time()
                                        self.requests_for_land = self.requests_for_land + 1
                                        self.mpstate.functions.process_stdin("mode LAND")
                                    else:
                                        # exceeded max attempts, reset s2d
                                        self.reset_safe2ditch()
                                        # pilot must land vehicle
			'''
                                
        elif type == 'GLOBAL_POSITION_INT':
            # units can be found at mavlink.org/messages.common:
            # time_boot_ms   ms since system boot
            # lat            degrees * 1E7
            # lon            degrees * 1E7
            # alt            millimeters
            # relative_alt   millimeters 
            # vx             m/s * 100
            # vy             m/s * 100
            # vz             m/s * 100
            # hdg            deg * 100

            # store local copies for distance to go calculation
            self.lat = m.lat*1.0e-7
            self.lon = m.lon*1.0e-7
            self.heading = m.hdg*0.01
              
            # mavlink sends vx and vy in meters/sec * 100, convert to feet/sec
            vx_fps = m.vx * 0.01 * self.meters_to_feet
            vy_fps = m.vy * 0.01 * self.meters_to_feet
            self.speed_fps = math.sqrt(vx_fps * vx_fps + vy_fps * vy_fps)
            # Convert millimeters from mavlink to feet
            self.alt = m.alt*1.0e-3 * self.meters_to_feet
       
    def pick_ditch_site(self):
        print("picking_ditch_site")
        time_left_sec = 200    # need to update to take in time to crash
        #self.triage = TriageLogic(self.parser.ditch_site_package, "/home/nvidia/dev/MyCopter/path.txt")

        #triage.output_best_site_for_ralley(time_left_sec, 
        #                                   self.lat, 
        #                                   self.lon, 
        #                                   self.alt, 
        #                                   10.0)
        #self.best_site = triage.get_best_site(time_left_sec, 
        #                                      self.lat, 
        #                                      self.lon, 
        #                                      self.alt, 
        #                                      10.0)
        #self.guide_lat = self.best_site['latitude']
        #self.guide_lon = self.best_site['longitude']
        #self.guide_alt = self.best_site['altitude']
        #print("best site returned {} {} {}".format(self.guide_lat, self.guide_lon,
        #                                           self.guide_alt))

        # get navigation path from triage logic
        max_veh_speed = 10.0        # feet/sec, should really come from perf data
        self.path_to_site = self.triage.select_new_waypoint(time_left_sec, 
                                                            max_veh_speed,
                                                            self.veh_cur_wp)
    def simulate_site_arrival(self):
        # asymptotic decay
        sim_gain = 0.2
        self.simulated_meters_2_go = self.simulated_meters_2_go - self.simulated_meters_2_go * sim_gain
        return self.simulated_meters_2_go

    def select_ditch_site(self):
        return

def init(mpstate):
    '''initialise module'''
    return Safe2ditchModule(mpstate)

