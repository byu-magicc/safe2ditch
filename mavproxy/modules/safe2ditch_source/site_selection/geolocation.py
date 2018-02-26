# -*- coding: utf-8 -*-
"""
Created on Thu Feb  2 14:37:56 2017

@author: nnmonroe
"""

""" Class for using geographic location for selecting the best ditch site

    In terms of distance, the units will be in terms of km"""
   
from waypoint import Waypoint


class Geolocation:
    
    def __init__(self, ditch_site_package):
        self.ditch_sites = ditch_site_package
     
    def get_path_lat(self, index):
        
        # returns the latitude residing at the desired index
        return self.ditch_sites[index].lat
     
    def get_path_lon(self, index):
        
        # returns the longitude residing at the desired index
        return self.ditch_sites[index].lon
    
    def get_path_alt(self, index):
        
        # returns the altitude residing at the desired index
        return self.ditch_sites[index].alt 
    
    def pick_site(self, veh_cur_wp):
        ditch_site_package = []
        site_lat = self.get_path_lat(0)
        site_lon = self.get_path_lon(0)
        site_alt = self.get_path_alt(0)
        
        ditch_site_wp = Waypoint(site_lat, site_lon, site_alt)
        ditch_site_package.append(ditch_site_wp)
        
        """ Pick the first ditch site as the closest ditch site, update 
            continuously as closest site is found """
        
        smallest_distance = veh_cur_wp.distance(ditch_site_wp)
        
        for site in range(len(self.ditch_sites)):
            site_lat = self.get_path_lat(site)
            site_lon = self.get_path_lon(site)
            site_alt = self.get_path_alt(site)
            ditch_site_wp = Waypoint(site_lat, site_lon, site_alt)
            distance = veh_cur_wp.distance(ditch_site_wp)
            
            if smallest_distance > distance:
                smallest_distance = distance
                ditch_site_wp = Waypoint(site_lat, site_lon, site_alt)
                # only going to use the first element, saves from continuous
                # system calls of allocating and deallocating memory
                ditch_site_package[0] = ditch_site_wp
        
        return ditch_site_package
                
    
    

