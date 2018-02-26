# -*- coding: utf-8 -*-
"""
Created on Fri Mar 10 09:55:46 2017

@author: nnmonroe
"""

import json
from math import radians
from site_selection.waypoint import Waypoint
from ditch_site import DitchSite

class ConfigFileParser:
    
    def __init__(self, config_file):
        self.file = self.read_file(config_file)
        self.data = self.file['config']
        self.route_file = self.get_route_file()
        self.ditch_file = self.get_ditch_site_file()
        self.veh_path_file = self.get_vehicle_output_file()
        self.route_file_data = self.get_route_file_data()
        self.ditch_file_data = self.get_ditch_site_data()
        self.ditch_site_package = self.package_ditch_site_data()
        self.route_file_package = self.package_route_file_data()
    
    def read_file(self, file): 
        
        # read in the config file into a dictionary
        with open(file) as data_file:
            data = json.load(data_file)
            
        return data
    
    def get_route_file(self):
        
        # gets the name of the route file from the config file
        return self.data['files']['route_file_name']
    
    def get_ditch_site_file(self):
        
        # gets the name of the ditch site file from the config file
        return self.data['files']['ditch_sites_file_name']
        
    def get_vehicle_output_file(self):
        
        # gets the name of the vehicle output file from the config file
        return self.data['files']['vehicle_path_file']
        
    def get_route_file_data(self):
        
        # gets the data from the route json file 
        route_file_name = self.get_route_file()
        route_file_data = self.read_file(route_file_name)
        
        return route_file_data['routes']['route_def']
    
    def get_ditch_site_data(self):
        
        # gets the data from the ditch site json file
        ditch_sites_file_name = self.get_ditch_site_file()
        ditch_sites_file_data = self.read_file(ditch_sites_file_name)
        
        return ditch_sites_file_data['ditch_sites']['sites']
    
    def package_route_file_data(self):
        
        route_package = []
        route_data = self.get_route_file_data()
        
        for index in range(len(route_data)):
            waypoint = Waypoint()
            waypoint.lat = radians(float(route_data[index]['latitude']))
            waypoint.lon = radians(float(route_data[index]['longitude']))
            waypoint.alt = float(route_data[index]['altitude'])
            route_package.append(waypoint)
            
        return route_package
    
    def package_ditch_site_data(self):
        
        ditch_site_package = []
        ditch_site_data = self.get_ditch_site_data()
        
        for index in range(len(ditch_site_data)):
            ditch_site = DitchSite()
            ditch_site.name = ditch_site_data[index]['name']
            ditch_site.lat = radians(float(ditch_site_data[index]['latitude']))
            ditch_site.lon = radians(float(ditch_site_data[index]['longitude']))
            ditch_site.alt = float(ditch_site_data[index]['altitude'])
            ditch_site.radius = float(ditch_site_data[index]['radius'])
            ditch_site.type = int(ditch_site_data[index]['type'])
            ditch_site.reliability = float(ditch_site_data[index]['reliability'])
            ditch_site_package.append(ditch_site)
            
        return ditch_site_package
        
    
if __name__ == '__main__':
    
    cfp = ConfigFileParser('config_file_flight5.json')
    
    rfpackage = cfp.package_route_file_data()
    dspackage = cfp.package_ditch_site_data()
    
    for i in range(len(rfpackage)):
        print(rfpackage[i].lat)
        
    for i in range(len(dspackage)):
        print(dspackage[i].lon)
