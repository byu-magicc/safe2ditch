# -*- coding: utf-8 -*-
"""
Created on Thu Mar  9 11:53:08 2017

@author: nnmonroe
"""
from math import radians, degrees, sin, cos, atan2, sqrt, asin

class Waypoint:
    
    def __init__(self, lat=0.0, lon=0.0, alt=0.0, name="unset"):
        self.lat = float(lat)
        self.lon = float(lon)
        self.alt = float(alt)
        self.name = name
    
    def reset_lat_lon(self, lat_in, lon_in):
        self.lat = lat_in
        self.lon = lon_in

    def update_lat_lon_alt(self, lat_in, lon_in, alt_in):
	self.lat = lat_in
	self.lon = lon_in
	self.alt = alt_in

    def distance(self, other_wp, radius=None):
        if radius is None:
            radius = 20.902 * 10**6 # radius of the Earth in ft
            
        dlon = other_wp.lon - self.lon
        dlat = other_wp.lat - self.lat
        a = (sin(dlat/2)**2 + cos(self.lat) 
                * cos(other_wp.lat) * sin(dlon/2)**2)
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
       
        return radius * c
    
    def bearing(self, other_wp):
        """ Calculate the initial bearing for the direction needed to reach the 
            destination """   
        y = sin(other_wp.lon - self.lon) * cos(other_wp.lat)
        x = (cos(self.lat) 
                * sin(other_wp.lat) 
                - sin(self.lat) 
                * cos(other_wp.lat) 
                * cos(other_wp.lon - self.lon))
        
        bearing = degrees(atan2(y, x)) 
        
        return radians((bearing + 360) % 360)
    
    def calc_new_lat(self, bearing, distance, radius=None): 
        if radius is None:
            radius = 20.902 * 10**6 # radius of the Earth in ft
            
        new_lat = asin(sin(self.lat) 
                        * cos(distance / radius)
                        + cos(self.lat)
                        * sin(distance / radius)
                        * cos(bearing))
        
        return new_lat
    
    def calc_new_lon(self, new_lat, bearing, distance, radius=None):
        if radius is None:
            radius = 20.902 * 10**6 # radius of the Earth in ft
        
        y = sin(bearing) * sin(distance / radius) * cos(self.lat)
        x = cos(distance / radius) - sin(self.lat) * sin(new_lat)
        
        return self.lon + atan2(y, x)
    
if __name__ == '__main__':
    
    wp1 = Waypoint()
    wp1.lat = radians(37.102148)
    wp1.lon = radians(-76.387542)
    wp1.alt = 0.0
    
    wp2 = Waypoint()
    wp2.lat = radians(37.102193)
    wp2.lon = radians(-76.38712)
    wp2.alt = 30.0
    
    distance = wp1.distance(wp2)
    bearing = wp1.bearing(wp2)
    
    print (wp1.lat, wp1.lon)
    print(distance)
    print(bearing)
    
    
