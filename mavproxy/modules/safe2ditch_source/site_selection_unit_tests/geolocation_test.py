# -*- coding: utf-8 -*-
"""
Created on Thu Feb  2 21:53:00 2017

@author: nnmonroe
"""

from site_selection.geolocation import Geolocation
import unittest 

class TestGeolocation(unittest.TestCase):
    def TestDegreeConversion(self): # Test degree to radian conversion
        loc1 = Geolocation.to_radians(26.062951, -80.238853)
        loc2 = Geolocation.to_degrees(loc1.lat_rad, loc1.lon_rad)
        
        self.assertEqual(loc1.lat_rad, loc2.lat_rad)
        self.assertEqual(loc1.lon_rad, loc2.lon_rad) 
        self.assertEqual(loc1.lat_deg, loc2.lat_deg)
        self.assertEqual(loc1.lon_deg, loc2.lon_deg)
        
    def TestDistanceTo(self): # Test distance between two locations
        loc1 = Geolocation.to_radians(26.062951, -80.238853)
        loc2 = Geolocation.to_radians(26.060484,-80.207268)
        self.assertEquals(loc1.distance_to(loc2), loc2.distance_to(loc1))
    
    def TestBoundingCoordinates(self): # Test bounding box
        loc = Geolocation.to_radians(26.062951, -80.238853)
        distance = 1 # 1 kilometer
        SW_loc, NE_loc = loc.bounding_coordinates(distance)
        

def suite():
    suite = unittest.TestSuite()
    suite.addTest(TestGeolocation("TestDegreeConversion"))
    suite.addTest(TestGeolocation("TestDistanceTo"))
    suite.addTest(TestGeolocation("TestBoundingCoordinates"))
    return suite