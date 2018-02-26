# -*- coding: utf-8 -*-
"""
Created on Thu Feb  2 14:36:45 2017

@author: nnmonroe
"""

from site_selection.day_constraint import Day
import unittest

class TestDay(unittest.TestCase):
    def TestMonday(self):
        self.assertTrue(Day.convert_to_char(0) == 'M')
                
    def TestTuesday(self):
        self.assertTrue(Day.convert_to_char(1) == 'T')
        
    def TestWednesday(self):
        self.assertTrue(Day.convert_to_char(2) == 'W')
        
    def TestThursday(self):
        self.assertTrue(Day.convert_to_char(3) == 'H')
        
    def TestFriday(self):
        self.assertTrue(Day.convert_to_char(4) == 'F')
        
    def TestSaturday(self):
        self.assertTrue(Day.convert_to_char(5) == 'S')
        
    def TestSunday(self):
        self.assertTrue(Day.convert_to_char(6) == 'N')
        
    def TestDayConstraint(self):
        today1 = Day.today()
        today2 = Day.convert_to_char(today1)
        self.assertTrue(Day.avoid_constraint(today1, today2))
         
        today3 = Day.today()
        today4 = (Day.today()) - 1
        today4 = Day.convert_to_char(today4)
        self.assertFalse(Day.avoid_constraint(today3, today4))
        
def suite():
    suite = unittest.TestSuite()
    suite.addTest(TestDay("TestMonday"))
    suite.addTest(TestDay("TestTuesday"))
    suite.addTest(TestDay("TestWednesday"))
    suite.addTest(TestDay("TestThursday"))
    suite.addTest(TestDay("TestFriday"))
    suite.addTest(TestDay("TestSaturday"))
    suite.addTest(TestDay("TestSunday"))
    suite.addTest(TestDay("TestDayConstraint"))
    return suite

