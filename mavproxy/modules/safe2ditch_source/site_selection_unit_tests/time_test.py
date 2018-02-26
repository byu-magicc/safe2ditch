# -*- coding: utf-8 -*-
"""
Created on Thu Feb  2 14:37:01 2017

@author: nnmonroe
"""

from site_selection.time_constraint import Time
from time import localtime, strftime
import unittest


class TestTime(unittest.TestCase):
    def TestCurrentTime(self):
        cur_time = int(strftime("%H%M", localtime()))
        cur_time2 = Time.current_time()
        
        self.assertEqual(cur_time, cur_time2)
    
    def TestTimeConstraint(self):
        cur_time = Time.current_time()
        lh_time1 = cur_time + 2
        rh_time1 = cur_time + 5
        self.assertFalse(Time.avoid_constraint(cur_time, lh_time1, rh_time1))
        
        lh_time2 = cur_time - 3
        rh_time2 = cur_time + 5
        self.assertTrue(Time.avoid_constraint(cur_time, lh_time2, rh_time2))
        
                
def suite():
    suite = unittest.TestSuite()
    suite.addTest(TestTime("TestCurrentTime"))
    suite.addTest(TestTime("TestTimeConstraint"))
    return suite