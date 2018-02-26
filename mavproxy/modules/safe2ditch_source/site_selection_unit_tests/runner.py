# -*- coding: utf-8 -*-
"""
Created on Thu Feb  2 14:44:32 2017

@author: nnmonroe
"""

import unittest

import day_test
import time_test
import geolocation_test

day_suite = day_test.suite()
time_suite = time_test.suite()
geo_suite = geolocation_test.suite()

suite = unittest.TestSuite()
suite.addTest(day_suite)
suite.addTest(time_suite)
suite.addTest(geo_suite)

unittest.TextTestRunner(verbosity=2).run(suite)