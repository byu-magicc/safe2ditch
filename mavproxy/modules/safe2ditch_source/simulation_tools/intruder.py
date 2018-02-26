# -*- coding: utf-8 -*-
"""
Created on Fri Feb  2 13:08:56 2018

@author: pglaab
"""

""" Class for holding intruder data """
   
class Intruder:
    
    def __init__(self, name="unset"):
        self.name = name
        self.when = []     # strings holds time in seconds
        self.gxcoord = []  # strings hold lat, lon, alt with space to separate
     
    def add_when(self, when_string):
        self.when.append(when_string)

    def add_gxcoord(self, gxcoord_string):
        self.gxcoord.append(gxcoord_string)

    def get_when(self):
        return self.when

    def get_gxcoord(self):
        return self.gxcoord
