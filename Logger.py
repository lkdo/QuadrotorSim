# -*- coding: utf-8 -*-
"""
Created on Tue Jun 18 20:32:31 2019

@author: Luminita-Cristiana Totu
"""

class Logger:
  """ Handles the logging """
  
  def __init__(self, baseName, runName):
     self.baseName = baseName
     self.runName = runName
     
     self.RigidBodyLogger_time = []
     self.RigidBodyLogger_pos = []
     
  def log_RigidBody(self, t, RB):
     self.RigidBodyLogger_time.append(t)   
     self.RigidBodyLogger_pos.append(RB.pos)
     
  def print_RigidBody(self):
      for i in range(len(self.RigidBodyLogger_time)):
        print(self.RigidBodyLogger_time[i],self.RigidBodyLogger_pos[i])