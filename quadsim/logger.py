# -*- coding: utf-8 -*-
"""
Part of the QuadrotorSim package
Copyright (C) 2019  Luminita-Cristiana Totu
Contact: luminita.totu@gmail.com

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this package, in a file called LICENSE.  
If not, see <https://www.gnu.org/licenses/>.
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