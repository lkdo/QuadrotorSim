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
import numpy as np
import utils as ut

class Logger:
  """ This class implements logging functionality """
  
  def __init__(self, baseName, runName):
     self.baseName = baseName
     self.runName = runName
     
     self.RigidBodyLogger_time = []
     self.RigidBodyLogger_pos = []
     self.RigidBodyLogger_euler = []
     
  def log_RigidBody(self, t, RB):
     self.RigidBodyLogger_time.append(t)   
     self.RigidBodyLogger_pos.append(RB.pos)
     self.RigidBodyLogger_euler.append(ut.R2EXYZ(np.transpose(RB.Rb2e)))
     
  def print_RigidBody(self):
      for i in range(len(self.RigidBodyLogger_time)):
        print(self.RigidBodyLogger_time[i],self.RigidBodyLogger_pos[i],
              self.RigidBodyLogger_euler[i])
        