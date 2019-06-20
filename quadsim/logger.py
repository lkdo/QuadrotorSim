# -*- coding: utf-8 -*-
# 
# Copyright 2019 Luminita-Cristiana Totu
#
# Part of the QuadrotorSim aka quadsim package
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this package, in a file called LICENSE.
# If not, see <https://www.gnu.org/licenses/>.

""" This module implements the logging functionality """

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2004 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"


import numpy as np
import os 

class logger:
    """ This class implements logging functionality """
  
    def __init__(self, location, basename):
        self.location = location
        self.basename = basename
     
        self.rb_logger_time = []
        self.rb_logger_pos = []
        self.rb_logger_euler = []
        self.rb_logger_vb = []
        self.rb_logger_omegab = []
     
    def log_rigidbody(self, t, rb):
        
        self.rb_logger_time.append(t)
        
        self.rb_logger_pos.append(rb.pos)
        
        self.rb_logger_euler.append(rb.euler_xyz())
        
        self.rb_logger_vb.append(rb.vb)
        
        self.rb_logger_omegab.append(rb.omegab)
     
    def print_rigidbody(self):
        
        print("time ","position ", "euler_XYZ")
        for i in range(len(self.rb_logger_time)):
            print( self.rb_logger_time[i],self.rb_logger_pos[i],
                   self.rb_logger_euler[i], self.rb_logger_vb,
                   self.rb_logger_omegab )

    def log2file_rigidbody(self):
        
        location_rb = self.location + "/" + "rigidbody"
        
        if not os.path.exists(location_rb):
            os.makedirs(location_rb)
    
        fullname = location_rb + "/" +  self.basename + ".txt"
        with open(fullname,"w") as f:
            f.write("time position euler_xyz vb omegab \n")
            for i in range(len(self.rb_logger_time)):
                c1 = np.array2string(self.rb_logger_time[i], precision = 8, 
                        suppress_small=False, sign=" ",floatmode ="fixed")
                c2 = np.array2string(self.rb_logger_pos[i], precision = 8, 
                        suppress_small=False, sign=" ",floatmode ="fixed")
                c3 = np.array2string(self.rb_logger_euler[i],precision = 8, 
                        suppress_small=False, sign=" ",floatmode ="fixed")
                c4 = np.array2string(self.rb_logger_vb[i],precision = 8, 
                        suppress_small=False, sign=" ",floatmode ="fixed")
                c5 = np.array2string(self.rb_logger_omegab[i],precision = 8, 
                        suppress_small=False, sign=" ",floatmode ="fixed")
                
                fullline = c1+"  "+c2+"  "+c3+"  "+c4+"  "+c5+"\n"
                f.write(fullline)
        