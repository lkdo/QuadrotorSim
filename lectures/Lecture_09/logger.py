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
import math 
import os 

class Logger:
    """ This class implements logging functionality """
  
    def __init__(self, location, basename):
        
        self.location = location
        self.basename = basename
     
        self.rb_time = []
        self.rb_pos = []
        self.rb_ve = []
        self.rb_euler = []
        self.rb_vb = []
        self.rb_omegab = []
        self.rb_alphab = []

        self.cmd_time = []
        self.cmd_rotors = []
        
        self.ftau_time = []
        self.ftau_fe = []
        self.ftau_fb = []
        self.ftau_taue = []
        self.ftau_taub = []
        
    # rigid body elements 
    ###################################################
    
    def log_rigidbody(self, t, rb):
        
        self.rb_time.append(t)
        self.rb_pos.append(rb.pos)
        self.rb_vb.append(rb.vb)  
        self.rb_euler.append(180/math.pi*rb.rpy)
        self.rb_ve.append(rb.ve)
        self.rb_omegab.append(rb.omegab)
        self.rb_alphab.append(rb.d_omegab)
     
    def log2file_rigidbody(self):
        
        location = self.location 
        if not os.path.exists(location):
            os.makedirs(location)
        fullname = location + "/" +  self.basename + "__rigidbody.txt"
        with open(fullname,"w") as f:
            f.write("time position ve vb euler_xyz omegab alphab \n")
            for i in range(len(self.rb_time)):
                c1 = np.array2string(np.array([self.rb_time[i]]), precision = 8, 
                        suppress_small=False, sign=" ",floatmode ="fixed")
                c2 = np.array2string(self.rb_pos[i], precision = 8, 
                        suppress_small=False, sign=" ",floatmode ="fixed")
                c3 = np.array2string(self.rb_euler[i],precision = 8, 
                        suppress_small=False, sign=" ",floatmode ="fixed")
                c4 = np.array2string(self.rb_vb[i],precision = 8, 
                        suppress_small=False, sign=" ",floatmode ="fixed")
                c5 = np.array2string(self.rb_omegab[i],precision = 8, 
                        suppress_small=False, sign=" ",floatmode ="fixed")
                c6 = np.array2string(self.rb_ve[i],precision = 8, 
                        suppress_small=False, sign=" ",floatmode ="fixed")
                c7 = np.array2string(self.rb_alphab[i],precision = 8, 
                        suppress_small=False, sign=" ",floatmode ="fixed")   
                
                fullline = c1+"  "+c2+"  "+c6+"  "+c4+"  "+c3+"  "+c5+"  "+c7+"\n"
                fullline = str(fullline).replace('[','').replace(']','')
                f.write(fullline)
    
    # Command elements 
    ###################################################
    
    def log_cmd(self, t, cmd_rotors):
        
        self.cmd_time.append(t)
        self.cmd_rotors.append(cmd_rotors)
   
    def log2file_cmd(self):
        
        location = self.location 
        if not os.path.exists(location):
            os.makedirs(location)
        fullname = location + "/" +  self.basename + "__cmd.txt"
        with open(fullname,"w") as f:
            f.write("time cmd_rotors \n")
            for i in range(len(self.cmd_time)):
                c1 = np.array2string(np.array([self.cmd_time[i]]), precision = 8, 
                        suppress_small=False, sign=" ",floatmode ="fixed")
                c2 = np.array2string(self.cmd_rotors[i], precision = 8, 
                        suppress_small=False, sign=" ",floatmode ="fixed")
                fullline = c1+"  "+c2+"\n"
                fullline = str(fullline).replace('[','').replace(']','')
                f.write(fullline)
                
    # forces and torques 
    ####################################################

    def log_ftau(self, t, fe, fb, taue, taub):
        
        self.ftau_time.append(t)
        self.ftau_fe.append(fe)
        self.ftau_fb.append(fb)
        self.ftau_taue.append(taue)
        self.ftau_taub.append(taub)
             
    def log2file_ftau(self):
        
        location = self.location 
        if not os.path.exists(location):
            os.makedirs(location)
        fullname = location + "/" +  self.basename + "__ftau.txt"
        with open(fullname,"w") as f:
            f.write("time fe taue fb taub \n")
            for i in range(len(self.ftau_time)):
                c1 = np.array2string(np.array([self.ftau_time[i]]), precision = 8, 
                        suppress_small=False, sign=" ",floatmode ="fixed")
                c2 = np.array2string(self.ftau_fe[i], precision = 8, 
                        suppress_small=False, sign=" ",floatmode ="fixed")
                c3 = np.array2string(self.ftau_taue[i], precision = 8, 
                        suppress_small=False, sign=" ",floatmode ="fixed")
                c4 = np.array2string(self.ftau_fb[i], precision = 8, 
                        suppress_small=False, sign=" ",floatmode ="fixed")
                c5 = np.array2string(self.ftau_taub[i], precision = 8, 
                        suppress_small=False, sign=" ",floatmode ="fixed")
                
                fullline = c1+"  "+c2+"  "+c3+"  "+c4+"  "+c5+"\n"
                fullline = str(fullline).replace('[','').replace(']','')
                f.write(fullline)
                