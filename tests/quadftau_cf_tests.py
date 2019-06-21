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

""" Unit Tests for the quadftau module """

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

import numpy as np
import os

from context import ut
from context import qftau_cf
from context import cts

def unitttest_template_A():
    file.write("\n\nUnit Test %s: \n" % u_name)
    file.write("####################################################\n")
    try:
        file.write("cmd = %s\n" % np.array2string(cmd))
        fb, taub = qftau_cf.input2ftau(cmd, np.identity(3), np.array([0,0,0]))

        file.write("fb = %s\n" % np.array2string(fb))
        file.write("taub = %s\n" % np.array2string(taub)) 
   
    except Exception as e:  
        file.write("TEST FAILED: %s\n" % e)
   
# Create a file for these tests
location_rb = "testresults" + "/" + "quadftau_cf"
if not os.path.exists(location_rb):
    os.makedirs(location_rb)
fullname = location_rb + "/" + "batch_01" + ".txt"

with open(fullname,"w") as file:
            
    u_name = "0001_cmd_zero"
    cmd = np.array([0,0,0,0])
    unitttest_template_A()

    u_name = "0002_cmd_verysmall"
    cmd = np.array([800,800,800,800])
    unitttest_template_A()
    
    u_name = "0003_cmd_zerotreshold"
    cmd = np.array([1000,1000,1000,1000])
    unitttest_template_A()
    
    u_name = "0004_cmd_hoovering"
    cmd = np.array([37278,37278,37278,37278])
    # this value is very close to the 36000 I used in my project
    # so this is good news for the identification report 
    unitttest_template_A()
    
    u_name = "0005_cmd_rolling_pos"
    cmd = np.array([38000,38000+1000,38000,38000-1000])
    unitttest_template_A()
    
    u_name = "0006_cmd_rolling_neg"
    cmd = np.array([38000,38000-1000,38000,38000+1000])
    unitttest_template_A()
    
    u_name = "0007_cmd_pitching_pos"
    cmd = np.array([38000-1000,38000,38000+1000,38000])
    unitttest_template_A()
    
    u_name = "0008_cmd_pitching_neg"
    cmd = np.array([38000+1000,38000,38000-1000,38000])
    unitttest_template_A()
    
    u_name = "0009_cmd_yawing_pos"
    cmd = np.array([38000+1000,38000,38000+1000,38000])
    unitttest_template_A()
    
    u_name = "0010_cmd_yawing_neg"
    cmd = np.array([38000,38000+1000,38000,38000+1000])
    unitttest_template_A()