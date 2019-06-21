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
from context import qftau

# Values from the Crazyflie    
m1_omegar2ftau = qftau.Omegar2FTau(0.05,0.005022,1.8580*(10**-5))

def unitttest_template_A():
    file.write("\n\nUnit Test %s: \n" % u_name)
    file.write("####################################################\n")
    try:
        file.write("omega_r = %s\n" % np.array2string(omegar, precision = 8))
        f, tau = m1_omegar2ftau.omegar2ftau(omegar)
    
        file.write("f = %s\n" % np.array2string(f))
        file.write("tau = %s\n" % np.array2string(tau)) 
    
        omegar_2 = m1_omegar2ftau.ftau2omegar(f,tau)
        file.write("omega_r = %s\n" % np.array2string(omegar_2, precision = 8))
    except Exception as e:  
        file.write("TEST FAILED: %s\n" % e)
    else:
        diff = omegar - omegar_2
        if ( np.linalg.norm(diff) > 0.000001 ):
            file.write("TEST FAILED: %s != %s\n" % (omegar, omegar_2) )
    
# Create a file for these tests
location_rb = "testresults" + "/" + "quadftau"
if not os.path.exists(location_rb):
    os.makedirs(location_rb)
fullname = location_rb + "/" + "batch_01" + ".txt"

with open(fullname,"w") as file:
            
    u_name = "0001_tau_zero"
    omegar = np.array([100,100,100,100])   
    unitttest_template_A()

    u_name = "0002_taux" # roll positive
    omegar = np.array([100,200,100,0])  
    unitttest_template_A()

    u_name = "0003_taux_minus" # roll negative
    omegar = np.array([100,0,100,200])   
    unitttest_template_A()

    u_name = "0004_tauy" #pitch positive 
    omegar = np.array([0,100,200,100])   
    unitttest_template_A()

    u_name = "0004_tauy_minus" #pitch negative 
    omegar = np.array([200,100,0,100])   
    unitttest_template_A()

    u_name = "0004_tauz" #yaw positive 
    omegar = np.array([200,100,200,100])
    unitttest_template_A()

    u_name = "0004_tauz_minus" #yaw negative 
    omegar = np.array([100,200,100,200])
    unitttest_template_A()
