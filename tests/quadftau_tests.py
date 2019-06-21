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

from context import ut
from context import qftau

# Values from the Crazyflie    
m1_omegar2ftau = qftau.Omegar2FTau(0.05,0.005022,1.8580*(10**-5))

def unitttest_template_A():
    print("\nUnit Test %s: " % u_name)
    print("####################################################")
    try:
        print("omega_r = %s" % np.array2string(omegar, precision = 8))
        f, tau = m1_omegar2ftau.omegar2ftau(omegar)
    
        print("f = %s " % np.array2string(f))
        print("tau = %s" % np.array2string(tau)) 
    
        omegar_2 = m1_omegar2ftau.ftau2omegar(f,tau)
        print("omega_r = %s" % np.array2string(omegar_2, precision = 8))
    except Exception as e:  
        print("TEST FAILED: %s" % e)
    else:
        diff = omegar - omegar_2
        if ( np.linalg.norm(diff) > 0.000001 ):
            print("TEST FAILED: %s != %s" % (omegar, omegar_2) )
        else: 
            print("Test passed")    
    
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

