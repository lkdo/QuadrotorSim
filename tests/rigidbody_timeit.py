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

""" Test Cases for the Rigid Body class implementing basic kinematics """

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

import numpy as np
import timeit

from context import rb
from context import log
from context import ut


def testcase_template_B():
    """ Testcase template for initial conditions and constant input """
    quad = rb.rigidbody(pos,rotmb2e,vb,omegab,mass,I)
    
    for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):
   
        # Run the dynamic / time forward
        quad.run_quadrotor(dt_sim, Fb, taub)

        # Logging frequency    
        if ( abs(t % dt_log) < 0.00001 ):
             quad.check()

#######################################################################

def testcase_template_C():
    """ Testcase template for initial conditions and constant input """
    
    quad = rb.rigidbody_q(pos,q,vb,omegab,mass,I)
    
    for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):
   
        # Run the dynamic / time forward
        quad.run_quadrotor(dt_sim, Fb, taub)

        # Logging frequency    
        if ( abs(t % dt_log) < 0.00001 ):
             quad.check()

#######################################################################

# Initialization values for the states of the quadrotor

pos = np.array([0,0,0])
q = np.array([1.0,0.0,0.0,0.0])
rotmb2e = ut.quat2rotm(q)
vb = np.array([0,0,0])
omegab = np.array([0,0,0])
mass = 1
I = np.identity(3)

# Simulation parameters
dt_sim = 0.001   # seconds
T_sim = 10       # seconds
dt_log = 1       # seconds

Fb = np.array([0.01,0.01,0.01])
taub = np.array([0.01,0.01,0.01])

N = 100  # number of speed tests

name = "1000_rigidbody_speedcheck"
print("\nRunning Test Case: %s " % name)
logger = log.logger("testresults",name) 
timer_rigidbody=timeit.timeit(testcase_template_B,
                              number=N,globals=globals())
print("Average time for rigidbody class %s \n" % (timer_rigidbody/N))

# Test zero forces and moments 
name = "1001_rigidbody_q_speedcheck"
print("Running Test Case: %s " % name)
logger = log.logger("testresults",name)
timer_rigidbody_q=timeit.timeit(testcase_template_C,
                                number=N,globals=globals())
print("Average time for rigidbody_q class %s \n" % (timer_rigidbody_q/N))