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

""" Unit Tests for the rigidbody module """

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

import numpy as np

from context import rb
from context import log
from context import ut


def unittest_template_A():
    """ Unit test template for initial conditions and constant input """
    
    print("Running Test Case: %s \n" % name)
    
    logger = log.logger("testresults",name)
    
    for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):
   
        # Run the dynamic / time forward
        quad.run_quadrotor(dt_sim, Fb, taub)

        # Logging frequency    
        if ( abs(t % dt_log) < 0.00001 ):
             quad.check()
             logger.log_rigidbody(t, quad) 

    logger.log2file_rigidbody()
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


# Test zero forces and moments 
quad = rb.rigidbody(pos,rotmb2e,vb,omegab,mass,I)
name="0000_rigidbody_zeros"
Fb = np.array([0,0,0])
taub =  np.array([0,0,0])
unittest_template_A()

# Test zero forces and moments 
quad = rb.rigidbody_q(pos,q,vb,omegab,mass,I)
name = "0001_rigidbody_q_zeros"
Fb = np.array([0,0,0])
taub =  np.array([0,0,0])
unittest_template_A()

# Test Fx force 
quad = rb.rigidbody(pos,rotmb2e,vb,omegab,mass,I)
name = "0010_rigidbody_fx"
Fb = np.array([0.1,0,0])
taub =  np.array([0,0,0])
unittest_template_A()

# Test Fx force 
quad = rb.rigidbody_q(pos,q,vb,omegab,mass,I)
name = "0011_rigidbody_q_fx"
Fb = np.array([0.1,0,0])
taub =  np.array([0,0,0])
unittest_template_A()

# Test Fy force 
quad = rb.rigidbody(pos,rotmb2e,vb,omegab,mass,I)
name = "0020_rigidbody_fy"
Fb = np.array([0,0.1,0])
taub =  np.array([0,0,0])
unittest_template_A()

# Test Fy force 
quad = rb.rigidbody_q(pos,q,vb,omegab,mass,I)
name = "0021_rigidbody_q_fy"
Fb = np.array([0,0.1,0])
taub =  np.array([0,0,0])
unittest_template_A()

# Test Fz force
quad = rb.rigidbody(pos,rotmb2e,vb,omegab,mass,I)
name = "0030_rigidbody_fz"
Fb = np.array([0,0,0.1])
taub =  np.array([0,0,0])
unittest_template_A()

# Test Fz force
quad = rb.rigidbody_q(pos,q,vb,omegab,mass,I)
name = "0031_rigidbody_q_fz"
Fb = np.array([0,0,0.1])
taub =  np.array([0,0,0])
unittest_template_A()

# Test tau_x moment
quad = rb.rigidbody(pos,rotmb2e,vb,omegab,mass,I)
name = "0040_rigidbody_taux"
Fb = np.array([0,0,0])
taub =  np.array([0.1,0,0])
unittest_template_A()

# Test tau_x moment
quad = rb.rigidbody_q(pos,q,vb,omegab,mass,I)
name = "0041_rigidbody_q_taux"
Fb = np.array([0,0,0])
taub =  np.array([0.1,0,0])
unittest_template_A()

# Test tau_y moment
quad = rb.rigidbody(pos,rotmb2e,vb,omegab,mass,I)
name = "0050_rigidbody_tauy"
Fb = np.array([0,0,0])
taub =  np.array([0,0.1,0])
unittest_template_A()

# Test tau_y moment
quad = rb.rigidbody_q(pos,q,vb,omegab,mass,I)
name = "0051_rigidbody_q_tauy"
Fb = np.array([0,0,0])
taub =  np.array([0,0.1,0])
unittest_template_A()

# Test tau_z moment
quad = rb.rigidbody(pos,rotmb2e,vb,omegab,mass,I)
name = "0060_rigidbody_tauz"
Fb = np.array([0,0,0])
taub =  np.array([0,0,0.1])
unittest_template_A()

# Test tau_z moment
quad = rb.rigidbody_q(pos,q,vb,omegab,mass,I)
name = "0061_rigidbody_q_tauz"
Fb = np.array([0,0,0])
taub =  np.array([0,0,0.1])
unittest_template_A()

# Test initial conditions on Vx
quad = rb.rigidbody(pos,rotmb2e,vb+[0.1, 0, 0],omegab,mass,I)
name = "0070_rigidbody_vx"
Fb = np.array([0,0,0])
taub =  np.array([0,0,0])
unittest_template_A()

# Test initial conditions on Vx
quad = rb.rigidbody_q(pos,q,vb+[0.1, 0, 0],omegab,mass,I)
name = "0071_rigidbody_q_vx"
Fb = np.array([0,0,0])
taub =  np.array([0,0,0])
unittest_template_A()

# Test initial conditions on Vy
quad = rb.rigidbody(pos,rotmb2e,vb+[0, 0.1, 0],omegab,mass,I)
name = "0080_rigidbody_vy"
Fb = np.array([0,0,0])
taub =  np.array([0,0,0])
unittest_template_A()

# Test initial conditions on Vy
quad = rb.rigidbody_q(pos,q,vb+[0, 0.1, 0],omegab,mass,I)
name = "0081_rigidbody_q_vy"
Fb = np.array([0,0,0])
taub =  np.array([0,0,0])
unittest_template_A()

# Test initial conditions on Vz
quad = rb.rigidbody(pos,rotmb2e,vb+[0, 0, 0.1],omegab,mass,I)
name = "0090_rigidbody_vz"
Fb = np.array([0,0,0])
taub =  np.array([0,0,0])
unittest_template_A()

# Test initial conditions on Vz
quad = rb.rigidbody_q(pos,q,vb+[0, 0, 0.1],omegab,mass,I)
name = "0091_rigidbody_q_vz"
Fb = np.array([0,0,0])
taub =  np.array([0,0,0])
unittest_template_A()

# Test initial conditions on ox
quad = rb.rigidbody(pos,rotmb2e,vb,omegab+[0.1, 0, 0],mass,I)
name = "0100_rigidbody_ox"
Fb = np.array([0,0,0])
taub =  np.array([0,0,0])
unittest_template_A()

# Test initial conditions on ox
quad = rb.rigidbody_q(pos,q,vb,omegab+[0.1, 0, 0],mass,I)
name = "0101_rigidbody_q_ox"
Fb = np.array([0,0,0])
taub =  np.array([0,0,0])
unittest_template_A()

# Test initial conditions on oy
quad = rb.rigidbody(pos,rotmb2e,vb,omegab+[0, 0.1, 0],mass,I)
name = "0110_rigidbody_oy"
Fb = np.array([0,0,0])
taub = np.array([0,0,0])
unittest_template_A()

# Test initial conditions on oy
quad = rb.rigidbody_q(pos,q,vb,omegab+[0, 0.1, 0],mass,I)
name = "0111_rigidbody_q_oy"
Fb = np.array([0,0,0])
taub =  np.array([0,0,0])
unittest_template_A()

# Test initial conditions on oz
quad = rb.rigidbody(pos,rotmb2e,vb,omegab+[0, 0, 0.1],mass,I)
name = "0120_rigidbody_oz"
Fb = np.array([0,0,0])
taub =  np.array([0,0,0])
unittest_template_A()

# Test initial conditions on oz
quad = rb.rigidbody_q(pos,q,vb,omegab+[0, 0, 0.1],mass,I)
name = "0121_rigidbody_q_oz"
Fb = np.array([0,0,0])
taub =  np.array([0,0,0])
unittest_template_A()

# Test all
quad = rb.rigidbody(pos,rotmb2e,vb,omegab,mass,I)
name = "0130_rigidbody_all"
Fb = np.array([0.01,0.01,0.01])
taub =  np.array([0.01,0.01,0.01])
unittest_template_A()

# Test all
quad = rb.rigidbody_q(pos,q,vb,omegab,mass,I)
name = "0131_rigidbody_q_all"
Fb = np.array([0.01,0.01,0.01])
taub =  np.array([0.01,0.01,0.01])
unittest_template_A()
