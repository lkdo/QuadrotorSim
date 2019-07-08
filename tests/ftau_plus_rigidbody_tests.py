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

""" Component testing for the rigidbody and quadftau_cf modules """

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

import numpy as np

from context import rb
from context import qftau_cf
from context import log  
from context import envir
from context import ut
from context import plot

def testcase_template_A():
    
    # Initialize the logger object 
    fullname = "testresults/ftau_plus_rigidbody/" + name
    logger = log.Logger(fullname, name)
    plotter = plot.Plotter()
    
    idx = 0
    
    # The main simulation loop     
    for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):
   
        # Calculate body-based forces and torques
        fb, taub = qftau.input2ftau(cmd[idx,:],qrb.vb)
        idx += 1 
    
        # Apply environment-based forces and torques 
        fe = np.dot(qrb.rotmb2e, fb)
        taue = np.dot(qrb.rotmb2e, taub)
    
        fe, taue = envir.applyenv2ftaue(fe, taue, qrb.mass)
    
        fb = np.dot(np.transpose(qrb.rotmb2e), fe)
        taub = np.dot(np.transpose(qrb.rotmb2e), taue)
    
        # Run the kinematic / time forward
        qrb.run_quadrotor(dt_sim, fb, taub)

        # Logging frequency    
        if abs(t/dt_log - round(t/dt_log)) < 0.000001 :
            qrb.check()
            logger.log_rigidbody(t, qrb)
            logger.log_cmd(t, cmd[idx,:])

    logger.log2file_rigidbody()
    logger.log2file_cmd()
    
    plotter.plot_rigidbody(logger)
    plotter.plot_cmd(logger)

# Initialization values for the states of the quadrotor
pos = np.array([0,0,0])
q = np.array([1,0,0,0])
rotmb2e = ut.quat2rotm(q) 
ve = np.array([0,0,0])
omegab = np.array([0,0,0])

# Simulation parameters
dt_sim = 0.001   # seconds
T_sim = 10       # seconds
dt_log = 1       # seconds

# Test Case 
#######################################################################
name = "0000_hoover"
cmd = 37278 * np.ones([round(T_sim/dt_sim)+10, 4])
index = int((5)/dt_sim) 
cmd[index:,:] = 0
qftau = qftau_cf.QuadFTau_CF(0)
qrb = rb.rigidbody_q(pos, q, ve, omegab, qftau.mass, qftau.I)
testcase_template_A()


# Test Case 
#######################################################################
name = "0010_roll_pos"
dt_log = 0.1
cmd = 37278 * np.ones([round(T_sim/dt_sim)+10, 4])
index = int((5)/dt_sim) 
cmd[index:,1] += 10
cmd[index:,3] -= 10
qftau = qftau_cf.QuadFTau_CF(0)
qrb = rb.rigidbody_q(pos, q, ve, omegab, qftau.mass, qftau.I)
testcase_template_A()

# Test Case 
#######################################################################
name = "0011_roll_neg"
dt_log = 0.1
cmd = 37278 * np.ones([round(T_sim/dt_sim)+10, 4])
index = int((5)/dt_sim) 
cmd[index:,1] -= 10
cmd[index:,3] += 10
qftau = qftau_cf.QuadFTau_CF(0)
qrb = rb.rigidbody_q(pos, q, ve, omegab, qftau.mass, qftau.I)
testcase_template_A()

# Test Case 
#######################################################################
name = "0020_pitch_pos"
dt_log = 0.1
cmd = 37278 * np.ones([round(T_sim/dt_sim)+10, 4])
index = int((5)/dt_sim) 
cmd[index:,0] -= 10
cmd[index:,2] += 10
qftau = qftau_cf.QuadFTau_CF(0)
qrb = rb.rigidbody_q(pos, q, ve, omegab, qftau.mass, qftau.I)
testcase_template_A()

# Test Case 
#######################################################################
name = "0021_pitch_neg"
dt_log = 0.1
cmd = 37278 * np.ones([round(T_sim/dt_sim)+10, 4])
index = int((5)/dt_sim) 
cmd[index:,0] += 10
cmd[index:,2] -= 10
qftau = qftau_cf.QuadFTau_CF(0)
qrb = rb.rigidbody_q(pos, q, ve, omegab, qftau.mass, qftau.I)
testcase_template_A()

# Test Case 
#######################################################################
name = "0031_yaw_pos"
dt_log = 0.1
cmd = 37278 * np.ones([round(T_sim/dt_sim)+10, 4])
index = int((5)/dt_sim) 
cmd[index:,0] += 10
cmd[index:,2] += 10
qftau = qftau_cf.QuadFTau_CF(0)
qrb = rb.rigidbody_q(pos, q, ve, omegab, qftau.mass, qftau.I)
testcase_template_A()

# Test Case 
#######################################################################
name = "0031_yaw_neg"
dt_log = 0.1
cmd = 37278 * np.ones([round(T_sim/dt_sim)+10, 4])
index = int((5)/dt_sim) 
cmd[index:,1] += 10
cmd[index:,3] += 10
qftau = qftau_cf.QuadFTau_CF(0)
qrb = rb.rigidbody_q(pos, q, ve, omegab, qftau.mass, qftau.I)
testcase_template_A()
