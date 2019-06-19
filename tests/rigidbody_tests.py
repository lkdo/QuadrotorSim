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

from context import rb
from context import plt
from context import log

# Initialization values for the states of the quadrotor
pos = np.array([0,0,0])
Rb2e = np.identity(3)
Vb =  np.array([0,0,0])
Ob = np.array([0,0,0])
mass = 1
I = np.identity(3)

# Simulation parameters
dt_sim = 0.01  # seconds
T_sim = 10     # seconds
dt_log = 1     # seconds

# Test zero forces and moments 
quad = rb.RigidBody(pos,Rb2e,Vb,Ob,mass,I)
logger = log.Logger("test_zero","B")
for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):

    # Control inputs to the quadrotor
    Fb = np.array([0,0,0])
    taub =  np.array([0,0,0])
	
    # Run the dynamic / time forward
    quad.run_quadrotor(dt_sim, Fb, taub)

    # Logging frequency    
    if ( abs(t % dt_log) < 0.00001 ):
        logger.log_RigidBody(t, quad) 

logger.print_RigidBody()
#######################################################################

# Test Fx force 
quad = rb.RigidBody(pos,Rb2e,Vb,Ob,mass,I)
logger = log.Logger("test_Fx","B")
for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):

    # Control inputs to the quadrotor
    Fb = np.array([0.1,0,0])
    taub =  np.array([0,0,0])
	
    # Run the dynamic / time forward
    quad.run_quadrotor(dt_sim, Fb, taub)

    # Logging frequency    
    if ( abs(t % dt_log) < 0.00001 ):
        logger.log_RigidBody(t, quad) 

logger.print_RigidBody()
#######################################################################

# Test Fy force 
quad = rb.RigidBody(pos,Rb2e,Vb,Ob,mass,I)
logger = log.Logger("test_Fy","B")
for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):

    # Control inputs to the quadrotor
    Fb = np.array([0,0.1,0])
    taub =  np.array([0,0,0])
	
    # Run the dynamic / time forward
    quad.run_quadrotor(dt_sim, Fb, taub)

    # Logging frequency    
    if ( abs(t % dt_log) < 0.00001 ):
        logger.log_RigidBody(t, quad) 

logger.print_RigidBody()
#######################################################################

# Test Fz force
quad = rb.RigidBody(pos,Rb2e,Vb,Ob,mass,I)
logger = log.Logger("test_Fz","B")
for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):

    # Control inputs to the quadrotor
    Fb = np.array([0,0,0.1])
    taub =  np.array([0,0,0])
	
    # Run the dynamic / time forward
    quad.run_quadrotor(dt_sim, Fb, taub)

    # Logging frequency    
    if ( abs(t % dt_log) < 0.00001 ):
        logger.log_RigidBody(t, quad) 

logger.print_RigidBody()
#######################################################################

# Test tau_x moment
quad = rb.RigidBody(pos,Rb2e,Vb,Ob,mass,I)
logger = log.Logger("test_taux","B")
for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):

    # Control inputs to the quadrotor
    Fb = np.array([0,0,0])
    taub =  np.array([0.1,0,0])
	
    # Run the dynamic / time forward
    quad.run_quadrotor(dt_sim, Fb, taub)

    # Logging frequency    
    if ( abs(t % dt_log) < 0.00001 ):
        logger.log_RigidBody(t, quad) 

logger.print_RigidBody()
#######################################################################

# Test tau_y moment
quad = rb.RigidBody(pos,Rb2e,Vb,Ob,mass,I)
logger = log.Logger("test_tauy","B")
for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):

    # Control inputs to the quadrotor
    Fb = np.array([0,0,0])
    taub =  np.array([0,0.1,0])
	
    # Run the dynamic / time forward
    quad.run_quadrotor(dt_sim, Fb, taub)

    # Logging frequency    
    if ( abs(t % dt_log) < 0.00001 ):
        logger.log_RigidBody(t, quad) 

logger.print_RigidBody()
#######################################################################

# Test tau_z moment
quad = rb.RigidBody(pos,Rb2e,Vb,Ob,mass,I)
logger = log.Logger("test_tauz","B")
for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):

    # Control inputs to the quadrotor
    Fb = np.array([0,0,0])
    taub =  np.array([0,0,0.1])
	
    # Run the dynamic / time forward
    quad.run_quadrotor(dt_sim, Fb, taub)

    # Logging frequency    
    if ( abs(t % dt_log) < 0.00001 ):
        logger.log_RigidBody(t, quad) 

logger.print_RigidBody()
#######################################################################

# Test initial conditions on Vx
quad = rb.RigidBody(pos,Rb2e,Vb+[0.1, 0, 0],Ob,mass,I)
logger = log.Logger("test_Vx","B")
for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):

    # Control inputs to the quadrotor
    Fb = np.array([0,0,0])
    taub =  np.array([0,0,0])
	
    # Run the dynamic / time forward
    quad.run_quadrotor(dt_sim, Fb, taub)

    # Logging frequency    
    if ( abs(t % dt_log) < 0.00001 ):
        logger.log_RigidBody(t, quad) 

logger.print_RigidBody()
#######################################################################

# Test initial conditions on Vy
quad = rb.RigidBody(pos,Rb2e,Vb+[0, 0.1, 0],Ob,mass,I)
logger = log.Logger("test_Vy","B")
for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):

    # Control inputs to the quadrotor
    Fb = np.array([0,0,0])
    taub =  np.array([0,0,0])
	
    # Run the dynamic / time forward
    quad.run_quadrotor(dt_sim, Fb, taub)

    # Logging frequency    
    if ( abs(t % dt_log) < 0.00001 ):
        logger.log_RigidBody(t, quad) 

logger.print_RigidBody()
#######################################################################

# Test initial conditions on Vz
quad = rb.RigidBody(pos,Rb2e,Vb+[0, 0, 0.1],Ob,mass,I)
logger = log.Logger("test_Vz","B")
for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):

    # Control inputs to the quadrotor
    Fb = np.array([0,0,0])
    taub =  np.array([0,0,0])
	
    # Run the dynamic / time forward
    quad.run_quadrotor(dt_sim, Fb, taub)

    # Logging frequency    
    if ( abs(t % dt_log) < 0.00001 ):
        logger.log_RigidBody(t, quad) 

logger.print_RigidBody()
#######################################################################

# Test initial conditions on ox
quad = rb.RigidBody(pos,Rb2e,Vb,Ob+[0.1, 0, 0],mass,I)
logger = log.Logger("test_ox","B")
for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):

    # Control inputs to the quadrotor
    Fb = np.array([0,0,0])
    taub =  np.array([0,0,0])
	
    # Run the dynamic / time forward
    quad.run_quadrotor(dt_sim, Fb, taub)

    # Logging frequency    
    if ( abs(t % dt_log) < 0.00001 ):
        logger.log_RigidBody(t, quad) 

logger.print_RigidBody()
#######################################################################

# Test initial conditions on oy
quad = rb.RigidBody(pos,Rb2e,Vb,Ob+[0, 0.1, 0],mass,I)
logger = log.Logger("test_oy","B")
for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):

    # Control inputs to the quadrotor
    Fb = np.array([0,0,0])
    taub =  np.array([0,0,0])
	
    # Run the dynamic / time forward
    quad.run_quadrotor(dt_sim, Fb, taub)

    # Logging frequency    
    if ( abs(t % dt_log) < 0.00001 ):
        logger.log_RigidBody(t, quad) 

logger.print_RigidBody()
#######################################################################

# Test initial conditions on oz
quad = rb.RigidBody(pos,Rb2e,Vb,Ob+[0, 0, 0.1],mass,I)
logger = log.Logger("test_oz","B")
for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):

    # Control inputs to the quadrotor
    Fb = np.array([0,0,0])
    taub =  np.array([0,0,0])
	
    # Run the dynamic / time forward
    quad.run_quadrotor(dt_sim, Fb, taub)

    # Logging frequency    
    if ( abs(t % dt_log) < 0.00001 ):
        logger.log_RigidBody(t, quad) 

logger.print_RigidBody()
#######################################################################
