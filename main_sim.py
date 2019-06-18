# -*- coding: utf-8 -*-
"""
Created on Mon Jun 17 20:47:53 2019

@author: Luminita-Cristiana Totu
"""

import numpy as np
from RigidBody import RigidBody
from Logger import Logger
from Plotter import Plotter

# Initialization values for the states of the quadrotor
pos = np.array([0,0,0])
Rb2e = np.identity(3)
Vb =  np.array([0,0,0])
Ob = np.array([0,0,0])
mass = 1
I = np.identity(3)

# Instantiate a rigid body object, a quadrotor 
quad = RigidBody(pos,Rb2e,Vb,Ob,mass,I)

# Initialize a logger
log = Logger("A","B")

# Simulation parameters
dt_sim = 0.01  # seconds
T_sim = 10     # seconds
dt_log = 1     # seconds

for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):
   
   # Control inputs to the quadrotor
   Fb = np.array([0,0,-10])
   taub =  np.array([0,0,0])
	 
   # Run the dynamic / time forward
   quad.run_quadrotor(dt_sim, Fb, taub)

   # Logging frequency    
   if ( abs(t % dt_log) < 0.00001 ):
      log.log_RigidBody(t, quad)        


log.print_RigidBody()

plot = Plotter()
plot.plot_RigidBody(log)