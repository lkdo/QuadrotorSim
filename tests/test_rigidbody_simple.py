# -*- coding: utf-8 -*-
""" Test Cases for the Rigid Body class implementing basic kinematics """

""" Part of the QuadrotorSim package
Copyright (C) 2019  Luminita-Cristiana Totu
Contact: luminita.totu@gmail.com

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this package, in a file called LICENSE.  
If not, see <https://www.gnu.org/licenses/>.
"""

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

# One time initializations

# Initialize a plotter
plot = plt.Plotter()

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



# Test Fx force 
quad = rb.RigidBody(pos,Rb2e,Vb,Ob,mass,I)
logger = log.Logger("test_Fx","B")
for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):
   
   # Control inputs to the quadrotor
   Fb = np.array([1,0,0])
   taub =  np.array([0,0,0])
	 
   # Run the dynamic / time forward
   quad.run_quadrotor(dt_sim, Fb, taub)

   # Logging frequency    
   if ( abs(t % dt_log) < 0.00001 ):
      logger.log_RigidBody(t, quad)        
logger.print_RigidBody()



# Test Fy force 
quad = rb.RigidBody(pos,Rb2e,Vb,Ob,mass,I)
logger = log.Logger("test_Fy","B")
for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):
   
   # Control inputs to the quadrotor
   Fb = np.array([0,1,0])
   taub =  np.array([0,0,0])
	 
   # Run the dynamic / time forward
   quad.run_quadrotor(dt_sim, Fb, taub)

   # Logging frequency    
   if ( abs(t % dt_log) < 0.00001 ):
      logger.log_RigidBody(t, quad)        
logger.print_RigidBody()


# Test Fz force 
quad = rb.RigidBody(pos,Rb2e,Vb,Ob,mass,I)
logger = log.Logger("test_Fy","B")
for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):
   
   # Control inputs to the quadrotor
   Fb = np.array([0,0,1])
   taub =  np.array([0,0,0])
	 
   # Run the dynamic / time forward
   quad.run_quadrotor(dt_sim, Fb, taub)

   # Logging frequency    
   if ( abs(t % dt_log) < 0.00001 ):
      logger.log_RigidBody(t, quad)
logger.print_RigidBody()


# Test tau_x moment 
quad = rb.RigidBody(pos,Rb2e,Vb,Ob,mass,I)
logger = log.Logger("test_ox","B")
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


# Test tau_y moment 
quad = rb.RigidBody(pos,Rb2e,Vb,Ob,mass,I)
logger = log.Logger("test_oy","B")
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


# Test tau_z moment 
quad = rb.RigidBody(pos,Rb2e,Vb,Ob,mass,I)
logger = log.Logger("test_oz","B")
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

# Test initial conditions on Vx
quad = rb.RigidBody(pos,Rb2e,Vb+[1,0,0],Ob,mass,I)
logger = log.Logger("test_vx","B")
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
 
# Test initial conditions on Vy
quad = rb.RigidBody(pos,Rb2e,Vb+[0,1,0],Ob,mass,I)
logger = log.Logger("test_vy","B")
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

# Test initial conditions on Vz
quad = rb.RigidBody(pos,Rb2e,Vb+[0,0,1],Ob,mass,I)
logger = log.Logger("test_vz","B")
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

# Test initial conditions on ox
quad = rb.RigidBody(pos,Rb2e,Vb,Ob+[0.1,0,0],mass,I)
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

# Test initial conditions on oy
quad = rb.RigidBody(pos,Rb2e,Vb,Ob+[0,0.1,0],mass,I)
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

# Test initial conditions on oz
quad = rb.RigidBody(pos,Rb2e,Vb,Ob+[0,0,0.1],mass,I)
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
