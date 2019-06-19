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

""" Module implements the rigid body kinematic via the RigidBody class

To add more text  
"""

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

import numpy as np
import utils as ut

class RigidBody:
    """ Holds the minimum number of states and parameters 
    to describe a Rigid Body, and implements the kinematics
    using rotation matrices and quaternions 
    Works with NED coordinates """
	
    def __init__(self,pos,Rb2e,Vb,Ob,mass,inertiaMatrix):
    
        self.pos = pos   # position vector, float in R3, meters
        self.Rb2e = Rb2e 
                    # rotation matrix from body to earth, float in R3x3
                    # ve = Rb2e*vb, 
                    #           where vb - vector in body frame
                    #                 ve - same vector in earth frame
        self.Vb = Vb # velocity vector, float in R3, m/s     
        self.Ob = Ob # angular velocity vector, float in R3, rad/s
        self.mass = mass  # body mass, in kg
        self.I = inertiaMatrix 
                    # Inertia Matrix in the body frame 
                    # I =  [ Ixx  -Ixy  -Ixz 
                    #       -Ixy   Iyy  -Iyz
                    #       -Ixz  -Iyz  Izz  ], where 
                    # Ixx, Iyy, Izz - Moments of Inertia around bodys
                    # own 3 axis 
                    #   Ixx = Integral_Volume (y*y + z*z) dm 
                    #   Iyy = Integral_Volume (x*x + z*z) dm 
                    #   Izz = Integral_Volume (x*x + y*y) dm
                    # Ixy, Ixz, Iyz - Products of Inertia
                    #   Ixy = Integral_Volume (xy) dm 
                    #   Ixz = Integral_Volume (xz) dm 
                    #   Iyz = Integral_Volume (yz) dm 
    
    def run_quadrotor(self,dt,Fb,taub):
      """ Dynamic/Differential equations for rigid body motion/flight """
    
      # dpos/dt = Ve = Rb2e*Vb 
      d_pos = np.dot(self.Rb2e,self.Vb)  
    
      # dRb2e/dt = Rb2e*Sks(Ob)
      d_Rb2e = np.dot(self.Rb2e,ut.SkS(self.Ob)) 
    
      # dVb/dt =  -Sks(Ob)*Vb + 1/m*Fb
      d_Vb = -np.dot(ut.SkS(self.Ob),self.Vb) + 1/self.mass*Fb 
    
      # dOb/dt = I^(-1)*(-SkS(Ob)*I*Ob + taub)
      d_Ob = ( np.dot(np.linalg.inv(self.I), 
               np.dot(-ut.SkS(self.Ob),np.dot(self.I,self.Ob)) + taub ) )
    
      ## Integrate for over dt
    
      # Simple Euler, the step dt must be small
      X = np.concatenate([self.pos, self.Rb2e.reshape(-1), self.Vb, self.Ob])
      dX = np.concatenate([ d_pos, d_Rb2e.reshape(-1), d_Vb, d_Ob ])
      X = X + dt*dX
  
      # unpack the vector state
      self.pos = X[1-1:3]
      self.Rb2e = X[4-1:12].reshape(3,3)
      self.Vb = X[13-1:15]
      self.Ob = X[16-1:18]

# ToDo: Write the dynamics in terms of the quaternion
#       instead of the rotation matrix	
#def run_quadrotor_quat(self,dt,Fb,taub):     	
	   