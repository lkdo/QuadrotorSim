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

""" Module implements the rigid body kinematic via the RigidBody classes

"""

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

import numpy as np

import envir
import utils 

class rigidbody:
    """ Holds the states and parameters to describe a Rigid Body, 
    and implements the kinematics  using quaternions
    """
    
    def __init__(self,pos,q,ve,omegab,mass,I):
    
        self.pos = pos   
        """ Position vector, float in R3, meters """
        
        self.q = q
        """ quaternion, in the form [ scalar vector3 ]   """
        
        self.rotmb2e = utils.quat2rotm(q)
        """ the rotation matrix """
        
        self.rpy = utils.rotm2exyz(utils.quat2rotm(self.q),1) 
        
        self.ve = ve 
        """ velocity vector in earth frame, float in R3, m/s """     
       
        self.vb =np.transpose(self.rotmb2e)@self.ve
        """ velocity vector in body frame, float in R3, m/s """     
       
        self.omegab = omegab 
        """angular velocity vector, float in R3, rad/s"""
        
        self.mass = mass  
        """ body mass, in kg """
        
        self.I = I
        """ Inertia Matrix in the body frame 
        I =  [ Ixx  -Ixy  -Ixz 
              -Ixy   Iyy  -Iyz
              -Ixz  -Iyz  Izz  ], 
        where
        Ixx, Iyy, Izz - Moments of Inertia around body'sown 3-axes
             Ixx = Integral_Volume (y*y + z*z) dm 
             Iyy = Integral_Volume (x*x + z*z) dm 
             Izz = Integral_Volume (x*x + y*y) dm
        Ixy, Ixz, Iyz - Products of Inertia
             Ixy = Integral_Volume (xy) dm 
             Ixz = Integral_Volume (xz) dm 
             Iyz = Integral_Volume (yz) dm 
        """
        self.invI = np.linalg.inv(self.I)
    
        self.d_pos = np.zeros(3)
        self.d_q = np.zeros(4)
        self.d_ve = np.zeros(3)
        self.d_omegab = np.zeros(3)
        
    def run_quadrotor(self,dt,fb,taub):
        """ Dynamic/Differential equations for rigid body motion/flight """
    
        # dpos/dt = ve 
        self.d_pos = self.ve
    
        # q = [ s ] = [ s v1 v2 v3]^T
        #     [ v ]
        # dq/dt = 0.5*[      -v              ] 
        #                  [  sI3 + skew(v) ] * omegab
        
        # version 1, unfolded - fastest, but still lower than rotm
        self.d_q=np.array(
            [(-self.q[1]*self.omegab[0]
                 -self.q[2]*self.omegab[1]-self.q[3]*self.omegab[2]),
              (self.q[0]*self.omegab[0]
                  -self.q[3]*self.omegab[1]+self.q[2]*self.omegab[2]),
              (self.q[3]*self.omegab[0]
                  +self.q[0]*self.omegab[1]-self.q[1]*self.omegab[2]),
              (-self.q[2]*self.omegab[0]
                  +self.q[1]*self.omegab[1]+self.q[0]*self.omegab[2])
            ])*0.5
        
        # version 2, more compact but slower than version 1
        #d_q = 0.5*np.dot(
        #        np.block([[-self.q[1:3+1]],
        #                  [self.q[0]*np.identity(3)+ut.skew(self.q[1:3+1])]]),
        #        self.omegab
        #                )
        
        # dve/dt = 1/m*Rbe*fb + g
        self.d_ve = 1/self.mass*utils.quat2rotm(self.q)@fb + np.array([0,0,-envir.g ])
    
        # domegab/dt = I^(-1)*(-skew(omegab)*I*omegabb + taub)
        self.d_omegab = (np.dot(
                       self.invI, 
                       np.dot(
                               -utils.skew(self.omegab),
                               np.dot(self.I,self.omegab)
                             )
                       + taub
                       ) 
               )
    
        # Integrate for over dt
        # Simple Euler, the step dt must be small
        X = np.concatenate([self.pos, self.q, self.ve, self.omegab])
        dX = np.concatenate([ self.d_pos, self.d_q, self.d_ve, self.d_omegab ])
        X = X + dt*dX
  
        # unpack the vector state
        self.pos = X[1-1:3]
        self.q = X[4-1:7] / np.linalg.norm(X[4-1:7])  #update and normalize 
        self.rotmb2e = utils.quat2rotm(self.q)
        self.rpy = utils.rotm2exyz(self.rotmb2e)
        self.ve = X[8-1:10]
        self.vb =np.transpose(self.rotmb2e)@self.ve
        self.omegab = X[11-1:13]
        
            