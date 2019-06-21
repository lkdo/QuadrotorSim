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

To add more text  
"""

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

import numpy as np

import qutils as ut

class rigidbody:
    """ Holds the minimum number of states and parameters 
    to describe a Rigid Body, and implements the kinematics
    using rotation matrices. Works with NED coordinates. 
    """
	
    def __init__(self,pos,rotmb2e,vb,omegab,mass,I):
        """ Initial values for the rigid body states."""
        
        self.pos = pos
        """ Position vector, float in R3, meters """
        
        self.rotmb2e = rotmb2e 
        """ rotation matrix from body to earth, float in R3x3
        ve = Rb2e*vb, 
        where vb - vector in body frame
        ve - same vector in earth frame
        """
        
        self.vb = vb 
        """ velocity vector, float in R3, m/s """     
       
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
         
    def run_quadrotor(self,dt,fb,taub):
        """ Dynamic/Differential equations for rigid body motion/flight """
    
        # dpos/dt = Ve = Rb2e*Vb 
        d_pos = np.dot(self.rotmb2e,self.vb)  
    
        # dRb2e/dt = Rb2e*Sks(Ob)
        d_rotmb2e = np.dot(self.rotmb2e,ut.skew(self.omegab)) 
    
        # dVb/dt =  -Sks(Ob)*Vb + 1/m*Fb
        d_vb = -np.dot(ut.skew(self.omegab),self.vb) + 1/self.mass*fb 
    
        # dOb/dt = I^(-1)*(-SkS(Ob)*I*Ob + taub)
        d_omegab = ( np.dot(
                            self.invI, 
                            np.dot(
                                   -ut.skew(self.omegab), 
                                   np.dot(self.I,self.omegab)
                                   )
                            + taub 
                            ) 
                   )
    
        ## Integrate for over dt
    
        # Simple Euler, the step dt must be small
        X = np.concatenate([self.pos, self.rotmb2e.reshape(-1), self.vb, 
                            self.omegab])
        dX = np.concatenate([ d_pos, d_rotmb2e.reshape(-1), d_vb, d_omegab ])
        X = X + dt*dX
  
        # unpack the vector state
        self.pos = X[1-1:3]
        self.rotmb2e = X[4-1:12].reshape(3,3)
        self.vb = X[13-1:15]
        self.omegab = X[16-1:18]
        
    def euler_xyz(self):
        """ Returns the 1-2-3/x-y-z Euler angles for E2B """
        return ut.rotm2exyz(np.transpose(self.rotmb2e))
   	
    def check(self):
        if ( abs(np.linalg.det(self.rotmb2e)-1)>0.001 or 
             np.linalg.norm(np.linalg.inv(self.rotmb2e) 
                            -np.transpose(self.rotmb2e)) >0.001 ) :
            print("Warning: determinant of rotation matrix is %f\n" % 
                  (np.linalg.det(self.rotmb2e)) 
                 )
            print("Warning: norm(inv(R)-R^T) is %f \n" % 
                  (np.linalg.det(self.rotmb2e)) 
                 )
            
class rigidbody_q:
    """ Holds the minimum number of states and parameters 
    to describe a Rigid Body, and implements the kinematics
    using quaternions. Works with NED coordinates. 
    """
    
    def __init__(self,pos,q,vb,omegab,mass,I):
    
        self.pos = pos   
        """ Position vector, float in R3, meters """
        
        self.q = q
        """ quaternion, in the form [ scalar vector3 ]   """
        
        self.vb = vb 
        """ velocity vector, float in R3, m/s """     
       
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
    
    def run_quadrotor(self,dt,fb,taub):
        """ Dynamic/Differential equations for rigid body motion/flight """
    
        # dpos/dt = ve = rotmb2e*vb 
        d_pos = np.dot(ut.quat2rotm(self.q),self.vb) 
    
        # q = [ s ] = [ s v1 v2 v3]^T
        #     [ v ]
        # dq/dt = 0.5*[      -v        ] * omegab
        #             [  sI3 + skew(v) ] 
        
        # version 1, unfolded - fastest, but still lower than rotm
        d_q=np.array(
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
        #d_q = 0.5*np.dot(np.block([[dq1],[dq2]]),self.omegab)
        #d_q = 0.5*np.dot(
        #        np.block([[-self.q[1:3+1]],
        #                  [self.q[0]*np.identity(3)+ut.skew(self.q[1:3+1])]]),
        #        self.omegab
        #                )
        
        # dvb/dt = -skew(omegab)*vb + 1/m*fb
        d_vb = -np.dot(ut.skew(self.omegab),self.vb) + 1/self.mass*fb 
    
        # domegab/dt = I^(-1)*(-skew(omegab)*I*omegabb + taub)
        d_omegab = (np.dot(
                       self.invI, 
                       np.dot(
                               -ut.skew(self.omegab),
                               np.dot(self.I,self.omegab)
                             )
                       + taub
                       ) 
               )
    
        ## Integrate for over dt
    
        # Simple Euler, the step dt must be small
        X = np.concatenate([self.pos, self.q, self.vb, self.omegab])
        dX = np.concatenate([ d_pos, d_q, d_vb, d_omegab ])
        X = X + dt*dX
  
        # unpack the vector state
        self.pos = X[1-1:3]
        self.q = X[4-1:7]
        self.vb = X[8-1:10]
        self.omegab = X[11-1:13]
    
    def euler_xyz(self):
        """ Returns the 1-2-3/x-y-z Euler angles for E2B """
        return ut.rotm2exyz(np.transpose(ut.quat2rotm(self.q)))
    
    def check(self):
        if abs(np.linalg.norm(self.q)-1)>0.001:
            print("Warning: norm of quaternions is %f\n" % 
                  (np.linalg.norm(self.q)) 
                 )