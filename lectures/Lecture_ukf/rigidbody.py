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

import ctypes
import numpy as np
import envir
import utils
import math
from scipy.integrate import odeint
from numpy.linalg import inv


def quadrotor_dt_euler(X, t, velb, omegab):
    
    cr = math.cos(X[3])
    sr = math.sin(X[3])
    cp = math.cos(X[4])
    sp = math.sin(X[4])
    cy = math.cos(X[5])
    sy = math.sin(X[5])
    
    Einv = 1.0/cp*np.array([ [ cp, sr*sp, cr*sp ], [ 0, cr*cp, -sr*cp ], [ 0, sr, cr ] ])
    
    Reb = utils.rpy2rotm(X[3:6])
    
    d_pos = Reb@velb
    d_rpy = Einv@omegab

    return np.concatenate([ d_pos, d_rpy ])
##########################################################

def quadrotor_dt_rotm(X, t, velb, omegab):
    
    Reb = X[3:12].reshape([3,3])
    
    d_pos = Reb@velb
    d_Reb = (Reb@utils.skew(omegab)).flatten()
    
    return np.concatenate([ d_pos, d_Reb ])
##########################################################    


def quadrotor_meas(X):
    pos = X[0:3]
    return pos
##########################################################  

def quadrotor_dt_simple(X, t, velb, omegab):
    
    q = X[3:7] / np.linalg.norm(X[3:7])  #update and normalize 
    rotmb2e = utils.quat2rotm(q)
    d_pos = rotmb2e@velb

    # q = [ s ] = [ s v1 v2 v3]^T
    #     [ v ]
    # dq/dt = 0.5* [      -v^T             ] 
    #              [  sI3 + skew(v)        ] * omegab

    # version 1, unfolded - fastest, but still lower than rotm    
    d_q=np.array(
        [(-q[1]*omegab[0]
             -q[2]*omegab[1]-q[3]*omegab[2]),
          (q[0]*omegab[0]
              -q[3]*omegab[1]+q[2]*omegab[2]),
          (q[3]*omegab[0]
              +q[0]*omegab[1]-q[1]*omegab[2]),
          (-q[2]*omegab[0]
              +q[1]*omegab[1]+q[0]*omegab[2])
        ])*0.5

    # version 2, more compact but slower than version 1
    #d_q = 0.5*np.dot(
    #        np.block([[-q[1:3+1]],
    #                  [q[0]*np.identity(3)+utils.skew(q[1:3+1])]]),
    #        omegab
    #                )

    #version 3
    #d_q = 0.5 * utils.quaternion_multiply(np.array([0,omegab[0],omegab[1],omegab[2]]),q)

    return np.concatenate([ d_pos, d_q ])
##############################################################################

def quadrotor_dt(X, t, mass, I, invI, fb, taub):
    
    q = X[3:7] / np.linalg.norm(X[3:7])  #update and normalize 
    ve = X[7:10]
    omegab = X[10:13]

    d_pos = ve

    # q = [ s ] = [ s v1 v2 v3]^T
    #     [ v ]
    # dq/dt = 0.5* [      -v^T             ] 
    #              [  sI3 + skew(v)        ] * omegab

    # version 1, unfolded - fastest, but still lower than rotm    
    d_q=np.array(
        [(-q[1]*omegab[0]
             -q[2]*omegab[1]-q[3]*omegab[2]),
          (q[0]*omegab[0]
              -q[3]*omegab[1]+q[2]*omegab[2]),
          (q[3]*omegab[0]
              +q[0]*omegab[1]-q[1]*omegab[2]),
          (-q[2]*omegab[0]
              +q[1]*omegab[1]+q[0]*omegab[2])
        ])*0.5

    # dve/dt = 1/m*Rbe*fb + g
    d_ve = 1/mass*utils.quat2rotm(q)@fb + np.array([0,0,-envir.g ])

    # domegab/dt = I^(-1)*(-skew(omegab)*I*omegabb + taub)
    d_omegab = (np.dot(
                   invI, 
                   np.dot(
                           -utils.skew(omegab),
                           np.dot(I,omegab)
                         )
                   + taub
                   ) 
           )

    return np.concatenate([ d_pos, d_q, d_ve, d_omegab ])
##########################################################    

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
        
        self.rpy = utils.rotm2rpy(utils.quat2rotm(self.q),1) 
        
        self.ve = ve 
        """ velocity vector in earth frame, float in R3, m/s """     
       
        self.vb = np.transpose(self.rotmb2e)@self.ve
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
    
        # ODE Integration
        X = np.concatenate([self.pos, self.q, self.ve, self.omegab])
        Y = odeint(quadrotor_dt,X,np.array([0, dt]),args=(self.mass,self.I,self.invI,fb,taub,))
        Y = Y[1] # X[0]=x(t=t0) 

        # unpack the vector state
        self.pos = Y[1-1:3]
        self.q = Y[4-1:7] / np.linalg.norm(Y[4-1:7])  #update and normalize 
        self.rotmb2e = utils.quat2rotm(self.q)
        self.rpy = utils.rotm2rpy(self.rotmb2e)
        self.ve = Y[8-1:10]
        self.vb = np.transpose(self.rotmb2e)@self.ve
        self.omegab = Y[11-1:13]

    def run_quadrotor_simple(self,dt,vb,omegab):
        """ Dynamic/Differential equations for rigid body motion/flight """
    
        # ODE Integration
        X = np.concatenate([self.pos, self.q ])
        Y = odeint(quadrotor_dt_simple,X,np.array([0, dt]),args=(vb,omegab,))
        Y = Y[1] # X[0]=x(t=t0) 

        # unpack the vector state
        self.pos = Y[1-1:3]
        self.q = Y[4-1:7] / np.linalg.norm(Y[4-1:7])  #update and normalize 
        self.rotmb2e = utils.quat2rotm(self.q)
        self.rpy = utils.rotm2rpy(self.rotmb2e)
        #self.ve = self.rotmb2e*vb
        #self.vb = vb
        #self.omegab = omegab

    def run_quadrotor_rotm(self,dt,vb,omegab):
        """ Dynamic/Differential equations for rigid body motion/flight """
    
        # ODE Integration
        rf = self.rotmb2e.flatten() 
        X = np.concatenate([self.pos, rf])
        Y = odeint(quadrotor_dt_rotm,X,np.array([0, dt]),args=(vb,omegab,))
        Y = Y[1] # X[0]=x(t=t0) 

        # unpack the vector state
        self.pos = Y[1-1:3]
        self.rotmb2e = Y[3:12].reshape([3,3])
        #normalize the rotation matrix
        self.rotmb2e = utils.cay((np.transpose(self.rotmb2e) - self.rotmb2e)/(1.0+np.trace(self.rotmb2e))) 
        self.rpy = utils.rotm2rpy(self.rotmb2e)
        self.q = utils.rpy2q(self.rpy) 
        #self.ve = self.rotmb2e*vb
        #self.vb = vb
        #self.omegab = omegab
    