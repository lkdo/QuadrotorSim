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
import math
from scipy.integrate import odeint
from math import sin, cos

def quadrotor_dt_kinematic_euler(X, t, u):
    # X = [pos,euler,ve]
    
    Reb = utils.rpy2rotm(X[3:6])
    
    ab = (u[3:]) + Reb.transpose()@np.array([0,0,-envir.g])
    omegab = u[:3]

    cr = math.cos(X[3])
    sr = math.sin(X[3])
    cp = math.cos(X[4])
    sp = math.sin(X[4])
    Einv = 1.0/cp*np.array([ [ cp, sr*sp, cr*sp ], [ 0, cr*cp, -sr*cp ], [ 0, sr, cr ] ])
    
    d_pos = X[6:9]
    d_rpy = Einv@omegab
    d_ve = Reb@ab

    return np.concatenate([ d_pos, d_rpy, d_ve ])
##########################################################

def quadrotor_dt_kinematic_euler_vb(X, t, u):
    # X = [pos,euler,vb]
    
    Reb = utils.rpy2rotm(X[3:6])
    
    ab = (u[3:]) + Reb.transpose()@np.array([0,0,-envir.g])
    omegab = u[:3]
    vb = X[6:9]

    cr = math.cos(X[3])
    sr = math.sin(X[3])
    cp = math.cos(X[4])
    sp = math.sin(X[4])
    Einv = 1.0/cp*np.array([ [ cp, sr*sp, cr*sp ], [ 0, cr*cp, -sr*cp ], [ 0, sr, cr ] ])
    
    d_pos = Reb@vb
    d_rpy = Einv@omegab
    d_vb = np.transpose(utils.skew(omegab))@vb + ab

    return np.concatenate([ d_pos, d_rpy, d_vb ])
##########################################################

def quadrotor_dt_dynamic_quat(X, t, mass, I, invI, fb, taub):
    # Implemented here using quaternions
    # X = [pos, qeb, ve, omegab]

    q = X[3:7] / np.linalg.norm(X[3:7])  # update and normalize 
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
    
    def __init__(self,pos,q,ve,omegab,ab,mass,I):
    
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
        """ angular velocity vector, float in R3, rad/s"""
        
        self.abmg = ab
        """ body frame acceleration """
        
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
        
    def run_quadrotor_dynamic_quat(self,dt,fb,taub):
        """ Dynamic/Differential equations for rigid body motion/flight """
    
        # ODE Integration
        X = np.concatenate([self.pos, self.q, self.ve, self.omegab])
        Y = odeint(quadrotor_dt_dynamic_quat,X,np.array([0, dt]),args=(self.mass,self.I,self.invI,fb,taub,))
        Y = Y[1] # Y[0] = X(t=t0) 

        # unpack the vector state
        self.pos = Y[1-1:3]
        self.q = Y[4-1:7] / np.linalg.norm(Y[4-1:7])  # update and normalize 
        self.rotmb2e = utils.quat2rotm(self.q)
        self.rpy = utils.rotm2rpy(self.rotmb2e)
        self.ve = Y[8-1:10]
        self.vb = np.transpose(self.rotmb2e)@self.ve
        self.omegab = Y[11-1:13]
        
        # Get the acceleration, need it for accelerometer sensors
        dvemg = quadrotor_dt_dynamic_quat(Y, 0, self.mass, self.I, self.invI, fb, taub)[7:10] - np.array([0,0,-envir.g])  # minus gravity 
        self.abmg = np.transpose(self.rotmb2e)@(dvemg)
##########################################################       

def quadrotor_dt_kinematic_euler_vb_dFXdX(X,u):

    # X = [pos, euler, vb ]
    # H = [pos, vb ]
  
    # util exprs
    roll = X[3];  pitch = X[4]; yaw = X[5]
    vbx = X[6]; vby = X[7];  vbz = X[8]
    ox = u[0]; oy = u[1]; oz = X[2]

    sr = sin(roll)
    cr = cos(roll)
    sp = sin(pitch)
    cp = cos(pitch)
    sy = sin(yaw)
    cy = cos(yaw)
    cpi = 1.0/cp
    tp = sp/cp
 
    # From symbolic
    return np.array([
[0 , 0 , 0 , vby*(sp*cr*cy + sr*sy) + vbz*(-sp*sr*cy + sy*cr) , -vbx*sp*cy + vby*sr*cp*cy + vbz*cp*cr*cy , -vbx*sy*cp + vby*(-sp*sr*sy - cr*cy) + vbz*(-sp*sy*cr + sr*cy) , cp*cy , sp*sr*cy - sy*cr , sp*cr*cy + sr*sy], 
[0 , 0 , 0 , vby*(sp*sy*cr - sr*cy) + vbz*(-sp*sr*sy - cr*cy) , -vbx*sp*sy + vby*sr*sy*cp + vbz*sy*cp*cr , vbx*cp*cy + vby*(sp*sr*cy - sy*cr) + vbz*(sp*cr*cy + sr*sy) , sy*cp , sp*sr*sy + cr*cy , sp*sy*cr - sr*cy], 
[0 , 0 , 0 , vby*cp*cr - vbz*sr*cp , -vbx*cp - vby*sp*sr - vbz*sp*cr , 0 , -sp , sr*cp , cp*cr ],
[0 , 0 , 0 , oy*tp*cr - oz*tp*sr , oy*sr*tp**2 + oy*sr + oz*cr*tp**2 + oz*cr , 0 , 0 , 0 , 0 ], 
[0 , 0 , 0 , -oy*sr - oz*cr , 0 , 0 , 0 , 0 , 0 ],
[0 , 0 , 0 , oy*cr*cpi - oz*sr*cpi , oy*tp*sr*cpi + oz*tp*cr*cpi , 0 , 0 , 0 , 0], 
[0 , 0 , 0 , 0 , 0 , 0 , 0 , oz , -oy ], 
[0 , 0 , 0 , 0 , 0 , 0 , -oz , 0 , ox ],
[0 , 0 , 0 , 0 , 0 , 0 , oy , -ox , 0 ]
        ])

def quadrotor_dt_kinematic_euler_vb_meas_pos(X):
    # X = [pos, euler, vb ]
    # H = [pos]
    H1 = X[0:3]
    return H1

def quadrotor_dt_kinematic_euler_vb_meas_pos_dHXdX(X):
    return np.array([
      [1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ],
      [0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ], 
      [0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 ] ])

def quadrotor_dt_kinematic_euler_vb_meas_vb(X):
    # X = [pos, euler, vb, ob, ae]
    # H = [vb]
    H2 = X[6:9]
    return H2

def quadrotor_dt_kinematic_euler_vb_meas_vb_dHXdX(X):
    return np.array([
[0 , 0 , 0 , 0 , 0 , 0 , 1 , 0 , 0], 
[0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 , 0],
[0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 1] 
 ])
