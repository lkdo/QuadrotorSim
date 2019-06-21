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

""" Utility functions """

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"
	
import numpy as np
import math

def skew(X):
    """  Returns the skew-symmetric matrix form of the input vector """
        
    return np.array([[0,-X[3-1],X[2-1]],[X[3-1],0,-X[1-1]],[-X[2-1],X[1-1],0]])

def rotm2exyz(R):
    """ Transforms a rotation matrix to Euler X-Y-Z (1-2-3) angles
    
    That is input matrix R = Rz(phi)*Ry(theta)*Rx(psi), 
    and the output is [psi, theta, phi]. 
    Implements the pseudocode from
    "Computing Euler angles from a rotation matrix", 
    by Gregory G. Slabaugh
    """
    
    if R[3-1,1-1]!=1 and R[3-1,1-1]!=-1:
        theta1 = math.asin(R[3-1,1-1])
        theta2 = math.pi - theta1
        psi1 = math.atan2(R[3-1,2-1]/math.cos(theta1),
                          R[3-1,3-1]/math.cos(theta1))
        psi2 = math.atan2(R[3-1,2-1]/math.cos(theta2),
                          R[3-1,3-1]/math.cos(theta2))
        phi1 = math.atan2(R[2-1,1-1]/math.cos(theta1),
                          R[1-1,1-1]/math.cos(theta1))
        phi2 = math.atan2(R[2-1,1-1]/math.cos(theta2),
                          R[1-1,1-1]/math.cos(theta2))
       
        # Choose one set of rotations
        if  theta1 >= 0:
            return np.array([psi1,theta1,phi1])
        else:
            return np.array([psi2,theta2,phi2])        
    else:
        phi = 0 # can be anything 
        if R[3-1,1-1] == -1:
            theta = math.pi/2
            psi = phi + math.atan2(R[1-1,2-1],R[1-1,3-1])
        else:
            theta = -math.pi/2
            psi = -phi + math.atan2(-R[1-1,2-1],R[1-1,3-1])
        return np.array([psi,theta,phi])

def quat2rotm(q): 
    """ Takes a quaternion and returns the rotation matrix"""
    
    return np.array([
                      [ q[0]**2+q[1]**2-q[2]**2-q[3]**2,
                       2*(q[1]*q[2]-q[0]*q[3]), 
                       2*(q[1]*q[3]+q[0]*q[2])   ],
                      [ 2*(q[1]*q[2]+q[0]*q[3]),
                        (q[0]**2-q[1]**2+q[2]**2-q[3]**2),
                        2*(-q[0]*q[1]+q[2]*q[3]) ],
                      [ 2*(q[1]*q[3]-q[0]*q[2]), 
                        2*(q[2]*q[3]+q[0]*q[1]),
                        q[0]**2+q[3]**2-q[2]**2-q[1]**2 ] 
                    ])  
 
class FirstOrderSystem:
    """ implements a first order system 
    
    dx(t)/dt = a*x(t) + b*u(t)
    """
    
    def __init__(self, a, b, x0):
        
        self.a = a
        self.b = b
        self.x = x0
         
    def run(self, u, dt):
         
        dx = self.a*self.x + self.b*u
        self.x = self.x + dx*dt