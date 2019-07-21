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

""" Useful functions 

"""

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

import numpy as np
import math

def rotm2exyz(R, sol=1):
    """ Transforms a rotation matrix to Euler X-Y-Z (1-2-3) angles
    
    That is input matrix R = Rz(phi)*Ry(theta)*Rx(psi), 
    and the output is [psi, theta, phi]. 
    Implements the pseudocode from
    "Computing Euler angles from a rotation matrix", 
    by Gregory G. Slabaugh
    """
    
    if R[3-1,1-1]!=1 and R[3-1,1-1]!=-1:
        theta1 = -math.asin(R[3-1,1-1])
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
        if sol == 1:
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
##########################################################    


def quat2rotm(q): 
    """ Takes a quaternion and returns the rotation matrix"""
    
    return 2*np.array([
                      [ q[0]**2+q[1]**2-0.5,
                       (q[1]*q[2]-q[0]*q[3]), 
                       (q[1]*q[3]+q[0]*q[2])   ],
                      [ (q[1]*q[2]+q[0]*q[3]),
                        (q[0]**2+q[2]**2-0.5),
                        (-q[0]*q[1]+q[2]*q[3]) ],
                      [ (q[1]*q[3]-q[0]*q[2]), 
                        (q[2]*q[3]+q[0]*q[1]),
                        q[0]**2+q[3]**2-0.5 ] 
                    ])  
##########################################################

    
def skew(X):
    """  Returns the skew-symmetric matrix form of the input vector """
    return np.array([[0,-X[3-1],X[2-1]],[X[3-1],0,-X[1-1]],[-X[2-1],X[1-1],0]])
##########################################################
    

def build_signal_step(t_start, t_end, base_value, *arg):

     small = 0.00001
     n = len(arg)
     
     ref = np.zeros([2+4*n,2])
     
     k = 0
     ref[k,0] = t_start
     ref[k,1] = base_value
     k += 1
     
     for i in range(n):
         
         ref[k,0] = arg[i].t_start - small
         ref[k,1] = base_value
         k += 1
     
         ref[k,0] = arg[i].t_start
         ref[k,1] = base_value + arg[i].value
         k += 1
         
         ref[k,0] = arg[i].t_end 
         ref[k,1] = base_value + arg[i].value
         k += 1
         
         ref[k,0] = arg[i].t_end + small 
         ref[k,1] = base_value
         k += 1 

     ref[k,0] = t_end
     ref[k,1] = base_value
     k += 1

     return ref 
 
def build_signal_ramp(t_start, t_end, base_value, *arg):

     n = len(arg)
     
     ref = np.zeros([2+3*n,2])
     
     k = 0
     ref[k,0] = t_start
     ref[k,1] = base_value
     k += 1
     
     for i in range(n):
         
         ref[k,0] = arg[i].t_start 
         ref[k,1] = base_value
         k += 1
     
         ref[k,0] = arg[i].t_start + (arg[i].t_end - arg[i].t_start)/2
         ref[k,1] = base_value+arg[i].value
         k += 1
         
         ref[k,0] = arg[i].t_end 
         ref[k,1] = base_value
         k += 1 

     ref[k,0] = t_end
     ref[k,1] = base_value
     k += 1

     return ref     
    
    
def build_signal_sin(t_start, t_end, base_value, *arg):

    n = len(arg)
    
    N = 2
    for i in range(n):
        N += arg[i].no_periods*arg[i].no_points_per_period
    
    ref = np.zeros([N,2])
    
    k = 0
    ref[k,0] = t_start
    ref[k,1] = base_value
    k += 1
     
    for i in range(n):
         x = np.linspace(0,arg[i].no_periods*arg[i].period,arg[i].no_periods*arg[i].no_points_per_period)
         sinx = base_value + arg[i].amplitude*np.sin(2*math.pi/arg[i].period*x)
         ref[k:k+len(x),0]  = arg[i].t_start + x
         ref[k:k+len(x),1]  = sinx 
         k = k +len(x)
         
    ref[k,0] = t_end
    ref[k,1] = base_value
    k += 1
    
    return ref
    
def give_signal(ref, t):
    
    k = 0
    n = len(ref)
    while (ref[k,0] < t) and (k < n-1):
        k+=1 
    
    if (k==0) or (k == n):
        return ref[k,1] 
    else:
        return( ((ref[k,1]-ref[k-1,1])*t + (ref[k-1,1]*ref[k,0]-ref[k,1]*ref[k-1,0]))
                     /(ref[k,0] - ref[k-1,0]) )
        