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

""" Module implements the calculation of torques and forces
   
This is based on the following work:
  System Identification of the Crazyflie 2.0 Nano Quadrocopter
  by Julian Foerster
"""

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

import numpy as np

import qutils as ut
import cts


radius = 0.045 # from center of mass to rotor, meters 
mass = 0.028 # kg

def input2thrust_i(cmd):
    """ Input is 0-65535, output (per rotor thrust) is in Newtons """
    
    if cmd < 1000:
        return 0
    
    return (2.130295*10**-11*cmd**2 + 1.032633*10**-6*cmd + 5.484560*10**-4)

def input2thrust(cmd):
    """ Input is 4 dimensional """
    # thrust on each rotor
    ft1 = input2thrust_i(cmd[0])
    ft2 = input2thrust_i(cmd[1])
    ft3 = input2thrust_i(cmd[2])
    ft4 = input2thrust_i(cmd[3])
    
    return (ft1 + ft2 + ft3 + ft4)

# H(s) = c1/(d1+d2*s)
#c1 = 7.2345374*10-8
#d1 = 1
#d2 = -0.9695404

# dx(t)/dt = a*x(t) + b*u(t)
#input2thrust_fos = ut.FirstOrderSystem(-d1/d2, c1/d2, 0)

def input2omegar_i(cmd):    
    """ Input is 0-65535, rotor angular velocity is in rad/s """
    
    if cmd <= 1000:
        return 0
    else:
        return (0.04076521*cmd + 380.8359)

def omegar2input(omegar):

    cmd = (omegar - 380.8359)/0.04076521
    cmd[cmd < 0] = 0
    cmd[cmd > 65535] = 65535
    
    return cmd 
    
def input2omegar(cmd):
    return np.array([ input2omegar_i(cmd[0]), input2omegar_i(cmd[1]),
                      input2omegar_i(cmd[2]), input2omegar_i(cmd[3])  ])
    
def thrust2torque_i(fti):
    """ the torque [Nm] generated by a rotor yielding a given thrust [N] """
    
    if fti == 0:
       return 0
   
    return (0.005964552*fti + 1.563383*10**-5)

Kaero = np.array([  [-10.2506, -0.3177, -0.4332],
                    [-0.3177, -10.2506, -0.4332],
                    [-7.7050, -7.7050, -7.5530] 
                 ])*10**-7

def input2ftau(cmd, rotmb2e, vb):
        """ cmd is a 4 vector, each with values from 0 to 65535  
        
                 ^ x-axis
                (1) CCW
                 *
                 |
        y-axis   |      CW
        <(2)*----@----*(4) 
        CW       |
                 |
                 *
                (3) CCW
        
        """
        
        # thrust on each rotor
        ft1 = input2thrust_i(cmd[0])
        ft2 = input2thrust_i(cmd[1])
        ft3 = input2thrust_i(cmd[2])
        ft4 = input2thrust_i(cmd[3])
        
        omegar1 = input2omegar_i(cmd[0])
        omegar2 = input2omegar_i(cmd[1])
        omegar3 = input2omegar_i(cmd[2])
        omegar4 = input2omegar_i(cmd[3])
        
        # aerodynamic force
        fba = np.dot(Kaero,vb)*(omegar1+omegar2+omegar3+omegar4)
        
        # total force
        fb = ( np.array([0,0,ft1+ft2+ft3+ft4]) # thrust  
               + np.dot(np.transpose(rotmb2e),
                        np.array([0,0,-mass*cts.g_CONST])) 
               + fba ) # aerodynamic forces 
        
        taur1 = thrust2torque_i(ft1)
        taur2 = thrust2torque_i(ft2)
        taur3 = thrust2torque_i(ft3)
        taur4 = thrust2torque_i(ft4)
        
        taub_x = (ft2 - ft4)*radius   # rolling moment 
        taub_y = (ft3 - ft1)*radius   # pitching moment 
        taub_z = taur1 + taur3 - taur2 - taur4 # yawing moment 
        
        return (fb, np.array([taub_x,taub_y,taub_z]))
    
 ###############################################################

# Adding the old model, good for inversion   
cT = 1.88*(10**-8) 
cQ = cT*0.005964552

# Gamma * omega_motors^2 = [fb_z, taub_x, taub_y, taub_z]
Gamma = np.array([[cT, cT, cT, cT],
                  [0, radius*cT, 0, -radius*cT],
                  [-radius*cT, 0, radius*cT, 0],
                  [cQ, -cQ, cQ, -cQ]])
        
invGamma = np.linalg.inv(Gamma)

def omegar2ftau(omega_rotors):
    """ Given omega_rotors in R+^4, returns forces and torques """
        
    for i in range(omega_rotors.size):
        assert(omega_rotors[i]>=0), "Negative omega rotor not allowed"
                
    # ftau = [ fb_z, taub_x, taub_y, taub_z]
    ftau = np.dot(Gamma, omega_rotors*omega_rotors)
        
    return( np.array([0, 0, ftau[0]]), ftau[1:3+1] ) 
        
def ftau2omegar(fb, taub, rotmb2e) :
    """ Given a desired ftau, returns required omega rotors  """
        
    #assert(fb[0] == 0), "Non-zero fx in ftau2omegar()"
    #assert(fb[1] == 0), "Non-zero fy in ftau2omegar()"    
    
    fb = fb + np.dot(np.transpose(rotmb2e),np.array([0,0,+mass*cts.g_CONST]))
            
    omega_r_square = np.dot(invGamma,np.block([fb[2],taub])) 
 
    assert ((omega_r_square < 0).sum() == 0), \
                "Negative omegar_square in ftau2omegar()" 
        
    return np.sqrt(omega_r_square)

def input2ftau_b(cmd, rotmb2e):
    
    omegar = input2omegar(cmd)
    fb, taub = omegar2ftau(omegar)
    # add gravity
    fb = fb + np.dot(np.transpose(rotmb2e),
                        np.array([0,0,-mass*cts.g_CONST]))
    
    return fb, taub