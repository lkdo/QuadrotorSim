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
   
The model parameters are based on the following work:
  System Identification of the Crazyflie 2.0 Nano Quadrocopter
 by Julian Foerster
  
"""

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

import numpy as np
import math

#####################################################
# Julian Foerster's model 
#####################################################

class QuadFTau_CF:
    """ Model of the mini quadrotor the Crazyflie """
    
    def __init__(self, std_ratio, plus = True):
        """ std_ration should be in [0,1] """
        
        # if plus is false we have cross 
        self.plus = plus 
        
        self.radius = 0.045 # from center of mass to rotor, meters 
        self.mass = 0.028 # kg
        #self.Kaero = np.array([  [-10.2506, -0.3177, -0.4332],
        #                         [-0.3177, -10.2506, -0.4332],
        #                         [-7.7050, -7.7050, -7.5530] 
        #                     ])*10**-7
        self.Kaero = np.array([  [-9.1785, 0, 0],
                                            [0, -9.1785, 0],
                                            [0, 0, -10.311]  ])*10**-7

        self.input2thrust_coeff = [+ 2.130295*10**-11, +1.032633*10**-6, 
                              + 5.484560*10**-4]
        self.thrust2torque_coeff = [+0.005964552, +1.563383*10**-5]
        self.input2omegar_coeff = [+0.04076521, +380.8359]
        
        # [kg· m2]
        # Matrix of inertia, should be the same for + and x config
        if self.plus is False :
            # cross configuration 
            self.I = np.array([ [16.571710, -0.830806, -0.718277 ],
                            [ -0.830806, 16.655602, -1.800197],
                            [ -0.718277, -1.800197, 29.261652]])*10**-6 
        else:
            # plus configuration  ( Benoit Landry)
            self.I = np.array([ [2.3951, 0, 0 ],
                                        [ 0, 2.3951, 0],
                                        [ 0, 0, 3.23] ] )*10**-5 
            
        # Parameter variation, if requested
        if std_ratio > 0:
            self.radius += std_ratio*self.radius*np.random.randn(1)
            self.mass += std_ratio*self.mass*np.random.randn(1)
            self.Kaero += std_ratio*self.Kaero*np.random.randn(3,3)
            self.input2thrust_coeff += (std_ratio*self.input2thrust_coeff
                                       *np.random.randn(3))
            self.thrust2torque_coeff += (std_ratio*self.input2thrust_coeff  
                                        *np.random.randn(3))
            self.input2omegar_coeff += (std_ratio*self.input2thrust_coeff
                                       *np.random.randn(3))
            self.std_ratio = std_ratio 
           
        # After parameter variation, created the poly objects
        self.thrust2torque_poly = np.poly1d(self.thrust2torque_coeff)
        self.input2thrust_poly = np.poly1d(self.input2thrust_coeff)
        self.input2omegar_poly = np.poly1d(self.input2omegar_coeff)
        
        # assuming input2omegar polynomial remains degree 1
        self.omegar2input_coeff = [  1/self.input2omegar_coeff[0], 
                      -self.input2omegar_coeff[1]/self.input2omegar_coeff[0] ]
        self.omegar2input_poly = np.poly1d(self.omegar2input_coeff)
    
        # calculate cT & cQ 
        # Ns = 2000
        # thrust = np.zeros(65535-Ns)
        # omegar2 = np.zeros(65535-Ns)
        # torque = np.zeros(65535-Ns)
        # for i in range(65535-Ns):
        #     cmd = Ns + i
        #     thrust[i] = self.input2thrust_i(cmd)
        #     omegar2[i] = self.input2omegar_i(cmd)**2
        #     torque[i] = self.thrust2torque_i(thrust[i])
        # self.cT = np.mean(thrust/omegar2)
        # self.cT_std = np.std(thrust/omegar2)
        # self.cQ = np.mean(torque/omegar2)
        # self.cQ_std = np.std(torque/omegar2)
        # print("cT={}, std={}  \n cQ={}, std={}".format (self.cT,self.cT_std, 
        #                                                                           self.cQ, self.cQ_std))
        
        self.cT = 1.903*10**(-8)
        self.cQ = 1.246*10**(-10)
                  
    def input2thrust_i(self, cmd_i):
        """ Input is 0-65535, output (per rotor thrust) is in Newtons """
        if cmd_i < 1000:
            return 0
        return self.input2thrust_poly(cmd_i)
    
    def thrust2torque_i(self, ft_i):
        """ the torque [Nm] generated by a rotor yielding a given thrust [N] """
        if ft_i == 0:
           return 0
        return self.thrust2torque_poly(ft_i)
    
    def input2omegar_i(self, cmd_i):    
        """ Input is 0-65535, rotor angular velocity is in rad/s """
        
        if cmd_i <= 1000:
            return 0
        else:
            return self.input2omegar_poly(cmd_i)
    
    # the inverse of the above, vector form
    def omegar2input(self, omegar):
        """ Given vector of rotor angular velocities in rad/s, returns command """ 
        cmd = np.zeros(4)
        cmd[0] = self.omegar2input_poly(omegar[0])
        cmd[1] = self.omegar2input_poly(omegar[1])
        cmd[2] = self.omegar2input_poly(omegar[2])
        cmd[3] = self.omegar2input_poly(omegar[3])
        cmd[cmd < 0] = 0
        cmd[cmd > 65535] = 65535
        return cmd 
    
    def f_aero(self, omegar, vb):
      return np.dot(self.Kaero,vb)*np.sum(omegar)
        
    # Helper functions 
    ##################################################
    
    def input2thrust(self,cmd):
        # thrust on each rotor
        ft1 = self.input2thrust_i(cmd[0])
        ft2 = self.input2thrust_i(cmd[1])
        ft3 = self.input2thrust_i(cmd[2])
        ft4 = self.input2thrust_i(cmd[3])
        
        return (ft1 + ft2 + ft3 + ft4)
    
    def input2omegar(self,cmd):
        return np.array([ self.input2omegar_i(cmd[0]), 
                          self.input2omegar_i(cmd[1]),
                          self.input2omegar_i(cmd[2]), 
                          self.input2omegar_i(cmd[3]) ])
    
                
    # "Main" function
    ###################################################
        
    def input2ftau(self, cmd, vb):
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
            
            CW  (2)     (1) CCW
		          *    *
		           \  /
				    \/
				    @@
                    /\
                   /  \
		          *    *
		   CCW  (3)    (4) CW
		
		"""
            
        # thrust on each rotor
        ft1 = self.input2thrust_i(cmd[0])
        ft2 = self.input2thrust_i(cmd[1])
        ft3 = self.input2thrust_i(cmd[2])
        ft4 = self.input2thrust_i(cmd[3])
            
        # ang velocity on each rotor
        omegar = self.input2omegar(cmd)
          
        # aerodynamic force
        fba = self.f_aero(omegar,vb)
         
        # total force
        fb = ( np.array([0,0,ft1+ft2+ft3+ft4]) # thrust  
               + fba # aerodynamic forces s
              ) 
            
        taur1 = self.thrust2torque_i(ft1)
        taur2 = self.thrust2torque_i(ft2)
        taur3 = self.thrust2torque_i(ft3)
        taur4 = self.thrust2torque_i(ft4)
            
        if self.plus is True: 
            # total torques   		
            taub_x = (ft2 - ft4)*self.radius   # rolling moment 
            taub_y = (ft3 - ft1)*self.radius   # pitching moment 
            taub_z = -taur1 - taur3 + taur2 + taur4 # yawing moment 
        else: # we have cross
            taub_x = (ft2 + ft3 - ft1 - ft4)*math.sqrt(2)/2*self.radius   # rolling moment 
            taub_y = (ft3 + ft4 - ft2 - ft1)*math.sqrt(2)/2*self.radius   # pitching moment 
            taub_z = -taur1 - taur3 + taur2 + taur4 # yawing moment 
		
        return (fb, np.array([taub_x,taub_y,taub_z]))
        
    
####################################################
# A slightly simplified model, good for inversion   
####################################################

class QuadFTau_CF_b:
    
    def __init__(self, cT, cQ, radius, input2omegar_coeff, plus = True):
        """
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
            
            CW  (2)     (1) CCW
		          *    *
		           \  /
				    \/
				    @@
                    /\
                   /  \
		          *    *
		   CCW  (3)    (4) CW
           
        """
        
        self.cT = cT
        self.cQ = cQ
    
        # Gamma * omega_motors^2 = [fb_z, taub_x, taub_y, taub_z]
        if plus is True: 
            self.Gamma = np.array([[cT, cT, cT, cT],
                                   [0, radius*cT, 0, -radius*cT],
                                   [-radius*cT, 0, radius*cT, 0],
                                   [-cQ, cQ, -cQ, cQ]])
        else:
            self.Gamma = np.array([[cT, cT, cT, cT],
                                   [-radius*math.sqrt(2)/2*cT, radius*math.sqrt(2)/2*cT, 
                                            radius*math.sqrt(2)/2*cT, -radius*math.sqrt(2)/2*cT],
                                   [-radius*math.sqrt(2)/2*cT, -radius*math.sqrt(2)/2*cT, 
                                           radius*math.sqrt(2)/2*cT, radius*math.sqrt(2)/2*cT],
                                   [-cQ, cQ, -cQ, cQ]])
		
        self.invGamma = np.linalg.inv(self.Gamma)
       
        self.input2omegar_coeff = input2omegar_coeff  
        self.input2omegar_poly = np.poly1d(input2omegar_coeff)
       
        # assuming input2omegar polynomial remains degree 1
        self.omegar2input_coeff = [  1/self.input2omegar_coeff[0], 
                      -self.input2omegar_coeff[1]/self.input2omegar_coeff[0] ]
        self.omegar2input_poly = np.poly1d(self.omegar2input_coeff)
        
    def omegar2ftau(self, omega_rotors):
        """ Given omega_rotors in R+^4, returns forces and torques """
        for i in range(omega_rotors.size):
            assert(omega_rotors[i]>=0), "Negative omega rotor not allowed"
        # ftau = [ fb_z, taub_x, taub_y, taub_z]
        ftau = np.dot(self.Gamma, omega_rotors*omega_rotors)
        return( np.array([0, 0, ftau[0]]), ftau[1:3+1] ) 
            
    def ftau2omegar(self, fb, taub):
        """ Given a desired ftau, returns required omega rotors  """
        omega_r_square = np.dot(self.invGamma,np.block([fb[2],taub])) 
        assert ((omega_r_square < 0).sum() == 0), \
                    "Negative omegar_square in ftau2omegar()" 
        return np.sqrt(omega_r_square)
    
    def input2omegar_i(self,cmd_i):    
        """ Input is 0-65535, rotor angular velocity is in rad/s """
        if cmd_i <= 1000:
            return 0
        else:
            return self.input2omegar_poly(cmd_i)
    
    def input2omegar(self,cmd):
        return np.array([ self.input2omegar_i(cmd[0]), 
                          self.input2omegar_i(cmd[1]),
                          self.input2omegar_i(cmd[2]), 
                          self.input2omegar_i(cmd[3]) ])
    
    # the inverse of the above, vector form
    def omegar2input(self,omegar):
        """ Given vector of rotor angular velocities in rad/s, returns command """ 
        cmd = np.zeros(4)
        cmd[0] = self.omegar2input_poly(omegar[0])
        cmd[1] = self.omegar2input_poly(omegar[1])
        cmd[2] = self.omegar2input_poly(omegar[2])
        cmd[3] = self.omegar2input_poly(omegar[3])
        cmd[cmd < 0] = 0
        cmd[cmd > 65535] = 65535
        return cmd 
        
    # Final function (no aerodynamic component, drag is neglected)
    #####################################################
    
    def input2ftau(self,cmd):
        """ cmd is a 4 vector, each with values from 0 to 65535  """
            
        omegar = self.input2omegar(cmd)
        fb, taub = self.omegar2ftau(omegar)
        
        return fb, taub