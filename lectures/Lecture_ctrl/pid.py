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
# If not, see <https://www.gnu.org/licenses/>

""" Implements a PID controller in discrete time """

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"
	
class PID:
    """ implements a discrete time PID controller 
    
    as described in Small Unmanned Aircraft Theory and Practice
    by  Randal W. Beard and Timothy W. McLain 
    """
    
    def __init__(self, kp,ki,kd,limit_up, limit_down, tau):
        """ Initialize the PID with the kp, ki and kd constants, saturation limits and tau"""
        
        self.kp = kp
        """ Proportional gain """
        
        self.ki = ki 
        """ Integral gain """
        
        self.kd = kd 
        """ Derivative gain """
        
        self.limit_up = limit_up
        """ Upper saturation limit """
        
        self.limit_down = limit_down 
        """ Lower saturation limit """
        
        self.tau = tau 
        """ Time constant of the differentiator """

        self .integrator = 0
        """ Integrator state """
        
        self.differentiator = 0
        """ Differentiator state """
        
        self.error_d1 = 0 
        """ Previous error """
  
    def reset(self):

        self.integrator = 0
        self.differentiator = 0
        self.error_d1 = 0
       
    def run(self,  error, Ts):
    
        self.integrator = self.integrator + Ts/2*(error + self.error_d1)
        self.differentiator =(  (2*self.tau - Ts)/(2*self.tau + Ts)*self.differentiator  
                                          + 2/(2*self.tau +Ts)*(error - self.error_d1)  )
        self.error_d1 = error
        
        u_raw = self.kp*error + self.ki*self.integrator + self.kd*self.differentiator
        
        if (u_raw > self.limit_up ):
            u = self.limit_up
        elif (u_raw < self.limit_down ):
            u = self.limit_down
        else:
            u = u_raw 
            
        # Integrator anti-windup
        if (self.ki != 0):
            self.integrator = self.integrator +Ts/self.ki *(u - u_raw)
        
        return u 
      