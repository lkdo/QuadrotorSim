# -*- coding: utf-8 -*-
"""
Created on Mon Jun 17 20:47:53 2019

@author: Luminita-Cristiana Totu
"""

import numpy as np
import math
from constants import g_CONST
from utils import SkS

class RigidBody:
  """ Holds the minimum number of states and parameters 
  to describe a Rigid Body, and implements the dynamics  """
	
  def __init__(self, pos, Rb2e, Vb, 
                Ob, mass, inertiaMatrix):
    self.pos = pos
    self.Rb2e = Rb2e
    self.Vb = Vb
    self.Ob = Ob
    self.mass = mass;
    self.I = inertiaMatrix;
		
  def run_quadrotor(self,dt,Fb,taub):

    ## Dynamic/Differential equations for rigid body motion/flight
    
    # dpos/dt = Ve = Rb2e*Vb 
    d_pos = np.dot(self.Rb2e,self.Vb)  
    
    # dRb2e/dt = Rb2e*Sks(Ob)
    d_Rb2e = np.dot(self.Rb2e,SkS(self.Ob)) 
    
    # dVb/dt =  -Sks(Ob)*Vb + Re2b*ge + 1/m*Fb + noise
    d_Vb = ( -np.dot(SkS(self.Ob),self.Vb) +  
                 np.dot(self.Rb2e.transpose(),[0, 0, g_CONST]) + 
                 1/self.mass*Fb + 0.025*math.sqrt(1/dt)*np.random.randn(3) )
    
    # dOb/dt = I^(-1)*(-SkS(Ob)*I*Ob + taub) + noise
    d_Ob = ( np.dot(np.linalg.inv(self.I), np.dot(-SkS(self.Ob),
                  np.dot(self.I,self.Ob)) + taub ) + 
	              0.025*math.sqrt(1/dt)*np.random.randn(3) )
    
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
      	
	   