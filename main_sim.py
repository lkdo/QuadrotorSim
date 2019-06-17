# -*- coding: utf-8 -*-
"""
Created on Mon Jun 17 20:47:53 2019

@author: Luminita-Cristiana Totu
"""

import numpy as np
from RigidBody import RigidBody

# Initialization values for the states of the quadrotor
pos = np.array([0,0,0])
Rb2e = np.identity(3)
Vb =  np.array([0,0,0])
Ob = np.array([0,0,0])
mass = 1
I = np.identity(3)

# Instantiate a rigid body object, a quadrotor 
quad = RigidBody(pos,Rb2e,Vb,Ob,mass,I)

print(quad.pos)

for i in range(100):
  quad.run_quadrotor(0.01)
  print(quad.pos)

