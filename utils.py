# -*- coding: utf-8 -*-
"""
Created on Tue Jun 18 20:02:20 2019

@author: Luminita-Cristiana Totu
"""
import numpy as np

# the Skew-symmetric matrix form of a vector
# helpful in writing the vector cross product
def SkS(X):
  return np.array([[0,-X[3-1],X[2-1]],[X[3-1],0,-X[1-1]],[-X[2-1],X[1-1],0]])
