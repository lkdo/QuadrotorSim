import numpy as np
  
g_CONST = 9.80665 # m/s^2
  
def SkS(X):
  return np.array([ [0,-X[3-1],X[2-1]], [X[3-1],0,-X[1-1]], [-X[2-1],X[1-1],0] ])
		