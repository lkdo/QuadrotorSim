import numpy as np
import math
from constants import g_CONST
from constants import SkS

class RigidBody:
  """ Holds the minimum number of states and parameters 
  to describe a Rigid Body, and implements the dynamics  """
	
  def __init__(self, position, Rb2e, linVelocityB, angVelocityB, mass, inertiaMatrix):
    self.pos = position
    self.Rb2e = Rb2e
    self.Vb = linVelocityB
    self.Ob = angVelocityB
    self.mass = mass;
    self.I = inertiaMatrix;
		
  def dynamicEqQuadrotor(self,t,X,dt):
    l_rotmB2E = X[4-1:12].reshape(3,3); # the rotation matrix 
    l_linVelB = X[13-1:15]; # lin vel body frame
    l_angVelB = X[16-1:18]; # ang vel body frame 
    
    # hard coded forces and torques for now
    Fb = np.array([0,0,-5])
    taub =  np.array([0,0,0])
	
    d_pos = np.dot(l_rotmB2E,l_linVelB) 
    d_rotmB2E = np.dot(l_rotmB2E,SkS(l_angVelB))
    d_linVelB = ( -np.dot(SkS(l_angVelB),l_linVelB) + np.dot(l_rotmB2E.transpose(),[0, 0, g_CONST]) + 
                 1/self.mass*Fb + 0.025*math.sqrt(1/dt)*np.random.randn(3) )
    d_angVelB = ( np.dot(np.linalg.inv(self.I), np.dot(-SkS(l_angVelB),np.dot(self.I,l_angVelB)) + taub ) + 
	             0.025*math.sqrt(1/dt)*np.random.randn(3) )
    
    dX = np.concatenate([ d_pos, d_rotmB2E.reshape(-1), d_linVelB, d_angVelB ])
    
    return dX
    
  def run_quadrotor(self,dt):
    X = np.concatenate([self.pos, self.Rb2e.reshape(-1), self.Vb, self.Ob])
    dX = self.dynamicEqQuadrotor(0,X,dt)
    X = X + dt*dX
    self.unpack_X(X)
         
  def unpack_X(self,X):
    self.pos = X[1-1:3]
    self.Rb2e = X[4-1:12].reshape(3,3)
    self.Vb = X[13-1:15]
    self.Ob = X[16-1:18]
	
	
      	
	   