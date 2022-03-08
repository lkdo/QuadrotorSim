# -*- coding: utf-8 -*-
# 
# Copyright 2021 Luminita-Cristiana Totu
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

""" Module implements the a mems sensor with white noise and slowly moving bias

"""

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

import numpy as np
from scipy.linalg import cholesky
from scipy.linalg import sqrtm
from math import sqrt
from rigidbody import quadrotor_dt_kinematic_euler
from scipy.integrate import odeint
from numpy.linalg import inv
import utils 

def create_sigma_points( x, P , L, l ):
    sr_P = cholesky((L+l)*P, lower=True)
    #sr_P = sqrtm((L+l)*P)
    S = [x]
    for i in range(L):
        S.append(x+sr_P[:,i])
        S.append(x-sr_P[:,i])
    #print(S)
    #print("-------------")
    #input("Press Enter to continue...")
    return S

##########################################################    

class ukf:
    
    # x = [pos,euler,ve]
    # can add gyro and acc biases in future for example

    def __init__(self,x0,P0,Q,R,meas_func,alpha,kappa,beta):
    
        self.x = x0
        """ Initial State """
        self.wrap_state_angles()

        self.P = P0
        """ Initial Covariance """

        self.Q = Q
        """ Propagation noise matrix """

        self.R = R
        """ Measurement noise matrix """

        self.alpha = alpha
        self.kappa = kappa
        self.beta = beta
        """ Sigma points parameters """

        self.n = x0.size
        """ State dimensionality """
        
        self.l = (self.alpha**2)*(self.n+self.kappa) - self.n
        """ ct used in the  sigma points """
        
        # scaled unscented transform ( try also central differnece maybe)
        self.W0m = self.l/(self.n+self.l)
        self.W0c = self.W0m+1-self.alpha**2+self.beta
        self.Wi = 0.5/(self.n+self.l)
        """ sigma points weights """

        self.meas_func = meas_func
##########################################################  

    def run_predict(self,dt,ab,omegab):

        S = create_sigma_points(self.x, self.P, self.n, self.l)
  
        # propagate the points 
        Sp = [ ]
        for i in range(2*self.n+1):
            
            # ODE Int integration to make the state tranzition
            #Y = odeint(quadrotor_dt_kinematic_euler,S[i],np.array([0, dt]),args=(ab,omegab,))
            #Sp.append(Y[1]) # Y[0]=x(t=t0)
            
            # Euler faster
            Sp.append(S[i]+quadrotor_dt_kinematic_euler(S[i],0,ab,omegab)*dt)

        # calculate the mean, covariance and cross-covariance of the set 
        Xm = self.W0m*Sp[0]
        for i in range(2*self.n):
            Xm = Xm + self.Wi*Sp[i+1]
        Cx = self.W0c*np.outer(Sp[0]-Xm,Sp[0]-Xm)+dt*self.Q
        for i in range(2*self.n):
            Cx = Cx + self.Wi*np.outer(Sp[i+1]-Xm,Sp[i+1]-Xm)

        self.x = Xm
        self.P = Cx

        self.wrap_state_angles()

    def run_meas(self,meas):

        S = create_sigma_points(self.x, self.P, self.n, self.l)
        
        Z = [ ]
        for i in range(2*self.n+1):
            Z.append(self.meas_func(S[i]))

        # calculate the mean and covariance of the set 
        Zm = self.W0m*Z[0]
        for i in range(2*self.n):
            Zm = Zm + self.Wi*Z[i+1]
        Cx = self.W0c*np.outer(Z[0]-Zm,Z[0]-Zm)+self.R
        Csz = self.W0c*np.outer(S[0]-self.x,Z[0]-Zm)
        for i in range(2*self.n):
            Cx = Cx + self.Wi*np.outer(Z[i+1]-Zm,Z[i+1]-Zm)
            Csz = Csz + self.Wi*np.outer(S[i+1]-self.x,Z[i+1]-Zm)

        # Kalman Gain
        K = Csz@inv(Cx)

        # Update mean and covarince
        inn = meas-Zm  # if any angles here, I would need to wrap them, but I have none 
        self.x = self.x + K@inn
        self.P = self.P - K@Cx@np.transpose(K)

        self.wrap_state_angles()

    def wrap_state_angles(self):
        
        self.x[3] = utils.clampRotation(self.x[3])
        self.x[4] = utils.clampRotation(self.x[4])
        self.x[5] = utils.clampRotation(self.x[5])
