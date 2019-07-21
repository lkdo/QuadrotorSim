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

""" Attitude controller using Euler angles """

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

import math 
import numpy as np 

import pid 
import utils
import envir

class AttController_01:
    
    def __init__(self):
    
        self.dt_ctrl_rate = 0.002  # 500 Hz

        self.pid_rollrate = pid.PID(100, 0, 0, 8*360*math.pi/180,-8*360*math.pi/180, 0.01)
        self.pid_pitchrate = pid.PID(100, 0, 0, 8*360*math.pi/180,-8*360*math.pi/180, 0.01)
        self.pid_yawrate = pid.PID(100, 0, 0, 8*360*math.pi/180, -8*360*math.pi/180, 0.01)
        
        self.dt_ctrl_angle = 0.004  # 250 Hz
        
        self.pid_pitch = pid.PID(10, 0, 0, 2.5*360*math.pi/180, -2.5*360*math.pi/180, 0.01)
        self.pid_roll = pid.PID(10, 0, 0, 2.5*360*math.pi/180, -2.5*360*math.pi/180, 0.01)
        self.pid_yaw = pid.PID(5, 0, 0, 2.5*360*math.pi/180, -2.5*360**math.pi/180, 0.01)

    def run_rate(self, ref_omegab, meas_omegab, J ):
        
        alpha_ref = np. zeros(3)
        
        alpha_ref[0] = self.pid_rollrate.run(ref_omegab[0]-meas_omegab[0], self.dt_ctrl_rate)
        alpha_ref[1] = self.pid_pitchrate.run(ref_omegab[1]-meas_omegab[1],self.dt_ctrl_rate)
        alpha_ref[2] = self.pid_yawrate.run(ref_omegab[2]-meas_omegab[2],self.dt_ctrl_rate)
        
        tau_ref = J@alpha_ref + utils.skew(meas_omegab)@J@meas_omegab

        return tau_ref

    def run_angle(self, ref_rpy, meas_rpy ):
        
        omega_ref = np. zeros(3)
        
        omega_ref[0] = self.pid_roll.run(ref_rpy[0]-meas_rpy[0],self.dt_ctrl_angle)
        omega_ref[1] = self.pid_pitch.run(ref_rpy[1]-meas_rpy[1],self.dt_ctrl_angle)
        
        err_yaw = ref_rpy[2]-meas_rpy[2]
        if (err_yaw > math.pi):
            err_yaw = 2*math.pi - err_yaw
        elif (err_yaw < -math.pi):
            err_yaw = 2*math.pi + err_yaw
        
        omega_ref[2] = self.pid_yaw.run(err_yaw,self.dt_ctrl_angle)
        
        return omega_ref
    
    
class PosController_01:    
    
    def __init__(self):
        
        self.dt_ctrl_pos = 0.02  # 50 Hz
        self.K1 = np.array([[-2.5,0],[0,-2.5]])
        self.K2 = np.array([[-2.5,0],[0,-2.5]])
        self.K3 = -3
        self.K4 = -3
        self.max_thrust = 0.8*0.638

    def run(self, ref_pos, meas_pos, meas_ve, meas_yaw, mass):
       
        rp_ref = np.zeros(2)
        
        R = np.array([[math.sin(meas_yaw), -math.cos(meas_yaw)],
                              [math.cos(meas_yaw), math.sin(meas_yaw)]])
        rp_ref  = 1/envir.g*R@( self.K1@meas_ve[0:2] 
                                                         + self.K2@(meas_pos[0:2]-ref_pos[0:2]) )
        # and  saturate them 
        V = 40 * math.pi /180 
        v_max = np.max(abs(rp_ref))
        if (v_max > V):
            rp_ref = (V/v_max)*rp_ref[0:2]
        
        
        thrust_ref = mass*envir.g + mass*( self.K3*meas_ve[2]
                                                               + self.K4*(meas_pos[2]-ref_pos[2]) ) 
        
        # And saturate 
        if ( thrust_ref > self.max_thrust ):
            thrust_ref = self.max_thrust
        elif (thrust_ref < 0.9*mass*envir.g ):
            thrust_ref =  0.9*mass*envir.g 
        
        return rp_ref, thrust_ref
    
class PosController_02:

    def __init__(self):
        
          self.dt_ctrl_pos_v = 0.02  # 50 Hz
          self.pid_vx = pid.PID(4, 0, 0, 20, -20, 0.01)
          self.pid_vy = pid.PID(4, 0, 0, 20, -20, 0.01)
          self.pid_vz = pid.PID(5, 0, 0, 20, -20, 0.01)

          self.dt_ctrl_pos_p = 0.02  # 50 Hz
          self.pid_x = pid.PID(1, 0, 0, 20, -20, 0.01)
          self.pid_y = pid.PID(1, 0, 0, 20, -20, 0.01)
          self.pid_z = pid.PID(4, 0, 0, 10, -10, 0.01)
          
          self.max_thrust =  0.8*0.638
    
    def run_vel(self, ref_ve, meas_ve, meas_yaw, mass):
        
        rp_ref = np.zeros(2)
        
        T1 = self.pid_vx.run(ref_ve[0]-meas_ve[0],self.dt_ctrl_pos_v)
        T2 = self.pid_vy.run(ref_ve[1]-meas_ve[1],self.dt_ctrl_pos_v)
        
        R = np.array([[math.sin(meas_yaw), -math.cos(meas_yaw)],
                              [math.cos(meas_yaw), math.sin(meas_yaw)]])
        
        rp_ref = 1/envir.g *R@np.array([T1,T2])
        
        # and  saturate again 
        V = 40 * math.pi /180 
        v_max = np.max(abs(rp_ref))
        if (v_max > V):
            rp_ref = (V/v_max)*rp_ref
        
        T3 = self.pid_vz.run(ref_ve[2] - meas_ve[2], self.dt_ctrl_pos_v)
        thrust_ref = mass*envir.g + mass*T3 
 
       # And saturate again
        if ( thrust_ref > self.max_thrust ):
            thrust_ref = self.max_thrust
        elif (thrust_ref < 0.9*mass*envir.g ):
            thrust_ref =  0.9*mass*envir.g 
        
        return rp_ref, thrust_ref
    
    def run_pos(self, ref_pos, meas_pos):
        
        ref_ve = np.zeros(3)
        
        ref_ve[0] = self.pid_x.run(ref_pos[0] - meas_pos[0], self.dt_ctrl_pos_p)
        ref_ve[1] = self.pid_y.run(ref_pos[1] - meas_pos[1], self.dt_ctrl_pos_p)
        ref_ve[2] = self.pid_z.run(ref_pos[2] - meas_pos[2], self.dt_ctrl_pos_p)
        
        return ref_ve