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

""" Main loop of simulation for Rigid Body Model with Forces and Torques """

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

# Import general libraries
import numpy as np
import time
import datetime
import argparse
import math 
from dataclasses import dataclass

# Import local files
import pandaapp
import ftaucf 
import rigidbody
import logger 
import plotter
import envir  
import ctrlatt
import utils 

@dataclass
class StepAndRampMetaSignal:
    t_start: float
    t_end: float 
    value: float

@dataclass
class SinMetaSignal:
    t_start: float
    amplitude: float
    no_periods: float 
    period: float 
    no_points_per_period: float 

name1 = "AttControl"

# Initialization values for the quadrotor
##########################################################
plus = True
""" Quadrotor configuration, plus or cross """
pos = np.array([0,0,3])
""" position vetcor in meters """
q = np.array([1,0,0,0])
""" unit quaternion  representing attitude """
ve = np.array([0,0,0])
""" linear velocity vector in the earth-fixed frame """
omegab = np.array([0,0,0])
""" angular velocity vector in the body-fixed frame """
qftau = ftaucf.QuadFTau_CF(0,plus)
""" Model for the forces and torques of the crazyflie """
qftau_s = ftaucf.QuadFTau_CF_S(qftau.cT, qftau.cQ, qftau.radius, 
                                 qftau.input2omegar_coeff, plus)
""" Simplified model for the forces and torques """
qrb = rigidbody.rigidbody(pos, q, ve, omegab, qftau.mass, qftau.I)
""" Rigif body motion object  """

# Initialize controller  
##########################################################
att_controller = ctrlatt.AttController_01()

omegab_ref = np.zeros(3)
tau_ref = np.zeros(3)
thrust_ref = qrb.mass*envir.g
rpy_ref = np.zeros(3)
ref = np.block([thrust_ref,omegab_ref])

# Predefined controller references - for testing 
##########################################################
arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("ref_mode", help=""" Choose between an angle reference
                        template. Values are: step, ramp, sin, manual  """ )
args = arg_parser.parse_args()

if args.ref_mode == "step":
      
      name2 = "step"
     
      step_1 = StepAndRampMetaSignal(3,6,80*math.pi/180)
      step_2 = StepAndRampMetaSignal(21,24,40*math.pi/180)
      roll_ref = utils.build_signal_step(0,27,0,step_1, step_2)

      step_1 = StepAndRampMetaSignal(9,12,80*math.pi/180)
      step_2 = StepAndRampMetaSignal(21,24,40*math.pi/180)
      pitch_ref = utils.build_signal_step(0,27,0,step_1, step_2)

      step_1 = StepAndRampMetaSignal(15,18,170*math.pi/180)
      step_2 = StepAndRampMetaSignal(21,24,40*math.pi/180)
      yaw_ref = utils.build_signal_step(0,27,0,step_1, step_2)

elif args.ref_mode == "ramp":
    
      name2 = "ramp"
      
      step_1 = StepAndRampMetaSignal(3,6,80*math.pi/180)
      step_2 = StepAndRampMetaSignal(21,24,40*math.pi/180)
      roll_ref = utils.build_signal_ramp(0,27,0,step_1, step_2)

      step_1 = StepAndRampMetaSignal(9,12,80*math.pi/180)
      step_2 = StepAndRampMetaSignal(21,24,40*math.pi/180)
      pitch_ref = utils.build_signal_ramp(0,27,0,step_1, step_2)

      step_1 = StepAndRampMetaSignal(15,18,170*math.pi/180)
      step_2 = StepAndRampMetaSignal(21,24,40*math.pi/180)
      yaw_ref = utils.build_signal_ramp(0,27,0,step_1, step_2)

elif args.ref_mode == "sin":
    
    name2 = "sin"
    
    f = 1/3
    
    sin_1 = SinMetaSignal(3,80*math.pi/180,1,1/f,20)
    sin_2 = SinMetaSignal(21,40*math.pi/180,1,1/f,20)
    roll_ref = utils.build_signal_sin(0,27,0,sin_1, sin_2)

    sin_1 = SinMetaSignal(9,80*math.pi/180,1,1/f,20)
    sin_2 = SinMetaSignal(21,40*math.pi/180,1,1/f,20)
    pitch_ref = utils.build_signal_sin(0,27,0,sin_1, sin_2)

    sin_1 = SinMetaSignal(15,180*math.pi/180,1,1/f,20)
    sin_2 = SinMetaSignal(21,40*math.pi/180,1,1/f,20)
    yaw_ref = utils.build_signal_sin(0,27,0,sin_1, sin_2)
    
else:

      name2 = "manual"
      
# Simulation parameters
##########################################################
dt_sim = 0.0005  
""" integration step """
dt_log = 0.1
""" logging step """
dt_vis = 1/60   
""" visualization frame step """
t = 0
""" time variable """

# Initialize the logger & plotter 
##########################################################
ts = time.time()
name = name1 + "_" + name2 +datetime.datetime.fromtimestamp(ts).strftime("_%Y%m%d%H%M%S")
""" Name of run to save to log files and plots """
fullname = "logs/" + name
logger = logger.Logger(fullname, name)
plotter = plotter.Plotter()

# Initialize the visualization
#########################################################
panda3D_app = pandaapp.Panda3DApp(plus,qrb,ref)
readkeys = pandaapp.ReadKeys(ref, 2, panda3D_app)

# The main simulation loop     
#########################################################
while readkeys.exitpressed is False :

    #------------------------------------begin controller --------------------------------------------
    if abs(t/att_controller.dt_ctrl_angle - round(t/att_controller.dt_ctrl_angle)) < 0.000001 :
         
        #reference
        if ( args.ref_mode == "manual" ):
            rpy_ref = readkeys.ref[1:4]
        else:
            rpy_ref[0] = utils.give_signal(roll_ref, t)
            rpy_ref[1] = utils.give_signal(pitch_ref, t)
            rpy_ref[2] = utils.give_signal(yaw_ref, t)
            if ( t > max(roll_ref[-1,0], pitch_ref[-1,0], yaw_ref[-1,0]) ):
                readkeys.exitpressed = True 
        
        # perfect measurement
        meas_rpy = qrb.rpy
        
        # controller call 
        omegab_ref = att_controller.run_angle(rpy_ref, meas_rpy)
        
    if abs(t/att_controller.dt_ctrl_rate - round(t/att_controller.dt_ctrl_rate)) < 0.000001 :
        
        # thrust comes alwyas from the keyboard 
        thrust_ref = readkeys.ref[0]        
        
        # Get Measurement (perfect measurement)
        omegab_meas = qrb.omegab
       
        # Controller call 
        tau_ref = att_controller.run_rate(omegab_ref,omegab_meas,qrb.I)
        
        # Control allocation; use qftau_s model 
        cmd = qftau_s.fztau2cmd(np.array([thrust_ref,tau_ref[0],tau_ref[1],tau_ref[2]]))
   
    #------------------------------------------ end controller --------------------------------------
   
    # Calculate body-based forces and torques
    fb, taub = qftau.input2ftau(cmd,qrb.vb)
    
    # Run the kinematic / time forward
    qrb.run_quadrotor(dt_sim, fb, taub)

    # Time has increased now
    t = t + dt_sim
     
    # Visualization frequency    
    if abs(t/dt_vis - round(t/dt_vis)) < 0.000001 :
        panda3D_app.taskMgr.step()
        panda3D_app.screenText_pos(qrb.pos,qrb.rpy)
        
    # Logging frequency    
    if abs(t/dt_log - round(t/dt_log)) < 0.000001 :
        logger.log_attstab(t,np.array([rpy_ref[0], rpy_ref[1], rpy_ref[2]]),
                                      np.array([omegab_ref[0],omegab_ref[1],omegab_ref[2]]),
                                      np.array([tau_ref[0],tau_ref[1],tau_ref[2]]) )
        logger.log_rigidbody(t, qrb)
        fe = qrb.rotmb2e@fb + qrb.mass*np.array([0,0,-9.80665])
        taue = qrb.rotmb2e@taub
        logger.log_ftau(t,fe,taue,fb,taub)
        logger.log_cmd(t, cmd)
        
        
# End of program, wrap it up with logger and plotter  
#########################################################
logger.log2file_rigidbody()
logger.log2file_cmd()
logger.log2file_ftau()
logger.log2file_attstab()
plotter.plot_rigidbody(logger)
plotter.plot_cmd(logger)
plotter.plot_attstab(logger)
