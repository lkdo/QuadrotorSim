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

name1 = "PosControl"

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
pos_controller = ctrlatt.PosController_01()

omegab_ref = np.zeros(3)
tau_ref = np.zeros(3)
thrust_ref = qrb.mass*envir.g
rpy_ref = np.zeros(3)
pos_ref = np.zeros(3)
ref = np.array([0.0,0.0,3.0,0.0]) #  x,y,z,yaw

# Predefined controller references - for testing 
##########################################################
arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("ref_mode", help=""" Choose between an angle reference
                        template. Values are: step, ramp, sin, manual  """ )
args = arg_parser.parse_args()

if args.ref_mode == "step":
      
      name2 = "step"
     
      step_1 = StepAndRampMetaSignal(3,13,20)
      step_2 = StepAndRampMetaSignal(63,73,-20)
      pos_x_ref = utils.build_signal_step(0,80,0,step_1, step_2)

      step_1 = StepAndRampMetaSignal(23,33,20)
      step_2 = StepAndRampMetaSignal(63,73,-20)
      pos_y_ref = utils.build_signal_step(0,80,0,step_1, step_2)

      step_1 = StepAndRampMetaSignal(43,53,20)
      step_2 = StepAndRampMetaSignal(63,73,-20)
      pos_z_ref = utils.build_signal_step(0,80,3,step_1, step_2)

      step_1 = StepAndRampMetaSignal(3,13,120*math.pi/180)
      step_2 = StepAndRampMetaSignal(23,33,120*math.pi/180)
      step_3 = StepAndRampMetaSignal(43,53,120*math.pi/180)
      step_4 = StepAndRampMetaSignal(63,73,-120*math.pi/180)
      yaw_ref = utils.build_signal_step(0,80,0,step_1, step_2,step_3,step_4)

elif args.ref_mode == "ramp":
    
      name2 = "ramp"
      
      ramp_1 = StepAndRampMetaSignal(3,13,20)
      ramp_2 = StepAndRampMetaSignal(63,73,-20)
      pos_x_ref = utils.build_signal_ramp(0,80,0,ramp_1, ramp_2)

      ramp_1 = StepAndRampMetaSignal(23,33,20)
      ramp_2 = StepAndRampMetaSignal(63,73,-20)
      pos_y_ref = utils.build_signal_ramp(0,80,0,ramp_1, ramp_2)

      ramp_1 = StepAndRampMetaSignal(43,53,20)
      ramp_2 = StepAndRampMetaSignal(63,73,-20)
      pos_z_ref = utils.build_signal_ramp(0,80,3,ramp_1, ramp_2)

      ramp_1 = StepAndRampMetaSignal(3,13,120*math.pi/180)
      ramp_2 = StepAndRampMetaSignal(23,33,120*math.pi/180)
      ramp_3 = StepAndRampMetaSignal(43,53,120*math.pi/180)
      ramp_4 = StepAndRampMetaSignal(63,73,-120*math.pi/180)
      yaw_ref = utils.build_signal_ramp(0,80,0,ramp_1, ramp_2,ramp_3,ramp_4)

elif args.ref_mode == "sin":
    
    name2 = "sin"
    
    f = 1/10
    
    sin_1 = SinMetaSignal(3,10,1,1/f,20)
    sin_2 = SinMetaSignal(63,10,1,1/f,20)
    pos_x_ref = utils.build_signal_sin(0,27,0,sin_1, sin_2)

    sin_1 = SinMetaSignal(23,10,1,1/f,20)
    sin_2 = SinMetaSignal(63,10,1,1/f,20)
    pos_y_ref = utils.build_signal_sin(0,80,0,sin_1, sin_2)

    sin_1 = SinMetaSignal(43,10,1,1/f,20)
    sin_2 = SinMetaSignal(63,10,1,1/f,20)
    pos_z_ref = utils.build_signal_sin(0,80,3,sin_1, sin_2)
    
    sin_1 = SinMetaSignal(3,60*math.pi/180,1,1/f,20)
    sin_2 = SinMetaSignal(23,60*math.pi/180,1,1/f,20)
    sin_3 = SinMetaSignal(43,60*math.pi/180,1,1/f,20)
    sin_4 = SinMetaSignal(63,60*math.pi/180,1,1/f,20)
    yaw_ref = utils.build_signal_sin(0,80,0,sin_1, sin_2, sin_3, sin_4)
    
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
panda3D_app = pandaapp.Panda3DApp(plus,qrb,ref,3)
readkeys = pandaapp.ReadKeys(ref, 3, panda3D_app)

# The main simulation loop     
#########################################################
while readkeys.exitpressed is False :

    #------------------------------------begin controller --------------------------------------------
    if abs(t/pos_controller.dt_ctrl_pos - round(t/pos_controller.dt_ctrl_pos)) < 0.000001 :
        
        #reference
        if ( args.ref_mode == "manual" ):
            pos_ref = readkeys.ref
        else:
            pos_ref[0] = utils.give_signal(pos_x_ref, t)
            pos_ref[1] = utils.give_signal(pos_y_ref, t)
            pos_ref[2] = utils.give_signal(pos_z_ref, t)
            if ( t > max(pos_x_ref[-1,0], pos_y_ref[-1,0], pos_z_ref[-1,0]) ):
                readkeys.exitpressed = True 
                
        # perfect measurement 
        pos_meas = qrb.pos
        ve_meas = qrb.ve
        yaw_meas = qrb.rpy[2]
        
        rp, thrust_ref = pos_controller.run(pos_ref, pos_meas, ve_meas, 
                                                                                                      yaw_meas, qrb.mass)
        
        rpy_ref[0] = rp[0]
        rpy_ref[1] = rp[1]
        
    if abs(t/att_controller.dt_ctrl_angle - round(t/att_controller.dt_ctrl_angle)) < 0.000001 :
         
        # perfect measurement
        meas_rpy = qrb.rpy
        
        if ( args.ref_mode == "manual" ):
             rpy_ref[2] = readkeys.ref[3]
        else:
             rpy_ref[2] = utils.give_signal(yaw_ref, t)
             
        # controller call 
        omegab_ref = att_controller.run_angle(rpy_ref, meas_rpy)
        
    if abs(t/att_controller.dt_ctrl_rate - round(t/att_controller.dt_ctrl_rate)) < 0.000001 :
        
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
        logger.log_attstab(t,np.array([rpy_ref[0],rpy_ref[1],rpy_ref[2]]),
                                      np.array([omegab_ref[0],omegab_ref[1],omegab_ref[2]]),
                                      np.array([tau_ref[0],tau_ref[1],tau_ref[2]]) )
        logger.log_posctrl(t,np.array([ pos_ref[0], pos_ref[1], pos_ref[2] ]))
        logger.log_rigidbody(t, qrb)
        fe = qrb.rotmb2e@fb + qrb.mass*np.array([0,0,-envir.g])
        taue = qrb.rotmb2e@taub
        logger.log_ftau(t,fe,taue,fb,taub)
        logger.log_cmd(t, cmd)

        
# End of program, wrap it up with logger and plotter  
#########################################################
logger.log2file_rigidbody()
logger.log2file_cmd()
logger.log2file_ftau()
logger.log2file_attstab()
logger.log2file_posctrl()
plotter.plot_rigidbody(logger)
plotter.plot_cmd(logger)
plotter.plot_attstab(logger)
plotter.plot_posctrl(logger)
