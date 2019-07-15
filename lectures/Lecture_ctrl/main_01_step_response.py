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
import math
import envir

# Import local files
import pandaapp
import ftaucf 
import rigidbody
import logger 
import plotter
import pid  

# Initialization values for the quadrotor
##########################################################
plus = False
""" Quadrotor configuration, plus or cross """
pos = np.array([0,0,3])
""" position vetcor in meters """
q = np.array([1,0,0,0])
""" unit quaternion  representing attitude """
ve = np.array([0,0,0])
""" linear velocity vector in the earth-fixed frame """
omegab = np.array([0,0,0])
""" angular velocity vector in the body-fixed frame """
cmd = np.array([37278,37278,37278,37278])
""" intital command """
qftau = ftaucf.QuadFTau_CF(0,plus)
""" Model for the forces and torques of the crazyflie """
qftau_s = ftaucf.QuadFTau_CF_S(qftau.cT, qftau.cQ, qftau.radius, 
                                 qftau.input2omegar_coeff, plus)
""" Simplified model for the forces and torques """
qrb = rigidbody.rigidbody(pos, q, ve, omegab, qftau.mass, qftau.I)
""" Rigif body motion object  """

# Initialize controller assets 
##########################################################
dt_ctrl_rate = 0.002 # 500 Hz

alpha_ref = np.zeros(3)
tau_ref = np.zeros(3)

thrust_ref = qrb.mass*envir.g
omega_ref = np.zeros(3)
ref = np.block([thrust_ref,omega_ref])

tau = 0.005
pid_rollrate = pid.PID(100, 0, 0, 8*360*math.pi/180, -8*360*math.pi/180, tau)
pid_pitchrate = pid.PID(100, 0, 0, 8*360*math.pi/180, -8*360*math.pi/180, tau)
pid_yawrate = pid.PID(100, 0, 0, 8*360*math.pi/180, -8*360*math.pi/180, tau)

# Simulation parameters
##########################################################
dt_sim = 0.0005 
""" integration step """
dt_log = 0.01
""" logging step """
dt_vis = 1/60   
""" visualization frame step """
T_sim = 25
""" Total time of the simulation """

# Predefined omega-reference to step 
##########################################################
omega_ref_step = np.zeros([int(T_sim/dt_ctrl_rate)+1,3])
omega_ref_step[int(3/dt_ctrl_rate)+1:int(6/dt_ctrl_rate),0] = 4*360*math.pi/180 # rad/sec
omega_ref_step[int(7/dt_ctrl_rate)+1:int(10/dt_ctrl_rate),1] = 4*360*math.pi/180 # rad/sec
omega_ref_step[int(11/dt_ctrl_rate)+1:int(14/dt_ctrl_rate),2] = 4*360*math.pi/180 # rad/sec
omega_ref_step[int(15/dt_ctrl_rate)+1:int(18/dt_ctrl_rate),:] = 2.5*360*math.pi/180 # rad/sec

k = 0

# Initialize the logger & plotter 
##########################################################
ts = time.time()
name = "StepResponse_Omega"+datetime.datetime.fromtimestamp(ts).strftime("_%Y%m%d%H%M%S")
""" Name of run to save to log files and plots """
fullname = "logs/" + name
logger = logger.Logger(fullname, name)
plotter = plotter.Plotter()

# Initialize the visualization
#########################################################
panda3D_app = pandaapp.Panda3DApp(plus,qrb,ref)
readkeys = pandaapp.ReadKeys(ref, 1, panda3D_app)

# The main simulation loop     
#########################################################
for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):

    #------------------------------------begin controller --------------------------------------------
    
    if abs(t/dt_ctrl_rate - round(t/dt_ctrl_rate)) < 0.000001 :
        
        # perfect measurement
        meas_omegab = qrb.omegab 
        
        omega_ref = omega_ref_step[k,:];  k += 1
        thrust_ref = readkeys.ref[0]
        
        alpha_ref[0] = pid_rollrate.run(omega_ref[0]-meas_omegab[0], dt_ctrl_rate)
        alpha_ref[1] = pid_pitchrate.run(omega_ref[1]-meas_omegab[1],dt_ctrl_rate)
        alpha_ref[2] = pid_yawrate.run(omega_ref[2]-meas_omegab[2],dt_ctrl_rate)
        
        tau_ref = qftau.I@alpha_ref + rigidbody.skew(meas_omegab)@qftau.I@meas_omegab
        
        # Calculate the command based on tau_ref and thrust_ref 
        cmd = qftau_s.fztau2cmd(np.array([thrust_ref,tau_ref[0],tau_ref[1],tau_ref[2]]))
   
    #------------------------------------------ end controller --------------------------------------
    
    # Calculate body-based forces and torques
    fb, taub = qftau.input2ftau(cmd,qrb.vb)
    
    # Run the kinematic / time forward
    qrb.run_quadrotor(dt_sim, fb, taub)

    # Visualization frequency    
    if abs(t/dt_vis - round(t/dt_vis)) < 0.000001 :
        panda3D_app.taskMgr.step()
        panda3D_app.screenText_pos(qrb.pos,qrb.rpy)
        
    # Logging frequency    
    if abs(t/dt_log - round(t/dt_log)) < 0.000001 :
        logger.log_attstab(t,np.zeros(3),
                                      np.array([omega_ref[0],omega_ref[1],omega_ref[2]]),
                                      np.array([alpha_ref[0],alpha_ref[1],alpha_ref[2]]),
                                      np.array([tau_ref[0],tau_ref[1],tau_ref[2]]) )
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
plotter.plot_rigidbody(logger)
plotter.plot_cmd(logger)
plotter.plot_attstab(logger)
