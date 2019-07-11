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
import math
import time
import datetime

# Import local files
import pandaapp
import ftaucf 
import rigidbody
import logger 
import plotter  

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
cmd = 37278.0 * np.ones(4)
""" Initial command for the quadrotor, hoover command """
qftau = ftaucf.QuadFTau_CF(0,plus)
""" Model for the forces and torques of the crazyflie """
qrb = rigidbody.rigidbody(pos, q, ve, omegab, qftau.mass, qftau.I)
""" Rigif body motion object  """

# Simulation parameters
##########################################################
dt_sim = 0.001  
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
name = "Manual"+datetime.datetime.fromtimestamp(ts).strftime("_%Y%m%d%H%M%S")
""" Name of run to save to log files and plots """
fullname = "logs/" + name
logger = logger.Logger(fullname, name)
plotter = plotter.Plotter()

# Initialize the visualization
#########################################################
panda3D_app = pandaapp.Panda3DApp(plus,qrb,cmd)
readkeys = pandaapp.ReadKeys(cmd, plus, panda3D_app)

# The main simulation loop     
#########################################################
while readkeys.exitpressed is False :

    # Calculate body-based forces and torques
    fb, taub = qftau.input2ftau(readkeys.cmd,qrb.vb)
    
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
        logger.log_rigidbody(t, qrb)
        fe = qrb.rotmb2e@fb + qrb.mass*np.array([0,0,-9.80665])
        taue = qrb.rotmb2e@taub
        logger.log_ftau(t,fe,taue,fb,taub)
        logger.log_cmd(t, readkeys.cmd)
        
# End of program, wrap it up with logger and plotter  
#########################################################
logger.log2file_rigidbody()
logger.log2file_cmd()
logger.log2file_ftau()
plotter.plot_rigidbody(logger)
plotter.plot_cmd(logger)
