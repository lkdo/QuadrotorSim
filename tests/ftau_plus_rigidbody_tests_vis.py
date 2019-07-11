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

""" Component testing for the rigidbody and quadftau_cf modules """

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

import numpy as np
import math

# Panda 3D 
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.showbase import DirectObject
from panda3d.core import LQuaternionf
from direct.gui.OnscreenText import OnscreenText

from context import rb
from context import qftau_cf
from context import log  
from context import envir
from context import ut
from context import plot

plus = True
name = "Manual"

class ReadKeys(DirectObject.DirectObject):
    
    def __init__(self):
        
        self.accept("time-arrow_up", self.call_fw)
        self.accept("time-arrow_up-repeat", self.call_fw)
        
        self.accept("time-arrow_down", self.call_bw)
        self.accept("time-arrow_down-repeat", self.call_bw)
        
        self.accept("time-arrow_left", self.call_left)
        self.accept("time-arrow_left-repeat", self.call_left)
        
        self.accept("time-arrow_right", self.call_right)
        self.accept("time-arrow_right-repeat", self.call_right)
        
        self.accept("time-a", self.call_dz)
        self.accept("time-a-repeat", self.call_dz)
        
        self.accept("time-z", self.call_dz_neg)
        self.accept("time-z-repeat", self.call_dz_neg)
        
        self.accept("time-j",self.call_yaw_cw)
        self.accept("time-j-repeat",self.call_yaw_cw)
        
        self.accept("time-g",self.call_yaw_ccw)
        self.accept("time-g-repeat",self.call_yaw_ccw)
        
    def call_fw(self, when):
        global cmd 
        if plus is False:
            cmd = cmd + np.array([ -5.0, -5, 5, 5 ])
        else:
            cmd = cmd + np.array([ -10.0, 0, +10, 0 ])
    
    def call_bw(self, when):
        global cmd 
        if plus is False:
            cmd = cmd + np.array([ +5, +5, -5, -5.0 ])
        else:
            cmd = cmd + np.array([ +10, 0, -10, 0.0 ])
        
    def call_left(self, when):
        global cmd 
        if  plus is False:
            cmd = cmd + np.array([ 5.0, -5, -5, +5 ])
        else:
            cmd = cmd + np.array([ 0, -10, 0, +10.0 ])
        
    def call_right(self, when):
        global cmd 
        if plus is False:
            cmd = cmd + np.array([ -5.0, +5, +5, -5.0 ])	
        else:
            cmd = cmd + np.array([ 0, +10.0, 0, -10.0 ])	
        
    def call_dz(self, when):
        global cmd 
        cmd += np.array([ +5, +5, +5, +5 ])
        
    def call_dz_neg(self, when):
        global cmd 
        cmd += np.array([ -5, -5, -5, -5 ])	
        
    def call_yaw_cw(self, when):
        global cmd
        cmd += np.array([ +5, -5, +5, -5 ])	
    
    def call_yaw_ccw(self, when):
        global cmd
        cmd += np.array([ -5, +5, -5, +5 ])	
        
class Panda3DApp(ShowBase):
 
    def __init__(self):
        ShowBase.__init__(self)
		
        # Load the environment model.
        self.scene = self.loader.loadModel("../models/environment") 
           
	    # Reparent the model to render.
        self.scene.reparentTo(self.render)
    
	    # Apply scale and position transforms on the model.
        self.scene.setScale(0.25, 0.25, 0.25)
        self.scene.setPos(-8, 42, 0)
		
        if plus is False:
            self.quadrotor = self.loader.loadModel("../quadsim/res/CF21_cross")
        else:
            self.quadrotor = self.loader.loadModel("../quadsim/res/CF21_plus")
            
        self.quadrotor.setScale(1, 1, 1)
        self.quadrotor.reparentTo(self.render)
        self.quadrotor.setPos(0,0,3)
            
        # Add the moveActor procedure to the task manager.
        self.taskMgr.add(self.moveActorTask, "MoveQuadTask")
        
        # Add the rotateActor procedure to the task manager.
        self.taskMgr.add(self.rotateActorTask, "RotateQuadTask")
		
		# Add the followQuadCameraTask  procedure to the task manager.
        self.taskMgr.add(self.followQuadCameraTask, "followQuadCameraTask")
        
        monospaced_font = loader.loadFont("cmtt12.egg")
        self.textObject_pos = OnscreenText(" Hello ", pos = (0, +0.8), scale = 0.07,
                                                 fg=(255,255,255,1), bg=(0,0,0,1), mayChange=True,
                                                 font = monospaced_font)


    # Define a procedure to move the camera.
    def followQuadCameraTask(self, task):
        self.camera.setPos(self.quadrotor.getX()-20, self.quadrotor.getY(), self.quadrotor.getZ()+3)
        self.camera.lookAt(self.quadrotor)
        return Task.cont

    # Define a procedure to move the panda actor 
    def moveActorTask(self, task):
        global qrb
        pos = qrb.pos
        self.quadrotor.setPos(pos[0],pos[1],pos[2])
        return Task.cont
    
    # Define a procedure to rotate the actor 
    def rotateActorTask(self, task):
        global qrb
        q=LQuaternionf(qrb.q[0],qrb.q[1],qrb.q[2],qrb.q[3])
        self.quadrotor.setQuat(q)
        return Task.cont
 
    # function to write on screen 
    def screenText_TRPY(self,T,R,P,Y):
        text = "T={0:.4f},R={1:.2f},P={2:.2f},Y={3:.2f}".format(T,R*180/math.pi, 
                                                                                        P*180/math.pi,Y*180/math.pi)
        self.textObject_TRPY.text = text
    def screenText_pos(self,pos,RPY):
        text = "X={0:.2f},Y={1:.2f},Z={2:.2f}    R={3:.2f},P={4:.2f},Y={5:.2f}".format(
                        pos[0],pos[1],pos[2],
                        RPY[0]*180/math.pi,RPY[1]*180/math.pi,RPY[2]*180/math.pi)
        self.textObject_pos.text = text
        
# Initialization values for the states of the quadrotor
pos = np.array([0,0,3])
q = np.array([1,0,0,0])
rotmb2e = ut.quat2rotm(q) 
vb = np.array([0,0,0])
omegab = np.array([0,0,0])

# Simulation parameters
dt_sim = 0.001  # seconds
T_sim = 100     # seconds
dt_log = 0.001        # seconds
dt_vis = 1/60   # 60 frame per second 

# Test Case 
#######################################################################
cmd = 37278 * np.ones(4)
qftau = qftau_cf.QuadFTau_CF(0,plus)
qrb = rb.rigidbody_q(pos, q, vb, omegab, qftau.mass, qftau.I)

# Initialize the logger object 
fullname = "testresults/ftau_plus_rigidbody/" + name
logger = log.Logger(fullname, name)
plotter = plot.Plotter()

# Initialize the visualization
panda3D_app = Panda3DApp()
readkeys = ReadKeys()


# The main simulation loop     
for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):
   
    # Calculate body-based forces and torques
    fb, taub = qftau.input2ftau(cmd,qrb.vb)
    
    fe_e, taue_e = envir.applyenv2ftaue(qrb)
    
    fb = fb + np.transpose(qrb.rotmb2e)@fe_e
    taub = taub + np.transpose(qrb.rotmb2e)@taue_e
    
    # Run the kinematic / time forward
    qrb.run_quadrotor(dt_sim, fb, taub)

    # Visualization frequency    
    if abs(t/dt_vis - round(t/dt_vis)) < 0.000001 :
        panda3D_app.taskMgr.step()
        panda3D_app.screenText_pos(qrb.pos,qrb.euler_xyz())
        
    # Logging frequency    
    if abs(t/dt_log - round(t/dt_log)) < 0.000001 :
        qrb.check()
        logger.log_rigidbody(t, qrb)
        logger.log_cmd(t, cmd)
        
# End of program 
#logger.log2file_rigidbody()
#logger.log2file_cmd()
plotter.plot_rigidbody(logger)
plotter.plot_cmd(logger)
