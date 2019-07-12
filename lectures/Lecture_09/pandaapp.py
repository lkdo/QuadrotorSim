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

""" Panda3D related stuff: Keyboard controls class and game engine class """

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

# Global imports 
import numpy as np 
import math 

# Panda 3D imports 
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.showbase import DirectObject
from panda3d.core import LQuaternionf
from direct.gui.OnscreenText import OnscreenText

######################################################
class ReadKeys(DirectObject.DirectObject):
    """ Panda3D class to catch keyboard inputs for commanding the quadrotor model  """
   
    def __init__(self,cmd,plus,panda3D_app ):
        
        self.cmd = cmd
        self.plus = plus
        self.panda3D_app = panda3D_app
        self.exitpressed = False
        
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
        
        self.accept ("time-escape",self.call_exitpressed)
        
    def call_fw(self, when):
        if self.plus is False:
            self.cmd = self.cmd + np.array([ -5.0, -5.0, 5.0, 5.0 ])
        else:
            self.cmd = self.cmd + np.array([ -10.0, 0.0, +10, 0.0 ])
        self.panda3D_app.screenText_cmd(self.cmd)
         
    def call_bw(self, when):
        if self.plus is False:
            self.cmd = self.cmd + np.array([ +5.0, +5.0, -5.0, -5.0 ])
        else:
            self.cmd = self.cmd + np.array([ +10.0, 0.0, -10.0, 0.0 ])
        self.panda3D_app.screenText_cmd(self.cmd)
        
    def call_left(self, when):
        if  self.plus is False:
            self.cmd = self.cmd + np.array([ 5.0, -5.0, -5.0, +5.0 ])
        else:
            self.cmd = self.cmd + np.array([ 0.0, -10.0, 0.0, +10.0 ])
        self.panda3D_app.screenText_cmd(self.cmd)
        
    def call_right(self, when):
        if self.plus is False:
            self.cmd = self.cmd + np.array([ -5.0, +5.0, +5.0, -5.0 ])	
        else:
            self.cmd = self.cmd + np.array([ 0.0, +10.0, 0.0, -10.0 ])	
        self.panda3D_app.screenText_cmd(self.cmd)
        
    def call_dz(self, when):
        self.cmd += np.array([ +5.0, +5.0, +5.0, +5.0 ])
        self.panda3D_app.screenText_cmd(self.cmd)
        
    def call_dz_neg(self, when):
        self.cmd += np.array([ -5.0, -5.0, -5.0, -5.0 ])	
        self.panda3D_app.screenText_cmd(self.cmd)
        
    def call_yaw_cw(self, when):
        self.cmd += np.array([ +5.0, -5.0, +5.0, -5.0 ])	
        self.panda3D_app.screenText_cmd(self.cmd)
        
    def call_yaw_ccw(self, when):
        self.cmd += np.array([ -5.0, +5.0, -5.0, +5.0 ])	
        self.panda3D_app.screenText_cmd(self.cmd)
       
    def call_exitpressed(self,when):
        self.exitpressed = True
        
########################################################        


class Panda3DApp(ShowBase):
    """ Main class of the game engine Panda3D """
   
    def __init__(self, plus, qrb, cmd):
        
        self.qrb = qrb 
        """ The rigid Body object """
        
        ShowBase.__init__(self)
		
        # Load the environment model.
        self.scene = self.loader.loadModel("../models/environment") 
           
	    # Reparent the model to render.
        self.scene.reparentTo(self.render)
    
	    # Apply scale and position transforms on the model.
        self.scene.setScale(0.25, 0.25, 0.25)
        self.scene.setPos(-8, 42, 0)
		
        if plus is False:
            self.quadrotor = self.loader.loadModel("./res/CF21_cross")
        else:
            self.quadrotor = self.loader.loadModel("./res/CF21_plus")
            
        self.quadrotor.setScale(1, 1, 1)
        self.quadrotor.reparentTo(self.render)
        self.quadrotor.setPos(0,0,3)
            
        # Add the move  procedure to the task manager.
        self.taskMgr.add(self.moveActorTask, "MoveQuadTask")
        
		# Add the followQuadCameraTask  procedure to the task manager.
        self.taskMgr.add(self.followQuadCameraTask, "followQuadCameraTask")
        
        monospaced_font = self.loader.loadFont("cmtt12.egg")
        self.textObject_pos = OnscreenText("Position and Attitude", pos = (0, +0.8), scale = 0.07,
                                                 fg=(255,255,255,1), bg=(0,0,0,1), mayChange=True,
                                                 font = monospaced_font)
        text = "R1={0: .0f},R2={1: .0f},R3={2: .0f},R={3: .0f}".format(cmd[0],cmd[1],cmd[2],cmd[3])        
        self.textObject_cmd = OnscreenText(text, pos = (0, -0.8), scale = 0.07,
                                                 fg=(255,255,255,1), bg=(0,0,0,1), mayChange=True,
                                                 font = monospaced_font)


    # Define a procedure to move the camera.
    
    def followQuadCameraTask(self, task):
        self.camera.setPos(self.quadrotor.getX()-20, self.quadrotor.getY(), self.quadrotor.getZ()+3)
        self.camera.lookAt(self.quadrotor)
        return Task.cont


    # Define a procedure to move the panda actor 
    
    def moveActorTask(self, task):
        pos = self.qrb.pos
        q = LQuaternionf(self.qrb.q[0],self.qrb.q[1],self.qrb.q[2],self.qrb.q[3])
        self.quadrotor.setQuat(q)
        self.quadrotor.setPos(pos[0],pos[1],pos[2])
        return Task.cont

    
    # function to write on screen 
    
    def screenText_cmd(self,cmd):
        text = "R1={0:5.0f},R2={1:5.0f},R3={2:5.0f},R4={3:5.0f}".format(cmd[0],cmd[1],cmd[2],cmd[3])
        self.textObject_cmd.text = text


    def screenText_pos(self,pos,RPY):
        text = "X={0: 8.2f},Y={1: 8.2f},Z={2: 8.2f},R={3: 8.2f},P={4: 8.2f},Y={5: 8.2f}".format(
                        pos[0],pos[1],pos[2],
                        RPY[0]*180/math.pi,RPY[1]*180/math.pi,RPY[2]*180/math.pi)
        self.textObject_pos.text = text 