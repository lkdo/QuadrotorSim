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

# Local quadsim imports
from context import rb
from context import qftau_cf
from context import log  
from context import envir
from context import ut
from context import plot
from context import pid
from context import cts 

plus = True
name = "Run_01"

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
        
        self.accept("time-y",self.call_camera_yaw)
        
    def call_fw(self, when):
        global thrust_ref, roll_ref, pitch_ref, yaw_ref 
        global panda3D_app
        pitch_ref = pitch_ref + math.pi/180.0 * 1  # ref are in radians
        if ( pitch_ref > math.pi/180.0 * 80 ):
            pitch_ref = math.pi/180.0 * 80
        panda3D_app.screenText_TRPY(thrust_ref,roll_ref, pitch_ref, yaw_ref)
    
    def call_bw(self, when):
        global thrust_ref, roll_ref, pitch_ref, yaw_ref 
        global panda3D_app
        pitch_ref = pitch_ref - math.pi/180.0 * 1  # ref are in radians
        if ( pitch_ref < -math.pi/180.0 * 80 ):
            pitch_ref = - math.pi/180.0 * 80
        panda3D_app.screenText_TRPY(thrust_ref,roll_ref, pitch_ref, yaw_ref)
        
    def call_left(self, when):
        global thrust_ref, roll_ref, pitch_ref, yaw_ref 
        global panda3D_app
        roll_ref = roll_ref -  math.pi/180.0 * 1  # ref are in radians
        if ( roll_ref < - math.pi/180.0 * 80 ):
                roll_ref = - math.pi/180.0 * 80
        panda3D_app.screenText_TRPY(thrust_ref,roll_ref, pitch_ref, yaw_ref)
        
    def call_right(self, when):
        global thrust_ref, roll_ref, pitch_ref, yaw_ref 
        global panda3D_app
        roll_ref = roll_ref +  math.pi/180.0 * 1  # ref are in radians
        if ( roll_ref >  math.pi/180.0 * 80 ):
                roll_ref = math.pi/180.0 * 80
        panda3D_app.screenText_TRPY(thrust_ref,roll_ref, pitch_ref, yaw_ref)
        
    def call_yaw_cw(self, when):
        global thrust_ref, roll_ref, pitch_ref, yaw_ref 
        global panda3D_app
        yaw_ref = yaw_ref - math.pi/180.0 * 1  # ref are in radians
        if  (yaw_ref <= -math.pi  ):
            yaw_ref =  2*math.pi + yaw_ref 
        panda3D_app.screenText_TRPY(thrust_ref,roll_ref, pitch_ref, yaw_ref)
        
    def call_yaw_ccw(self, when):
        global thrust_ref, roll_ref, pitch_ref, yaw_ref 
        global panda3D_app
        yaw_ref = yaw_ref + math.pi/180.0 * 1  # ref are in radians
        if  (yaw_ref >= math.pi):
               yaw_ref =  yaw_ref - 2*math.pi
        panda3D_app.screenText_TRPY(thrust_ref,roll_ref, pitch_ref, yaw_ref)

    def call_dz(self, when):
        global thrust_ref, roll_ref, pitch_ref, yaw_ref 
        global panda3D_app
        global qrb 
        thrust_ref = thrust_ref + 1/400*qrb.mass*cts.g_CONST
        panda3D_app.screenText_TRPY(thrust_ref,roll_ref, pitch_ref, yaw_ref)
        
    def call_dz_neg(self, when):
        global thrust_ref, roll_ref, pitch_ref, yaw_ref 
        global panda3D_app
        global qrb
        thrust_ref = thrust_ref - 1/400*qrb.mass*cts.g_CONST
        panda3D_app.screenText_TRPY(thrust_ref,roll_ref, pitch_ref, yaw_ref)
    
    def call_camera_yaw(self,when):
        global panda3D_app
        panda3D_app.camera_yaw = not panda3D_app.camera_yaw
        
class Panda3DApp(ShowBase):
 
    def __init__(self, camera_yaw):
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
        self.textObject_TRPY = OnscreenText(" Hello ", pos = (0, -0.8), scale = 0.07,
                                                 fg=(255,255,255,1), bg=(0,0,0,1), mayChange=True,
                                                 font = monospaced_font)
        self.textObject_pos = OnscreenText(" Hello ", pos = (0, +0.8), scale = 0.07,
                                                 fg=(255,255,255,1), bg=(0,0,0,1), mayChange=True,
                                                 font = monospaced_font)
        self.camera_yaw = camera_yaw
        
    # Define a procedure to move the camera.
    def followQuadCameraTask(self, task):
        h=self.quadrotor.getH()*math.pi/180.0
        if self.camera_yaw is False :
            self.camera.setPos(self.quadrotor.getX()-20, self.quadrotor.getY(), self.quadrotor.getZ()+3)
        else:
            self.camera.setPos(self.quadrotor.getX()-20*math.cos(h), 
                                     self.quadrotor.getY()-20*math.sin(h), 
                                     self.quadrotor.getZ()+3)
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
dt_sim = 0.001  # seconds , 1000 Hz
T_sim = 100     # seconds
dt_log = 0.1  # seconds
dt_vis = 1/60   # 60 frame per second 

# AttStab parameter 
roll_ref = 0   # radians
pitch_ref = 0 # radians
yaw_ref = 0  # radians
dt_ctrl_rate = 0.002 # 500 Hz
dt_ctrl_angle = 0.004 # 250 Hz
omega_ref = np.zeros(3)
alpha_ref = np.zeros(3)
tau_ref = np.zeros(3)

tau = 0.005

#pid_rollrate = pid.PID(70, 0, 0, 20, -20, tau)
#pid_pitchrate = pid.PID(70, 0, 0, 20, -20, tau)
#pid_yawrate = pid.PID(70,  50, 0, 20, -20, tau)
#pid_pitch = pid.PID(3.5, 2.0, 0, 20.0, -20.0, tau)
#pid_roll = pid.PID(3.5, 2.0, 0, 20.0, -20.0, tau)
#pid_yaw = pid.PID(10, 1, 0.35, 20.0, -20.0, tau)

#pid_rollrate = pid.PID(2, 0, 0, 20, -20, tau)
#pid_pitchrate = pid.PID(2, 0, 0, 20, -20, tau)
#pid_yawrate = pid.PID(2,  50, 0, 20, -20, tau)
#pid_pitch = pid.PID(1, 0.5, 0, 20.0, -20.0, tau)
#pid_roll = pid.PID(1, 0.5, 0, 20.0, -20.0, tau)
#pid_yaw = pid.PID(2, 0.15, 0.05, 20.0, -20.0, tau)

pid_rollrate = pid.PID(10, 0, 0.8, 4*360*math.pi/180,-4*360*math.pi/180, tau)
pid_pitchrate = pid.PID(10, 0, 0.8, 4*360*math.pi/180,-4*360*math.pi/180, tau)
pid_yawrate = pid.PID(15, 0, 2.5, 4*360*math.pi/180, -4*360*math.pi/180, tau)
pid_pitch = pid.PID(2, 0.5, 0, 2*360*math.pi/180, -2*360*math.pi/180, tau)
pid_roll = pid.PID(2, 0.5, 0, 2*360*math.pi/180, -2*360**math.pi/180, tau)
pid_yaw = pid.PID(3, 0.5, 0, 2*360*math.pi/180, -2*360**math.pi/180, tau)
         
# Quadrotor Initialization
############################################
qftau = qftau_cf.QuadFTau_CF(0,plus)
qftau_s = qftau_cf.QuadFTau_CF_b(qftau.cT, qftau.cQ, qftau.radius, 
                                                                   qftau.input2omegar_coeff, plus)
qrb = rb.rigidbody_q(pos, q, vb, omegab, qftau.mass, qftau.I)

thrust_ref = qrb.mass*cts.g_CONST

# Initialize the logger object 
fullname = "testresults/attstab/" + name
logger = log.Logger(fullname, name)
plotter = plot.Plotter()

# Initialize the visualization
panda3D_app = Panda3DApp(False)
panda3D_app.screenText_TRPY(thrust_ref,roll_ref, pitch_ref, yaw_ref)
readkeys = ReadKeys()

# The main simulation loop    
############################################## 
for t in np.arange(dt_sim,T_sim+dt_sim,dt_sim):
   
    # Ctrl angle frequency    
    if abs(t/dt_ctrl_angle - round(t/dt_ctrl_angle)) < 0.000001 :
         
        # measurement, no error for now, no sensor models
        meas_rpy = qrb.euler_xyz()
               
        omega_ref[0] = pid_roll.run(roll_ref-meas_rpy[0],dt_ctrl_angle)
        
        omega_ref[1] = pid_pitch.run(pitch_ref-meas_rpy[1],dt_ctrl_angle)
        
        err_yaw = yaw_ref-meas_rpy[2]
        if (err_yaw > math.pi):
            err_yaw = 2*math.pi - err_yaw
        elif (err_yaw < -math.pi):
            err_yaw = 2*math.pi + err_yaw
        omega_ref[2] = pid_yaw.run(err_yaw,dt_ctrl_angle)
        
        
    # Ctrl rate frequency    
    if abs(t/dt_ctrl_rate - round(t/dt_ctrl_rate)) < 0.000001 :
        
        # measurement, no error for now, no sensor models
        meas_omegab = qrb.omegab
          
        alpha_ref[0] = pid_rollrate.run(omega_ref[0]-meas_omegab[0], dt_ctrl_rate)
        alpha_ref[1] = pid_pitchrate.run(omega_ref[1]-meas_omegab[1],dt_ctrl_rate)
        alpha_ref[2] = pid_yawrate.run(omega_ref[2]-meas_omegab[2],dt_ctrl_rate)
        tau_ref = qftau.I@alpha_ref + ut.skew(meas_omegab)@qftau.I@meas_omegab
        
    # Calculate the command based on tau_ref and thrust_ref 
    cmd = qftau_s.fztau2cmd(np.array([thrust_ref,tau_ref[0],tau_ref[1],tau_ref[2]]))
    
    # Calculate body-based forces and torques
    fb, taub = qftau.input2ftau(cmd,qrb.vb)
    # Calculate environment-based forces and torques ( only gravity for now)
    fe_e, taue_e = envir.applyenv2ftaue(qrb)
   # Sum them up 
    fb = fb + np.transpose(qrb.rotmb2e)@fe_e
    taub = taub + np.transpose(qrb.rotmb2e)@taue_e
    
    # Run the kinematic/time forward
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
        logger.log_attstab(t,np.array([roll_ref,pitch_ref,yaw_ref]),
                                      np.array([omega_ref[0],omega_ref[1],omega_ref[2]]),
                                      np.array([alpha_ref[0],alpha_ref[1],alpha_ref[2]]))
        
# End of program 
#################################################
logger.log2file_rigidbody()
logger.log2file_cmd()
logger.log2file_attstab()
plotter.plot_rigidbody(logger)
plotter.plot_cmd(logger)
plotter.plot_attstab(logger)
