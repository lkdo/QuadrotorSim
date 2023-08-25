import utils
import math
from dataclasses import dataclass

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

def buildCtrlReference(ref_mode):

    if ref_mode == "step":
      
        name = "step"
    
        step_1 = StepAndRampMetaSignal(3,13,20)
        step_2 = StepAndRampMetaSignal(63,73,-20)
        pos_x_ref = utils.build_signal_step(0,90,0,step_1, step_2)

        step_1 = StepAndRampMetaSignal(23,33,20)
        step_2 = StepAndRampMetaSignal(63,73,-20)
        pos_y_ref = utils.build_signal_step(0,90,0,step_1, step_2)

        step_1 = StepAndRampMetaSignal(43,53,3)
        step_2 = StepAndRampMetaSignal(63,73,1)
        pos_z_ref = utils.build_signal_step(0,90,3,step_1, step_2)

        step_1 = StepAndRampMetaSignal(3,13,450*math.pi/180)
        step_2 = StepAndRampMetaSignal(23,33,45*math.pi/180)
        step_3 = StepAndRampMetaSignal(43,53,45*math.pi/180)
        step_4 = StepAndRampMetaSignal(63,73,-45*math.pi/180)
        yaw_ref = utils.build_signal_step(0,90,0,step_1, step_2,step_3,step_4)

    elif ref_mode == "shortstep":
      
        name = "shortstep"
    
        step_1 = StepAndRampMetaSignal(1,20,20)
        pos_x_ref = utils.build_signal_step(0,20,0,step_1)

        step_1 = StepAndRampMetaSignal(1,20,20)
        pos_y_ref = utils.build_signal_step(0,20,0,step_1)

        step_1 = StepAndRampMetaSignal(10,20,6)
        pos_z_ref = utils.build_signal_step(0,20,6,step_1)

        step_1 = StepAndRampMetaSignal(10,20,0*math.pi/180)
        yaw_ref = utils.build_signal_step(0,20,0,step_1)

    elif ref_mode == "ramp":
    
        name = "ramp"
     
        ramp_1 = StepAndRampMetaSignal(3,13,20)
        ramp_2 = StepAndRampMetaSignal(63,73,-20)
        pos_x_ref = utils.build_signal_ramp(0,90,0,ramp_1, ramp_2)

        ramp_1 = StepAndRampMetaSignal(23,33,20)
        ramp_2 = StepAndRampMetaSignal(63,73,-20)
        pos_y_ref = utils.build_signal_ramp(0,90,0,ramp_1, ramp_2)

        ramp_1 = StepAndRampMetaSignal(43,53,20)
        ramp_2 = StepAndRampMetaSignal(63,73,20)
        pos_z_ref = utils.build_signal_ramp(0,90,3,ramp_1, ramp_2)

        ramp_1 = StepAndRampMetaSignal(3,13,120*math.pi/180)
        ramp_2 = StepAndRampMetaSignal(23,33,120*math.pi/180)
        ramp_3 = StepAndRampMetaSignal(43,53,120*math.pi/180)
        ramp_4 = StepAndRampMetaSignal(63,73,-120*math.pi/180)
        yaw_ref = utils.build_signal_ramp(0,90,0,ramp_1, ramp_2, ramp_3, ramp_4)

    elif ref_mode == "sin":
    
        name = "sin"
    
        f = 1.0/10.0
    
        sin_1 = SinMetaSignal(3,10,1,1/f,20)
        sin_2 = SinMetaSignal(63,10,1,1/f,20)
        pos_x_ref = utils.build_signal_sin(0,27,0,sin_1, sin_2)

        sin_1 = SinMetaSignal(23,10,1,1/f,20)
        sin_2 = SinMetaSignal(63,10,1,1/f,20)
        pos_y_ref = utils.build_signal_sin(0,90,0,sin_1, sin_2)

        sin_1 = SinMetaSignal(43,10,1,1/f,20)
        sin_2 = SinMetaSignal(63,10,1,1/f,20)
        pos_z_ref = utils.build_signal_sin(0,90,3,sin_1, sin_2)
    
        sin_1 = SinMetaSignal(3,60*math.pi/180,1,1/f,20)
        sin_2 = SinMetaSignal(23,60*math.pi/180,1,1/f,20)
        sin_3 = SinMetaSignal(43,60*math.pi/180,1,1/f,20)
        sin_4 = SinMetaSignal(63,60*math.pi/180,1,1/f,20)
        yaw_ref = utils.build_signal_sin(0,90,0,sin_1, sin_2, sin_3, sin_4)
    
    elif ref_mode == "hover":
      
        name = "hover"
    
        step_1 = StepAndRampMetaSignal(3,13,0)
        step_2 = StepAndRampMetaSignal(63,73,0)
        pos_x_ref = utils.build_signal_step(0,90,0,step_1, step_2)

        step_1 = StepAndRampMetaSignal(3,13,0)
        step_2 = StepAndRampMetaSignal(63,73,0)
        pos_y_ref = utils.build_signal_step(0,90,0,step_1, step_2)

        step_1 = StepAndRampMetaSignal(3,13,4)
        step_2 = StepAndRampMetaSignal(63,73,4)
        pos_z_ref = utils.build_signal_step(0,90,3,step_1, step_2)

        step_1 = StepAndRampMetaSignal(3,13,0*math.pi/180)
        step_2 = StepAndRampMetaSignal(63,73,0*math.pi/180)
        yaw_ref = utils.build_signal_step(0,90,0,step_1, step_2)

    else:

        name = "manual"

    return pos_x_ref, pos_y_ref, pos_z_ref, yaw_ref, name