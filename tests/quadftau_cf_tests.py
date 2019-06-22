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

""" Unit Tests for the quadftau module """

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

import numpy as np
import os

from context import ut
from context import qftau_cf
from context import cts

def unitttest_template_A():
    file.write("\n\nUnit Test %s: \n" % u_name)
    file.write("####################################################\n")
    
    rotmb2e = np.identity(3)
               
    vb = np.array([0,0,0])
           
    try:
        file.write("cmd = %s\n" % np.array2string(cmd))
        fb, taub = qftau_cf.input2ftau(cmd, vb)

        # add gravity 
        fb = fb + np.dot(np.transpose(rotmb2e),
                         np.array([0,0,-qftau_cf.mass*cts.g_CONST]))
        
        file.write("fb = %s\n" % np.array2string(fb))
        file.write("taub = %s\n" % np.array2string(taub)) 
   
        # remove gravity
        fb = fb + np.dot(np.transpose(rotmb2e),
                          np.array([0,0,+qftau_cf.mass*cts.g_CONST]))
        omegar = qftau_cf.ftau2omegar(fb, taub)
        cmd_recovered = qftau_cf.omegar2input(omegar)
        file.write("cmd_recovered = %s\n" % np.array2string(cmd_recovered))
        
    except Exception as e:  
        file.write("TEST FAILED: %s\n" % e)


def unitttest_template_B():
    file_b.write("\n\nUnit Test %s: \n" % u_name)
    file_b.write("####################################################\n")
    
    rotmb2e = np.identity(3)
    
    try:
        file_b.write("cmd = %s\n" % np.array2string(cmd))
        fb, taub = qftau_cf.input2ftau_b(cmd)
        
        # add gravity 
        fb = fb + np.dot( np.transpose(rotmb2e),
                          np.array([0,0,-qftau_cf.mass*cts.g_CONST]) )
       
        file_b.write("fb = %s\n" % np.array2string(fb))
        file_b.write("taub = %s\n" % np.array2string(taub)) 

        # remove gravity
        fb = fb + np.dot(np.transpose(rotmb2e),
                          np.array([0,0,+qftau_cf.mass*cts.g_CONST]))
        omegar = qftau_cf.ftau2omegar(fb, taub)
        cmd_recovered = qftau_cf.omegar2input(omegar)
        file_b.write("cmd_recovered = %s\n" % np.array2string(cmd_recovered))
   
    except Exception as e:  
        file_b.write("TEST FAILED: %s\n" % e)
   
# Create a file for these tests
location_rb = "testresults" + "/" + "quadftau_cf"
if not os.path.exists(location_rb):
    os.makedirs(location_rb)
fullname = location_rb + "/" + "batch_01" + ".txt"
fullname_b = location_rb + "/" + "batch_01_b" + ".txt"
fullname_c = location_rb + "/" + "batch_01_c" + ".txt"
fullname_d = location_rb + "/" + "batch_01_d" + ".txt"

with open(fullname,"w") as file, open(fullname_b,"w") as file_b:
            
    u_name = "0001_cmd_zero"
    cmd = np.array([0,0,0,0])
    unitttest_template_A()
    unitttest_template_B()

    u_name = "0002_cmd_verysmall"
    cmd = np.array([800,800,800,800])
    unitttest_template_A()
    unitttest_template_B()
    
    u_name = "0003_cmd_zerotreshold"
    cmd = np.array([1000,1000,1000,1000])
    unitttest_template_A()
    unitttest_template_B()
    
    u_name = "0004_cmd_hoovering"
    cmd = np.array([37278,37278,37278,37278])
    # this value is very close to the 36000 I used in my project
    # so this is good news for the identification report 
    unitttest_template_A()
    unitttest_template_B()
    
    u_name = "0005_cmd_rolling_pos"
    cmd = np.array([38000,38000+1000,38000,38000-1000])
    unitttest_template_A()
    unitttest_template_B()
    
    u_name = "0006_cmd_rolling_neg"
    cmd = np.array([38000,38000-1000,38000,38000+1000])
    unitttest_template_A()
    unitttest_template_B()
    
    u_name = "0007_cmd_pitching_pos"
    cmd = np.array([38000-1000,38000,38000+1000,38000])
    unitttest_template_A()
    unitttest_template_B()
    
    u_name = "0008_cmd_pitching_neg"
    cmd = np.array([38000+1000,38000,38000-1000,38000])
    unitttest_template_A()
    unitttest_template_B()
    
    u_name = "0009_cmd_yawing_pos"
    cmd = np.array([38000+1000,38000,38000+1000,38000])
    unitttest_template_A()
    unitttest_template_B()
    
    u_name = "0010_cmd_yawing_neg"
    cmd = np.array([38000,38000+1000,38000,38000+1000])
    unitttest_template_A()
    unitttest_template_B()
    
    u_name = "0100_test_k_ct"
    # Test to check the relation 
    # f_thrust_i = ct * omegar_i^2
    file.write("\n\nUnit Test %s: \n" % u_name)
    file.write("####################################################\n")
    N = 1000
    thrust = np.zeros(N)
    omegar = np.zeros(N)
    for i in range(N):
        cmd = 1000 + np.random.rand(1)*(65535-1000)
        thrust[i] = qftau_cf.input2thrust_i(cmd)
        omegar[i] = qftau_cf.input2omegar_i(cmd)
            
    k_mean = np.mean(thrust / omegar**2)
    k_std = np.std(thrust / omegar**2)
    file.write("k estimate is %s with std %s. \n" % (k_mean, k_std))
                  
    u_name = "0200_test_cTcQmodelcheck"
    rotmb2e = np.identity(3)
    vb = np.array([0,0,0])
    with open(fullname_c,"w") as file_c, open(fullname_d,"w") as file_d:
        file_c.write("\n\nUnit Test %s: \n" % u_name)
        file_c.write("####################################################\n")
        
        file_d.write("\n\nUnit Test %s: \n" % u_name)
        file_d.write("####################################################\n")
        
        N = 1000
        err_fb = np.zeros((N,3))
        err_taub = np.zeros((N,3))
        for i in range(N):
            cmd = 1000 + np.random.rand(4)*(65535-1000)
            fb, taub = qftau_cf.input2ftau(cmd, vb)
            
            file_c.write("fb = %s\n" % np.array2string(fb))
            file_c.write("taub = %s\n" % np.array2string(taub)) 
           
            fb_b, taub_b = qftau_cf.input2ftau_b(cmd)
           
            file_d.write("fb = %s\n" % np.array2string(fb_b))
            file_d.write("taub = %s\n" % np.array2string(taub_b)) 
           
            err_fb[i,:] = fb-fb_b
            err_taub[i,:] = taub-taub_b
    
        err_fb_mean = np.mean(err_fb,0)
        err_fb_std = np.std(err_fb,0)
    
        err_taub_mean = np.mean(err_taub,0)
        err_taub_std = np.std(err_taub,0)
    
        file_c.write("\n##################################\n")
        file_c.write("err_fb_mean is %s with std %s. \n" % (err_fb_mean, 
                                                         err_fb_std))
        file_c.write("err_taub_mean is %s with std %s. \n" % (err_taub_mean, 
                                                         err_taub_std))
        
        file_d.write("\n##################################\n")
        file_d.write("err_fb_mean is %s with std %s. \n" % (err_fb_mean, 
                                                         err_fb_std))
        file_d.write("err_taub_mean is %s with std %s. \n" % (err_taub_mean, 
                                                         err_taub_std))
        