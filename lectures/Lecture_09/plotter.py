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

""" This module implements the Graphical Plotter """

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

import numpy as np
import matplotlib.pyplot as plt
import os 

class Plotter:
    """ Handles the plotting from a logger object  """
  
    def __init__(self):
        
        self.big_font_size = 25
        
    def fig_style_1(self, fig, ax_lst):
        
        fig.set_size_inches(20, 15)
        fig.set_dpi(100)
        fig.subplots_adjust(hspace=0.25)
        for ax in ax_lst:
            ax.tick_params(axis='both', which='major', labelsize=20)
            ax.tick_params(axis='both', which='minor', labelsize=10)
            for item in [ax.title, ax.xaxis.label, ax.yaxis.label]:
                     item.set_fontsize(self.big_font_size)
            ax.grid(True)    
        
    def plot_rigidbody(self, log):
        
        # saving location 
        location = log.location 
        if not os.path.exists(location):
            os.makedirs(location)
        
        # Position plot
        ################################################
        fig, ax_lst = plt.subplots(3, 1)
        fig.suptitle('Rigid Body Position', fontsize = self.big_font_size)
        time = np.asarray(log.rb_time)
        pos = np.asarray(log.rb_pos)
        
        ax_lst[0].plot( time, pos[:,0], marker = "o" )
        ax_lst[0].set_title("X-axis")
        ax_lst[1].plot( time, pos[:,1], marker = "o" )
        ax_lst[1].set_title("Y-axis")
        ax_lst[2].plot( time, pos[:,2], marker = "o" ) 
        ax_lst[2].set_title("Z-axis")
        
        self.fig_style_1(fig, ax_lst)
        
        fullname = location + "/" +  log.basename + "__rigidbody_pos.png"
        fig.savefig(fullname)
        plt.close(fig)
        
        # Euler Angles plot 
        ################################################
        fig, ax_lst = plt.subplots(3, 1)
        fig.suptitle('Rigid Body Euler-XYZ', fontsize = self.big_font_size)
        time = np.asarray(log.rb_time)
        euler = np.asarray(log.rb_euler)
        
        ax_lst[0].plot( time, euler[:,0], marker = "o" )
        ax_lst[0].set_title("Roll - rotation around the X-axis")
        ax_lst[1].plot( time, euler[:,1], marker = "o" )
        ax_lst[1].set_title("Pitch - rotation around the Y-axis")
        ax_lst[2].plot( time, euler[:,2], marker = "o" ) 
        ax_lst[2].set_title("Yaw - rotation around the Z-axis")
        
        self.fig_style_1(fig, ax_lst)
        
        fullname = location + "/" +  log.basename+ "__rigidbody_eulerXYZ.png"
        fig.savefig(fullname)
        plt.close(fig)
        
    def plot_cmd(self, log):
        
        # saving location 
        location = log.location 
        if not os.path.exists(location):
            os.makedirs(location)

        # Rotor command plot
        fig, ax_lst = plt.subplots(4, 1)
        fig.suptitle('Raw Rotor Command', fontsize = self.big_font_size)
        time = np.asarray(log.cmd_time)
        cmd_rotors = np.asarray(log.cmd_rotors)
        
        ax_lst[0].plot( time, cmd_rotors[:,0], marker = "o" )
        ax_lst[0].set_title("Rotor 1")
        ax_lst[1].plot( time, cmd_rotors[:,1], marker = "o" )
        ax_lst[1].set_title("Rotor 2")
        ax_lst[2].plot( time, cmd_rotors[:,2], marker = "o" ) 
        ax_lst[2].set_title("Rotor 3")
        ax_lst[3].plot( time, cmd_rotors[:,3], marker = "o" ) 
        ax_lst[3].set_title("Rotor 4")
        
        self.fig_style_1(fig, ax_lst)
        
        fullname = location + "/" +  log.basename + "__cmd_rotors.png"
        fig.savefig(fullname)
        plt.close(fig)
        
 