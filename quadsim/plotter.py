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

class plotter:
    """ Handles the plotting from a logger object  """

    def plot_rigidbody(self, log):
        fig, ax_lst = plt.subplots(nrows=3, ncols=1) 
        fig.suptitle('Rigid Body Position Plot')  
     
        time = np.asarray(log.RigidBodyLogger_time)
        pos = np.asarray(log.RigidBodyLogger_pos)
     
        ax_lst[0].plot( time, pos[:,0], marker = "o" )
        ax_lst[0].set_title("X-axis")
     
        ax_lst[1].plot( time, pos[:,1], marker = "o" )
        ax_lst[1].set_title("Y-axis")
     
        ax_lst[2].plot( time, pos[:,2], marker = "o" ) 
        ax_lst[2].set_title("Z-axis")
     