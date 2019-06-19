# -*- coding: utf-8 -*-
"""
Created on Tue Jun 18 21:31:39 2019

@author: Luminita-Cristiana Totu
"""

import numpy as np
import matplotlib.pyplot as plt

class Plotter:
  """ Handles the plotting from a logger object  """

  def plot_RigidBody(self, log):
     fig, ax_lst = plt.subplots(3, 1) 
     fig.suptitle('RB Position Plot')  
     
     time = np.asarray(log.RigidBodyLogger_time)
     pos = np.asarray(log.RigidBodyLogger_pos)
     
     ax_lst[0].plot( time, pos[:,0], marker = "o" )
     ax_lst[0].set_title("X-axis")
     
     ax_lst[1].plot( time, pos[:,1], marker = "o" )
     ax_lst[1].set_title("Y-axis")
     
     ax_lst[2].plot( time, -pos[:,2], marker = "o" ) #minus because of the NED 
     ax_lst[2].set_title("Z-axis")
     