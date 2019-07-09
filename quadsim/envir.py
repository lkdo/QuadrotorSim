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

""" Environment module, contains gravity, surface normals, etc """

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

import numpy as np

import cts 

def applyenv2ftaue(rb):
    """ Applies enviornment forces and torques to the object 
    
    - applies gravity
    - Future: applies normals ( colision to ground )
    - applies dynamic noise / distrurbances 
    
    Calculations take place in Earth frame, NWU (north-west-up) 
    
    """
    
    # Add gravity 
    fe_e = np.array([0.0,0.0,-rb.mass*cts.g_CONST])
    taue_e = np.array([0.0,0.0,0.0])
    
    return fe_e, taue_e