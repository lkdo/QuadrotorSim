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

""" This is the Quadrotor Simulation aka quadsim package.

It contains/will contain in a first version:
    - a rigid body kinematic module which takes forces
      and torques and integrates then into velocities, and position
      and orientation;
    - a quadrorot dynamic module, which contains a basic model 
      of forces and torques given the rotor characteristics, and
      also a noise model;
    - a sensor module that implements accelerometers and gyroscope
      ( and maybe magnetometer ) models, and a position sensor 
    - an hierarhical PID control structure for flight stabilization
    - a position controller 
    - a logger module
    - a graphical plotter module for signals 
"""

__version__ = "0.1"
__author__ = "Luminita-Cristiana Totu"
__copyright__ = "Copyright (C) 2019 Luminita-Cristiana Totu"
__license__ = "GNU GPLv3"

__all__ = [
        "rigidbody",
        "utils",
        "plotter",
        "logger",
        "cts" ]