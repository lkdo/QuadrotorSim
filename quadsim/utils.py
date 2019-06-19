# -*- coding: utf-8 -*-
"""
Part of the QuadrotorSim package
Copyright (C) 2019  Luminita-Cristiana Totu
Contact: luminita.totu@gmail.com

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this package, in a file called LICENSE.  
If not, see <https://www.gnu.org/licenses/>.
"""
	
import numpy as np

# the Skew-symmetric matrix form of a vector
# helpful in writing the vector cross product
def SkS(X):
  return np.array([[0,-X[3-1],X[2-1]],[X[3-1],0,-X[1-1]],[-X[2-1],X[1-1],0]])
