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

self.RWc_lacc = n1 # Let B(t) be a Weiner process, with mean 0 and
                       # variance (RWc^2*t), RWc is called the random walk
                       # coefficient. This is meant to represent the effect
                       # of unmodelled forces on the linear acceleration, by 
                       # saying that on average, overt time t we have an
                       # effect/contribution to the linear acceleration 
                       # with variance (RWc^2*t).
                       # If we sample this process with Ts, BB(k)~B(t=k*Ts)
                       # and we want Var(BB(k)) = Var(B(t=kTs))
                       # and say that BB(k+1) = BB(k) + W, where W is a 
                       # white noise process, then var(W) = RWc^2/Ts
    self.RWc_aacc = n2
	
    