# -*- coding: utf-8 -*-
"""
Created on Wed Jun 19 11:21:00 2019

@author: Luminita-Cristiana Totu
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
	
    