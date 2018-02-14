# -*- coding: utf-8 -*-
"""
Created on Wed Feb 13 18:23:59 2018

@author: Michel
"""
import numpy as np
import math
def controller(xgv,ygv,d):
    L = 0.25
    vmax = 2
    deltamax = math.radians(30)
    u = np.zeros((2,1))
 
    goalPointVector = np.array([[xgv,ygv]]).T
    headingVector = np.array([[1,0]]).T
    
    alpha = math.atan2(goalPointVector[0]*headingVector[1]-goalPointVector[1]*headingVector[0],
                       goalPointVector[0]*headingVector[0]+goalPointVector[1]*headingVector[1])
    
    #compute steering control
    kappa = 2*math.sin(alpha)/d;
    u[1] = -math.atan(kappa*L)
    
    if u[1] > deltamax:
        u[1] = deltamax
        
    if u[1] < -deltamax:
        u[1] = -deltamax
    
    #compute throttle control
    k = 1.1
    u[0] = vmax*math.tanh(math.pi - k*math.pi*(abs(u[1])/deltamax));
    u[0] = u[0]/vmax
    
    return u
    
