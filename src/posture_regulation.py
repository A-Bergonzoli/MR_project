#!/usr/bin/env python3

'''
This file implements posture regulation techniques.
'''

import numpy as np 
import math

# Cartesian Regulation (theta is not controlled)
def cartesian_regulation_control_law(x, y, theta):

    #gains
    # k1 = 0.3  # goes backwards most of the time
    # k2 = 0.1
    k1 = 0.02
    k2 = 0.6
    
    #control inputs
    v = -k1 * (x * np.cos(theta) + y * np.sin(theta))
    w = k2 * (np.arctan2(y, x) + np.pi - theta)
	
    # return inputs
    return np.array([v, w])


# Polar coordinates approach
def posture_regulation_control_law(x, y, theta):

    #gains
    k1 = 0.2
    k2 = 0.09
    k3 = 0.01

    # polar coordinates
    rho = math.sqrt( x**2 + y**2 )
    gamma = math.atan2(y,x) - theta + math.pi
    delta = gamma + theta

    # compute inputs
    v = k1*rho*np.cos(gamma)
    w = k2*gamma + k1*( (np.sin(gamma)*np.cos(gamma))/(gamma) )*(gamma + k3*delta)

    return np.array([v, w])
