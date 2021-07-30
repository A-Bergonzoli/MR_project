#!/usr/bin/env python3

import numpy as np

def io_linearization_control_law(y_1, y_2, theta, y_1d, y_2d, doty_1d, doty_2d, b, ff):
    # define the two control gains
    k_1 = 0.7
    k_2 = 0.7
    
    if ff: # FF + P controller
        # compute virtual control inputs (i.e. doty_1, doty_2)
        u_1 = doty_1d + k_1*(y_1d - y_1)
        u_2 = doty_2d + k_2*(y_2d - y_2)
    else: # P controller
        # compute virtual control inputs (i.e. doty_1, doty_2)
        u_1 = k_1*(y_1d - y_1)
        u_2 = k_2*(y_2d - y_2)

    # compute real control inputs
    v = np.cos(theta) * u_1 + np.sin(theta) * u_2
    w = u_2/b * np.cos(theta) - u_1/b *np.sin(theta) #dottheta

    return np.array([v, w])



'''
This function implements a simple constant gain controller for $k_1(v_d,\omega_d)$
''' 
zeta = 1.7
a = 2


def k1(v_d, w_d):
    global zeta, a
    return 2*zeta*np.sqrt(v_d**2 + a*w_d**2)
    
'''
This function implements a simple constant gain controller for $k_3(v_d,\omega_d)$
''' 

def k3(v_d, w_d):
    global zeta, a
    return 2*zeta*np.sqrt(v_d**2 + a*w_d**2)
    

'''
This function implements the control. k1 and k3 functions
are used to (possibly) implement time varying gains, whereas
the gain k2 is set in the function.
'''
def nonLinear_control_law(e, v_d, w_d):
    global a, zeta
    k2 = a
    
    u_1 = -k1(v_d, w_d) * e[0]
    
    # be sure that if e[2] = 0 sin(e[2])/e[2] is computes to 1.0
    if e[2] == 0:
        u_2 = -k2*v_d*e[1] - k3(v_d,w_d)*e[2]
    else:
        u_2 = -k2*v_d*np.sin(e[2])/e[2]*e[1] - k3(v_d,w_d)*e[2]
    # compute control inputs
    v = v_d * np.cos(e[2]) - u_1
    w = w_d - u_2
    
    return np.array([v, w])
