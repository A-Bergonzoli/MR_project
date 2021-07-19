#!/usr/bin/env python3

'''
This functions will compute the consensus control laws inputs for the agents
'''

import numpy as np
import math
import posture_regulation

import trajectory_generation


# Rendez-vous control law: all agents are attracted to the average of their initial positions 
# w/o repulsion
def rv_control_law_a(q, N_i):

    v = np.zeros(len(N_i))
    w = np.zeros(len(N_i))
    i = 0

    for neighbour in N_i:
        # Compute phi
        phi = math.atan2( neighbour[1]-q[1], neighbour[0]-q[0] )
        # Compute inputs
        v[i] = ( (q[0] - neighbour[0])*np.cos(q[2]) + (q[1] - neighbour[1])*np.sin(q[2]) )
        w[i] = ( q[2] - phi )
        # Update
        i += 1

    return np.array([sum(v), sum(w)])


# Rendez-vous control law: all agents are attracted to the average of their initial positions 
# w/ repulsion. [NOTE: This function is the one used by rendezvous.py ]
def rv_control_law_ar1(q, N_i):

    c = 1.2 # clearence
    v = np.zeros(len(N_i))
    w = np.zeros(len(N_i))
    i = 0

    for neighbour in N_i:
        # Compute kd
        norm = math.sqrt( (q[0] - neighbour[0])**2 + (q[1] - neighbour[1])**2 )
        kd = 1 - math.exp( c - norm**2 )
        # Compute phi
        phi = math.atan2( neighbour[1]-q[1], neighbour[0]-q[0] )
        # Compute inputs
        v[i] = ( (q[0] - neighbour[0])*np.cos(q[2]) + (q[1] - neighbour[1])*np.sin(q[2]) )
        w[i] = ( q[2] - kd*phi ) 
        # Update
        i += 1

        errore_angolare = 3*math.pi/2 - q[2]

    return np.array([0.1*sum(v), 0.1*sum(w)*1.5*errore_angolare ])


# Rendez-vous control law: all agents are attracted to the same point, which is the origin (0,0) 
# w/ repulsion. [NOTE: This function is the one used by agents.py ]
def rv_control_law_ar2(q, N_i):

    c = 1.2 # clearence
    v = np.zeros(len(N_i))
    w = np.zeros(len(N_i))
    i = 0

    (u1, u2) = posture_regulation.posture_regulation_control_law(q[0], q[1], q[2])

    for neighbour in N_i:
        # Compute kd
        norm = math.sqrt( (q[0] - neighbour[0])**2 + (q[1] - neighbour[1])**2 )
        kd = 1 - math.exp( c - norm**2 )
        # Compute phi
        phi = math.atan2( neighbour[1]-q[1], neighbour[0]-q[0] )
        # Compute inputs
        v[i] = ( (q[0] - neighbour[0])*np.cos(q[2]) + (q[1] - neighbour[1])*np.sin(q[2]) ) * kd  - u1
        w[i] = ( q[2] - kd*phi ) + (kd - 1)*math.pi/2
        # Update
        i += 1

    return np.array([0.1*sum(v), 0.1*sum(w)])


'''
This function implements the control law to perform a rendezvous to the origin
'''
def rendezvous_control_law(q, N_i, theta_d):

    c = 1.2 #clearence
    v = np.zeros(len(N_i))
    w = np.zeros(len(N_i))
    i = 0

    for neighbour in N_i:
        # Compute kd
        norm = math.sqrt( (q[0] - neighbour[0])**2 + (q[1] - neighbour[1])**2 )
        kd = 1 - math.exp( c - norm**2 )
        # Compute phi
        phi = math.atan2( neighbour[1]-q[1], neighbour[0]-q[0] )
        # Compute inputs
        v[i] = ( (q[0] - neighbour[0])*np.cos(q[2]) + (q[1] - neighbour[1])*np.sin(q[2]) ) * kd
        w[i] = ( q[2] - kd*phi )
        # Update
        i += 1

    ang_err = theta_d

    return np.array([0.1*sum(v), 0.1*sum(w)*1.5*ang_err])
'''
Utility function to aid in the rendezvous operation
'''
def align_cantrol_law(error):

    kp = 0.45
    proportional = kp*(error - 0.0873)
    
    return proportional
