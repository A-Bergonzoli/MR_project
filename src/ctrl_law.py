#!/usr/bin/env python3

'''
This functions will compute the consensus control laws inputs for the agents
'''

import numpy as np
import math
import posture_regulation


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

    c = 1 # clearence
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

    return np.array([0.3*sum(v), 0.3*sum(w)])


# Rendez-vous control law: all agents are attracted to the same point, which is the origin (0,0) 
# w/ repulsion. [NOTE: This function is the one used by agents.py ]
def rendezvous_control_law(q, N_i, ang_err):

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

    return np.array([0.1*sum(v), 0.1*sum(w)*1.5*ang_err])
