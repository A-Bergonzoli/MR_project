#!/usr/bin/env python3


import numpy as np
import math
import sympy as sp


# constant gains
k_x = 1
k_y = 1
k_th = 10

k_1 = 0.5
k_2 = 0.5


'''
This function computes the artificial potential field (APF) for a given obstacle.
This function is used by the leader (only one that knows the position of the obstacles)
'''
def get_apf(x, y, x_obs, y_obs, r, R):
    inside_apf = False
    x_a, y_a = sp.symbols("x_a, y_a")

    # compute distance robot-obstacle
    l = sp.sqrt( (x_a-x_obs)**2 + (y_a-y_obs)**2 )  # symbolic
    l_real = math.sqrt( (x-x_obs)**2 + (y-y_obs)**2 )  # numeric
    # compute apf
    if l_real < r:
        B_a = 0
        print('B_a not defined')
    elif r <= l_real and l_real < R:
        B_a = sp.exp( (l-r) / (l-R) )
        inside_apf = True
    elif l_real >= R:
        B_a = 0
    V_a = B_a/(1 - B_a)
    return np.array([V_a, inside_apf])
'''
This function computes the modified tracking error (as a symbolic expression) to accomodate obstacle avoidance.
As above, this function is used only by the leader.
'''
def get_modified_position_error(x, y, x_d, y_d, V_a):
    x_a, y_a = sp.symbols("x_a, y_a")
    # compute the gradient of the apf V_a
    d_dx = sp.diff(V_a, x_a)
    d_dy = sp.diff(V_a, y_a)
    # get error
    e_x = x_d - x
    e_y = y_d - y
    # modified error
    E_x = e_x - d_dx
    E_y = e_y - d_dy
    return np.array([E_x, E_y])
'''
This function computes the linear control input for the obstacle avoidance phase.
As above, this function is used only by the leader.
'''
def obstacle_control_law(E_x, E_y, th, xdot_d, ydot_d):
    kp = 0.3
    h2 = kp*E_x + xdot_d
    h3 = kp*E_y + ydot_d
    v = h2*np.cos(th) + h3*np.sin(th)
    return v




'''
This function, based on the communication graph G, implements the control law to compute the control input u1_j of the j-th agent.
Only follower1 & follower4 know the specified reference trajectory.
'''
def consensus_control_law1(q0, w0, qj, Nj, A, j):
    alpha = 30    # non negative constant
    beta = 0.75   # positive constant (beta > k)

    z1i = np.zeros(len(Nj))
    var_appoggio1 = 0
    i = 0

    if j == 1 or j == 4: bj = 2
    else: bj = 0

    # compute the needed transformations:
    # virtual leader 
    z10 = q0[2]
    u10 = w0
    # follower j
    z1j = qj[2]
    
    # neighbours of follower j
    for neighbour in Nj:
        z1i[i] = neighbour[2]
        var_appoggio1 += A[j-1][i]*(z1j - z1i[i])      
        
        i += 1 # update index

    # control input
    u1j = u10 - alpha*var_appoggio1 - alpha*bj*(z1j - z10) - beta*np.sign( var_appoggio1 + bj*(z1j - z10) )

    return u1j

'''
As before this function, based on the communication graph G, implements the control law to compute the control input u2_j of the j-th agent.
It also allows us to obtain the actual control inputs (v_j, w_j) using the inverse tranformation.
Only follower1 & follower4 know the specified reference trajectory.
'''
def consensus_control_law2(q0, w0, qj, pjx, pjy, Nj, pix, piy, A, j, u1j, u1i):
    alpha = 30     # non negative constant
    beta = 0.75    # positive constant (beta > k)
    k0 = 0.2       # positive constant

    z2i = np.zeros(len(Nj))
    z3i = np.zeros(len(Nj))
    var_appoggio2 = 0
    i = 0

    if j == 1 or j == 4: bj = 2
    else: bj = 0

    # compute the needed transformations:
    # follower j
    z3j = (qj[0]-pjx)*np.sin(qj[2]) - (qj[1]-pjy)*np.cos(qj[2])
    z2j = (qj[0]-pjx)*np.cos(qj[2]) + (qj[1]-pjy)*np.sin(qj[2]) + k0*np.sign(u1j)*z3j
    # virtual leader
    z30 = (q0[0]-0)*np.sin(q0[2]) - (q0[1]-0)*np.cos(q0[2])
    z20 = (q0[0]-0)*np.cos(q0[2]) + (q0[1]-0)*np.sin(q0[2]) + k0*np.sign(w0)*z30

    # neighbours of follower j
    for neighbour in Nj:
        z3i[i] = (neighbour[0]-pix[i])*np.sin(neighbour[2]) - (neighbour[1]-piy[i])*np.cos(neighbour[2])
        z2i[i] = (neighbour[0]-pix[i])*np.cos(neighbour[2]) + (neighbour[1]-piy[i])*np.sin(neighbour[2]) + k0*np.sign(u1i[i])*z3i[i]
        var_appoggio2 += A[j-1][i]*(z2j - z2i[i])
        
        i += 1 # update index

    # control input
    u2j = - alpha*var_appoggio2 - alpha*bj*(z2j - z20) - beta*np.sign( var_appoggio2 + bj*(z2j - z20) ) - k0*abs(u1j)*z2j

    # actual control inputs
    vj = u2j + (1 + k0**2)*u1j*z3j
    wj = u1j

    return np.array([vj, wj])
