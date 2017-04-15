#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 12 12:11:09 2017

@author: ishana_shekhawat
"""

import numpy as np
import numdifftools as nd
import global_var

# the nonlinear dynamics function

def simple_car_Xdot(X,U):
    theta = X[2]# in radian
    u_s = U[0]
    u_phi = U[1]# in radian
    x_dot = u_s*np.cos(theta)
    y_dot = u_s*np.sin(theta)
    theta_dot = (u_s/global_var.L)*np.tan(u_phi)
    X_dot = np.array([x_dot, \
                      y_dot, \
                      theta_dot])
    return X_dot

def simple_car_Xdot1(U,X):
    return simple_car_Xdot(X,U)

def simple_car_Xnext(X,U):
    X_next = X + global_var.dT*simple_car_Xdot(X,U)
    return X_next

#def Xnext_array(X,U,L):
#    R,N = U.shape
#    X_traj = []
#    x_cur = X
#    for i in range (0,N-1):
#        u_cur = np.array([U[1,i], U[2,i]])
#        x_cur = x_cur + global_var.dT*simple_car_Xdot(X,U,L)

# this returns the jacobian wrt the state
f_jacob = nd.Jacobian(simple_car_Xdot)

# to get the jacobian wrt to the input
f_jacob1 = nd.Jacobian(simple_car_Xdot1)

# get a linearization about the point find A, B and r values depending on the operating point
# the linearization is performed at each point, assuming a slight disturbance for now. 
# the operating point is the x_cur + delta_x, u_cur+delta_u (ususally the u would stay same)

def lin_next(X,U):
    A = f_jacob(X,U)
    B = f_jacob1(U,X)
    r = -np.matmul(A,X) + np.matmul(B,U)
    delta_X = np.array([[global_var.dT*2],[global_var.dT*2],[global_var.dT*0.01]])
    delta_U = np.array([[0],[0]])
    x_next = X + global_var.dT*(np.matmul(A,delta_X) + np.matmul(B,delta_U) +r)
    return x_next