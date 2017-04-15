#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 12 12:11:09 2017

@author: ishana_shekhawat
"""

import numpy as np
import global_var

# the nonlinear dynamics function

def simple_car_Xdot(X,U,L):
    theta = X[2]# in radian
    u_s = U[0]
    u_phi = U[1]# in radian
    x_dot = u_s*np.cos(theta)
    y_dot = u_s*np.sin(theta)
    theta_dot = (u_s/L)*np.tan(u_phi)
    X_dot = np.array([x_dot, \
                      y_dot, \
                      theta_dot])
    return X_dot

def simple_car_Xnext(X,U,L):
    X_next = X + global_var.dT*simple_car_Xdot(X,U,L)
    return X_next

#def Xnext_array(X,U,L):
#    R,N = U.shape
#    X_traj = []
#    x_cur = X
#    for i in range (0,N-1):
#        u_cur = np.array([U[1,i], U[2,i]])
#        x_cur = x_cur + global_var.dT*simple_car_Xdot(X,U,L)
