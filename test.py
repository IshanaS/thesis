#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 12 12:29:43 2017

@author: ishana_shekhawat
"""

# main 
import numpy as np 
import matplotlib.pyplot as plt
import simple_car
import global_var


# initialize the car
X = np.array([0,0,np.pi/4])# origin facing 45 degrees
X.shape = (3,1)

#U = np.array([[0.1],[np.pi/4]])

## face validation
#
## test 1 - apply a sinusoidal steering input at constant speed input
#N1 = 100
#
#t1 = np.linspace(0,10,N1)
#t1.shape = (N1,1)
#
#temp1 = t1/(np.pi/2)
#U_steer = 0.25*np.pi*np.sin(temp1)
#
#plt.figure(1)
#plt.plot(t1,U_steer)
#
#sp = 100
#U_speed = sp*np.ones((N1,1))
#U = np.concatenate((U_speed,U_steer),axis=1)
#
#N,R = U.shape
#X_traj = X.T
#x_cur = X
#for i in range (0,N-1):
#    u_cur = U[i,:]
#    u_cur.shape = (2,1)
#    x_cur = simple_car.simple_car_Xnext(x_cur,u_cur)
#    X_traj = np.append(X_traj,x_cur.T,axis=0)
#    
#plt.figure(2)
#plt.plot(X_traj[:,0],X_traj[:,1])
#
#plt.figure(3)
#plt.plot(t1,X_traj)
#
#plt.figure(4)
#plt.plot(t1,X_traj[:,2])
#
#b = np.concatenate([U_steer, np.tan(U_steer)], axis=1)
#
#plt.figure(5)
#plt.plot(sp/global_var.L*b[:,1])
#
#
## test 2 - apply a constant steering with decreasing/increasing speed
## still need to put in effort into this
#
#N2 = 100000
#
#t2 = np.linspace(0,10,N2)
#t2.shape = (N2,1)
#
#st = np.pi/3
#U_steer = st*np.ones(t2.shape)
#U_speed = np.linspace(0,500,N2)
#U_speed.shape = (N2,1)
#U = np.concatenate((U_speed,U_steer),axis=1)
#
#N,R = U.shape
#X_traj = X.T
#x_cur = X
#for i in range (0,N-1):
#    u_cur = U[i,:]
#    u_cur.shape = (2,1)
#    x_cur = simple_car.simple_car_Xnext(x_cur,u_cur)
#    X_traj = np.append(X_traj,x_cur.T,axis=0)
#
#plt.figure(6)
#plt.plot(X_traj[:,0],X_traj[:,1])
#
## validate with actual data
#
#data = np.load('/Users/ishana_shekhawat/Downloads/data.npz')
#vicon = data['vicon']
#imu = data['imu']
#steer = data['command']
#
## corresponding inputs, current state and next state data from the recording
#
#cur_state = vicon[0:len(vicon)-2,:]
#next_state = vicon[1:len(vicon)-1,:]
#steering_cmd = steer[0:len(steer)-2]
#steering_cmd.shape = (len(steer)-2,1)
#cur_steer = (steering_cmd - 1550*np.ones(steering_cmd.shape))*(np.pi/6)/300
## get a cumulative sum starting from v = (0, 0)
#cur_vx = 0 + global_var.dT*np.cumsum(imu[0:len(imu)-2,7])
#cur_vy = 0 + global_var.dT*np.cumsum(imu[0:len(imu)-2,8])
#cur_speed = np.sqrt(np.square(cur_vx) + np.square(cur_vy))
#cur_speed.shape = cur_steer.shape
#cur_U = np.concatenate((cur_speed, cur_steer),axis=1)
#
## test the inputs and current state in the model to get simulated next states - next step validation
#
#x_cur = cur_state[0,:]
#x_cur.shape = (3,1)
#X_sim = x_cur.T
#i = 0
#for i in range(0,len(cur_U)-1):
#    x_cur = cur_state[i,:].T
#    x_cur.shape = (3,1)
#    u_cur = cur_U[i,:].T
#    u_cur.shape = (2,1)
#    X_n = simple_car.simple_car_Xnext(x_cur,u_cur)
#    X_sim = np.append(X_sim,X_n.T,axis=0)
#    
#plt.figure(7)
#plt.plot(next_state[:,0],next_state[:,1], label = 'actual')
#plt.plot(X_sim[:,0],X_sim[:,1], label = 'model')
#plt.legend()
#
## test over the path- see if errors accumulate - they do. A lot. 
#
#x_cur = cur_state[i,:]
#x_cur.shape = (3,1)
#X_sim = x_cur.T
#for i in range(0,len(cur_U)-1):
#    u_cur = cur_U[i,:].T
#    u_cur.shape = (2,1)
#    x_cur = simple_car.simple_car_Xnext(x_cur,u_cur)
#    X_sim = np.append(X_sim,x_cur.T,axis=0)
#    
#plt.figure(8)
#plt.plot(next_state[:,0],next_state[:,1], label = 'actual')
#plt.plot(X_sim[:,0],X_sim[:,1], label = 'model')
#plt.legend()

#############################################################

# !pip install numdifftools: to directly install from the iPython console

# simple_car.f_jacob(x_cur,u_cur)
# simple_car.f_jacob1(u_cur,x_cur)

# run the linearized  with all 4 methods as above

# test 1 - apply a sinusoidal steering input at constant speed input
N1 = 100

t1 = np.linspace(0,10,N1)
t1.shape = (N1,1)

temp1 = t1/(np.pi/2)
U_steer = 0.25*np.pi*np.sin(temp1)

plt.figure(1)
plt.plot(t1,U_steer)

sp = 100
U_speed = sp*np.ones((N1,1))
U = np.concatenate((U_speed,U_steer),axis=1)

N,R = U.shape
X_traj = X.T
x_cur = X
for i in range (0,N-1):
    u_cur = U[i,:]
    u_cur.shape = (2,1)
    x_cur = simple_car.lin_next(x_cur,u_cur)
    X_traj = np.append(X_traj,x_cur.T,axis=0)
    
plt.figure(2)
plt.plot(X_traj[:,0],X_traj[:,1])

plt.figure(3)
plt.plot(t1,X_traj)

plt.figure(4)
plt.plot(t1,X_traj[:,2])

b = np.concatenate([U_steer, np.tan(U_steer)], axis=1)

plt.figure(5)
plt.plot(sp/global_var.L*b[:,1])

## test 2 - apply a constant steering with decreasing/increasing speed
## still need to put in effort into this
#
#N2 = 100000
#
#t2 = np.linspace(0,10,N2)
#t2.shape = (N2,1)
#
#st = np.pi/3
#U_steer = st*np.ones(t2.shape)
#U_speed = np.linspace(0,500,N2)
#U_speed.shape = (N2,1)
#U = np.concatenate((U_speed,U_steer),axis=1)
#
#N,R = U.shape
#X_traj = X.T
#x_cur = X
#for i in range (0,N-1):
#    u_cur = U[i,:]
#    u_cur.shape = (2,1)
#    x_cur = simple_car.lin_next(x_cur,u_cur)
#    X_traj = np.append(X_traj,x_cur.T,axis=0)
#
#plt.figure(6)
#plt.plot(X_traj[:,0],X_traj[:,1])

# validate with actual data

data = np.load('/Users/ishana_shekhawat/Downloads/data.npz')
vicon = data['vicon']
imu = data['imu']
steer = data['command']

# corresponding inputs, current state and next state data from the recording

cur_state = vicon[0:len(vicon)-2,:]
next_state = vicon[1:len(vicon)-1,:]
steering_cmd = steer[0:len(steer)-2]
steering_cmd.shape = (len(steer)-2,1)
cur_steer = (steering_cmd - 1550*np.ones(steering_cmd.shape))*(np.pi/6)/300
# get a cumulative sum starting from v = (0, 0)
cur_vx = 0 + global_var.dT*np.cumsum(imu[0:len(imu)-2,7])
cur_vy = 0 + global_var.dT*np.cumsum(imu[0:len(imu)-2,8])
cur_speed = np.sqrt(np.square(cur_vx) + np.square(cur_vy))
cur_speed.shape = cur_steer.shape
cur_U = np.concatenate((cur_speed, cur_steer),axis=1)

# test the inputs and current state in the model to get simulated next states - next step validation

x_cur = cur_state[0,:]
x_cur.shape = (3,1)
X_sim = x_cur.T
i = 0
for i in range(0,len(cur_U)-1):
    x_cur = cur_state[i,:].T
    x_cur.shape = (3,1)
    u_cur = cur_U[i,:].T
    u_cur.shape = (2,1)
    X_n = simple_car.lin_next(x_cur,u_cur)
    X_sim = np.append(X_sim,X_n.T,axis=0)
    
plt.figure(7)
plt.plot(next_state[:,0],next_state[:,1], label = 'actual')
plt.plot(X_sim[:,0],X_sim[:,1], label = 'model')
plt.legend()

# test over the path- see if errors accumulate - they do. A lot. 

x_cur = cur_state[i,:]
x_cur.shape = (3,1)
X_sim = x_cur.T
for i in range(0,len(cur_U)-1):
    u_cur = cur_U[i,:].T
    u_cur.shape = (2,1)
    x_cur = simple_car.lin_next(x_cur,u_cur)
    X_sim = np.append(X_sim,x_cur.T,axis=0)
    
plt.figure(8)
plt.plot(next_state[:,0],next_state[:,1], label = 'actual')
plt.plot(X_sim[:,0],X_sim[:,1], label = 'model')
plt.legend()

