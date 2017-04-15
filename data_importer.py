import pickle
import numpy as np

data = pickle.load(open("data.pickle", "rb"))
#print(len(data))
#print(data[0]["imu_data"].linear_acceleration.x)
#print(data[1]["lidar_front"].ranges[10])

command = np.zeros([len(data)])

imu = np.zeros([len(data),10]) 
#orientation: x y z w, angular_v: x y z, linear_acc: x y z

lidar_f = np.zeros([len(data),len(data[0]['lidar_front'].ranges)])
lidar_r = np.zeros([len(data),len(data[0]['lidar_rear'].ranges)])
vicon = np.zeros([len(data),3]) #x,y,theta

for i in range(len(data)):
    command[i] = data[i]['command'].steering
    imu[i,0] = data[i]['imu_data'].orientation.x
    imu[i,1] = data[i]['imu_data'].orientation.y
    imu[i,2] = data[i]['imu_data'].orientation.z
    imu[i,3] = data[i]['imu_data'].orientation.w
    imu[i,4] = data[i]['imu_data'].angular_velocity.x
    imu[i,5] = data[i]['imu_data'].angular_velocity.y
    imu[i,6] = data[i]['imu_data'].angular_velocity.z
    imu[i,7] = data[i]['imu_data'].linear_acceleration.x
    imu[i,8] = data[i]['imu_data'].linear_acceleration.y
    imu[i,9] = data[i]['imu_data'].linear_acceleration.z
    lidar_f[i,:] = np.nan_to_num(data[i]['lidar_front'].ranges)
    lidar_r[i,:] = np.nan_to_num(data[i]['lidar_rear'].ranges)
    vicon[i,0] = data[i]['vicon_data'].positions[0].x
    vicon[i,1] = data[i]['vicon_data'].positions[0].y
    vicon[i,2] = data[i]['vicon_data'].positions[0].theta
    
    
lidar_f[lidar_f>100] = 100 #replace inf with 100 (NOTICE: MIGHT BE BAD IDEA)
lidar_r[lidar_r>100] = 100


np.savez('data',command=command,imu=imu,lidar_f=lidar_f,lidar_r=lidar_r,vicon=vicon)

#%%
import matplotlib.pyplot as plt
from matplotlib import animation


fig, ax = plt.subplots()
line, = ax.plot(vicon[:,0], vicon[:,1], color='k')
fig.set_size_inches(18,12)

def update(num, x, y, line):
    line.set_data(x[:num], y[:num])
    return line,

ani = animation.FuncAnimation(fig, update, len(data), fargs=[vicon[:,0], vicon[:,1], line],
                              interval=25, blit=True)
#ani.save('trajectory.gif',writer='imagemagick')
plt.show()