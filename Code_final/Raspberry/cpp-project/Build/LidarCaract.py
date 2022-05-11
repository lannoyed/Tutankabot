from cProfile import label
from re import X
from turtle import color
import matplotlib.pyplot as plt
import numpy as np

opponent = np.genfromtxt('LIDAR_795_2100.txt', skip_footer=1, skip_header=1)

len_file = len(opponent)
opponent_x = []
opponent_y = []

for i in range (len_file):
    if np.abs(opponent[i,0]) < 4 and np.abs(opponent[i,1]) < 4:
        opponent_x.append(opponent[i,0] )
        opponent_y.append(opponent[i,1] )
        

x = [0.795] 
y = [2.100]
plt.scatter(opponent_x, opponent_y, label='measured opponent 1', color= 'blue')
plt.scatter(x,y,label = 'real opponent 1', color='blue', marker='+')

opponent = np.genfromtxt('LIDAR_1270_2130.txt', skip_footer=1, skip_header=1)

len_file = len(opponent)
opponent_x = []
opponent_y = []

for i in range (len_file):
    if np.abs(opponent[i,0]) < 4 and np.abs(opponent[i,1]) < 4:
        opponent_x.append(opponent[i,0] )
        opponent_y.append(opponent[i,1] )
        
y = [2.130]
x = [1.270]

plt.scatter(opponent_x, opponent_y, label='measured opponent 2', color= 'green')
plt.scatter(x,y,label = 'real opponent 2', color='green', marker='+')

opponent = np.genfromtxt('LIDAR_1204_2894.txt', skip_footer=1, skip_header=1)

len_file = len(opponent)
opponent_x = []
opponent_y = []

for i in range (len_file):
    if np.abs(opponent[i,0]) < 4 and np.abs(opponent[i,1]) < 4:
        opponent_x.append(opponent[i,0] )
        opponent_y.append(opponent[i,1] )
        
y = [2.894]
x = [1.200]

plt.scatter(opponent_x, opponent_y, label='measured opponent 3', color= 'black')
plt.scatter(x,y,label = 'real opponent 3', color='black', marker='+')



opponent = np.genfromtxt('LIDAR_0400_2335.txt', skip_footer=1, skip_header=1)

len_file = len(opponent)
opponent_x = []
opponent_y = []

for i in range (len_file):
    if np.abs(opponent[i,0]) < 4 and np.abs(opponent[i,1]) < 4:
        opponent_x.append(opponent[i,0] )
        opponent_y.append(opponent[i,1] )
        
y = [2.335]  
x = [0.4]

plt.scatter(opponent_x, opponent_y, label='measured opponent 4', color= 'orange')
plt.scatter(x,y,label = 'real opponent 4', color='orange', marker='+')


opponent = np.genfromtxt('LIDAR_095_2775.txt', skip_footer=1, skip_header=1)

len_file = len(opponent)
opponent_x = []
opponent_y = []

for i in range (len_file):
    if np.abs(opponent[i,0]) < 4 and np.abs(opponent[i,1]) < 4:
        opponent_x.append(opponent[i,0] )
        opponent_y.append(opponent[i,1] )
        
y = [3.0-0.255]
x = [0.21]

plt.scatter(opponent_x, opponent_y, label='measured opponent 5', color= 'purple')
plt.scatter(x,y,label = 'real opponent 5', color='purple', marker='+')





plt.scatter(0.67, 2.79, label='robot position')




plt.legend()
plt.xlabel('X [cm]')
plt.ylabel('Y [cm]')
plt.axis('equal')
plt.show()