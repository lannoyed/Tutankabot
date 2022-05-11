from cProfile import label
from sys import implementation
import numpy as np 
import matplotlib.pyplot as plt

import time

val = np.array([0,0,-1,-1,0,1,2,2,1,0,-1,-2,-3,-4,-5,-6])
a = np.array([0,0,0,0,0,1,1,1,1,0,0,1,1,0,0,1]) 
b = np.array([0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1])

multiply = 100

a_mul = np.zeros(len(a)*multiply)
b_mul = np.zeros(len(a)*multiply)
val_mul = np.zeros(len(a)*multiply)
for i in range (len(a)):
    a_mul[i*multiply : (i+1) * multiply] = a[i]
    b_mul[i*multiply : (i+1) * multiply] = b[i]
    val_mul[i*multiply : (i+1) * multiply] = val[i]
    
fig1 = plt.figure()   
ax1 = fig1.add_subplot(311)
ax2 = fig1.add_subplot(312)
ax3 = fig1.add_subplot(313)

ax1.plot(a_mul, label= 'Signal A')
ax2.plot(b_mul, label= 'Signal B')
ax3.plot(val_mul,label=  'Counter value')
ax1.legend(loc ="lower left")
ax2.legend(loc ="lower left")
ax3.legend(loc ="lower left")
plt.subplots_adjust(left=0.1,bottom=0.07,right=0.9,top=0.95, wspace=0.4, hspace=0.4)


plt.show()