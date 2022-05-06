import numpy as np 
import matplotlib.pyplot as plt

myFile = np.genfromtxt("myPotentialField.txt")

print(myFile)
x = myFile[:,0]
y = myFile[:,1]
F = myFile[:,2]

x = np.reshape(x, (100, 100) )
y = np.reshape(y, (100, 100) )
F = np.reshape(F, (100, 100) )



fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot_surface(x, y, F, cmap='viridis', edgecolor='none')

ax.set_title('surface');

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()



