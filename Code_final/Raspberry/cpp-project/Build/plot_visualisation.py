import numpy as np 
import matplotlib.pyplot as plt

myFile = np.genfromtxt("myPotentialField.txt")

print(myFile)
x = myFile[:,0]
y = myFile[:,1]
F = myFile[:,2] 

for i in range (len(F)) : 
    if F[i] > 1.1:
        F[i] = 1.1


x = np.reshape(x, (1001, 1000) )
y = np.reshape(y, (1001, 1000) )
F = np.reshape(F, (1001, 1000) )



fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot_surface(x, y, F, cmap='viridis', edgecolor='none')

ax.set_title('surface');

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()



import numpy as np 
myFile = np.genfromtxt("myPotentialField.txt")
myFile[:,0]

val = 0
j = 0
for i in range(len(myFile[:, 0])):
    if not val == myFile[i, 0] :
        print(i - j)
        j = i
        val = myFile[i, 0] 

