from cProfile import label
import matplotlib.pyplot as plt
import numpy as np

fichier = np.genfromtxt("main_log2.txt", skip_footer=1, skip_header=1)

odo1 = fichier[:,0]
odo2 = fichier[:,1]

x = fichier[:,2]
y = fichier[:,3]

plt.plot(odo1[40:], label= "odometer wheel 1")
plt.plot(odo2[40:], label= "odometer wheel 2")
plt.title("Plot oder odometer measurements")
plt.xlabel("iteration")
plt.ylabel("number tics")
plt.legend()
plt.show()

plt.scatter(x,y)
plt.title("robot trajectory measured")
plt.show()






