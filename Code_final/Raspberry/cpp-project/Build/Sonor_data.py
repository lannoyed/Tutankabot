from cProfile import label
import numpy as np
import matplotlib.pyplot as plt

data_sonar = np.genfromtxt('SonarLogStatic.txt',skip_footer=1, skip_header=1)

sonar1 = data_sonar[:,0]
sonar2 = data_sonar[:,1]
sonar3 = data_sonar[:,2]
sonar4 = data_sonar[:,3]
sonar5 = data_sonar[:,4]

#plt.plot(sonar3, label='sonar3')
#plt.plot(sonar1, label='sonar1')


real_data = np.zeros(7000, dtype=np.float64)

real_data[0:3254] = 0.1
real_data[3254:5635] = 0.25
real_data[5635:] = 0.35

t = np.arange(len(real_data), dtype=np.float64) * 0.028

plt.plot(t, np.abs(sonar2[0:7000]-real_data) * 10, label = "error x 10")

plt.plot(t, sonar2[0:7000], label='measured value')

plt.plot(t, real_data, label = "real distance")

plt.xlabel('time [s]')
plt.ylabel('distance [m]')

#plt.


#plt.plot(sonar4, label='sonar droite')
#plt.plot(sonar5, label='sonar5')

plt.legend()
plt.show()




data_sonar = np.genfromtxt('SonarLogDynamic15cm.txt',skip_footer=1, skip_header=1)
sonar2 = data_sonar[:,1]

real_data = np.zeros(len(data_sonar), dtype=np.float64)

real_data[:] = 0.16

t = np.arange(len(real_data), dtype=np.float64) * 0.028

plt.plot(t, np.abs(sonar2-real_data) * 10, label = "error x 10")

plt.plot(t, sonar2, label='measured value')

plt.plot(t, real_data, label = "real distance")

plt.xlabel('time [s]')
plt.ylabel('distance [m]')

plt.legend()
plt.show()

data_sonar = np.genfromtxt('SonarLogDynamic46cm.txt',skip_footer=1, skip_header=1)
sonar2 = data_sonar[:,1]

real_data = np.zeros(len(data_sonar), dtype=np.float64)

real_data[:] = 0.46

t = np.arange(len(real_data), dtype=np.float64) * 0.028

plt.plot(t, np.abs(sonar2-real_data) * 10, label = "error x 10")

plt.plot(t, sonar2, label='measured value')

plt.plot(t, real_data, label = "real distance")

plt.xlabel('time [s]')
plt.ylabel('distance [m]')

plt.legend()
plt.show()


##############################################################
#              Impact de l'angle d'observation               #
##############################################################

list_angle =[0, 30, 45, 60]
list_distance = []

for angle in list_angle:
    data_sonar = np.genfromtxt('Sonar_'+str(angle)+'deg_40cm.txt',skip_footer=1, skip_header=1)
    sonar2 = data_sonar[:,1]
    val = sonar2.mean()
    list_distance.append(val)

plt.plot(list_angle, list_distance, color='blue')
plt.scatter(list_angle, list_distance, color='blue', label='measured distance for a given angle')

plt.plot([0,60], [0.4,0.4], label='real distance', color='orange')


plt.xlabel('angle de mesure [°]')
plt.ylabel('distance mesurée [m]')

plt.legend()
plt.show()




