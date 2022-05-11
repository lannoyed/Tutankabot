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



