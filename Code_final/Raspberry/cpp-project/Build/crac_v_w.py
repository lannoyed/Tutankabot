import matplotlib.pyplot as plt
import numpy as np

def limit_vref_omega(omega2):
    omega1 = 14;
    Vref = 0.06/2 * (omega1 + omega2)
    Omega = 0.06/(2*0.180) * (omega1 - omega2)
    return Vref, Omega

n=1000
omega2 = np.linspace(-14, 14, n)
cara = np.zeros((n,2))
for i in range(n):
    cara[i] = limit_vref_omega(omega2[i])


plt.plot(cara)
plt.show()