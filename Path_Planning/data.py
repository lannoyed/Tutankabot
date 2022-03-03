import matplotlib.pyplot as plt
import numpy as np

path ='Tutankabot\Path_Planning\cmake-build-debug'
data = np.loadtxt(path + '\data.txt')

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

#plt.plot(cara)
#plt.show()

n = len(data)
x= np.zeros(n)
y = np.zeros(n)
repulsive_x = np.zeros(n)
attractive_x = np.zeros(n)
repulsive_y = np.zeros(n)
attractive_y= np.zeros(n)
direct_speed = np.zeros(n)
omega_speed = np.zeros(n)

def generateCercle( center, radius, npoint):
    points = np.zeros((npoint, 2))
    theta = 2*np.pi / npoint
    for i in range(npoint):
        points[i,0] = center[0]  + radius*np.cos(i*theta)
        points[i,1] =  center[1]  + radius*np.sin(i*theta)
    return points

def plotSample(center):
    plt.scatter(center[0], center[1], c='red') # sample
    cercle = generateCercle((center[0], center[1]), 30, 20)
    plt.scatter(cercle[:,0], cercle[:,1], c='pink', marker='+')
    cercle = generateCercle((center[0], center[1]), 15, 20)
    plt.scatter(cercle[:,0], cercle[:,1], c='pink', marker='+')
    

for i in range (n):
    x[i] = data[i,0]
    y[i] = data[i,1]
    repulsive_x[i]= data[i,2]
    repulsive_y[i]= data[i,3]
    attractive_x[i] = data[i,4]
    attractive_y[i] = data[i,5]
    direct_speed[i] = data[i,6]
    omega_speed[i] = data[i,7]
    

plt.scatter(x,y, marker = '.')
plt.scatter(50,200)
plt.scatter(-100,50)
plotSample((-60,100))
plotSample((0,120))
plotSample((0,80))
plotSample((0,145))
plt.show()

plt.plot(attractive_y, label = 'attactive_y')
plt.plot(attractive_x, label = 'attactive_x')
plt.plot(repulsive_x, label = 'repulsive_x')
plt.plot(repulsive_y, label = 'repulsive_y')
plt.legend()
plt.show()

plt.plot(direct_speed*10, label = 'V ref')
plt.plot(omega_speed, label = 'omega ref')
plt.legend()
plt.show()


"""
test 1 :
    // création des obstacles
    double hitbox = 0.0001;
    double distanceOfInfluence = 30;
    std::tuple <double, double> center = std::make_tuple<double, double>(0.0, 120);
    double k_rep = 10000;

    Sample Sample1 = Sample(center, k_rep, distanceOfInfluence, hitbox);


    // creation potential field :
    std::tuple<double, double> goal_position     =  std::make_tuple<double, double>(50, 200);
    Goal goal = Goal(goal_position, 10);
    std::tuple<double, double> position =  std::make_tuple<double, double>(-100,50);
    Potential_Field myPotential_Field = Potential_Field(position, goal);
    std::tuple<double, double> initialSpeedVector =  std::make_tuple<double, double>(1.0,0.0);
    myPotential_Field.setSpeedVerctor(initialSpeedVector);

test 2 :




"""