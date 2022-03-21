import matplotlib.pyplot as plt
import numpy as np


data = np.loadtxt('data.txt')

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
    cercle = generateCercle((center[0], center[1]), 30, 20) #Zone d'influence.
    plt.scatter(cercle[:,0], cercle[:,1], c='pink', marker='+')
    cercle = generateCercle((center[0], center[1]), 20, 20) #Ici, hitbox.
    plt.scatter(cercle[:,0], cercle[:,1], c='pink', marker='+')
    

for i in range (n):
    x[i] = 100 * data[i,0]
    y[i] = 100 * data[i,1]
    repulsive_x[i]= data[i,2]
    repulsive_y[i]= data[i,3]
    attractive_x[i] = data[i,4]
    attractive_y[i] = data[i,5]
    direct_speed[i] = data[i,6]
    omega_speed[i] = data[i,7]
    
# AJOUTER LA CARTE DANS CE PÂTÉ
plt.plot(x,y)
plt.scatter(150,225)
plt.scatter(25,210)
plt.title('Displacement of the robot in the map')
plt.xlabel('[cm]')
plt.ylabel('[cm]')
plotSample((75,225))
#plotSample((0,120))
#plotSample((0,80))
plotSample((100,250))

# Les borders
x = [0, 149, 200, 200, 149, 0, 0]
y = [0, 0, 51, 249, 300, 300, 0]
plt.plot(x, y, linestyle='-', linewidth=1.5, color='black',label="Borders")

x = [5, 145.46, 195, 195, 145.46, 5, 5]
y = [5, 5, 54.54, 245.46, 295, 295, 5]
plt.plot(x, y, linestyle='--', linewidth=1, color='red',label="Hitbox of obstacles")


# Ajout des rectangles : en haut à droite.
x = [117.5, 117.5, 132.5, 132.5]
y = [300, 289.8, 289.8, 300]
plt.plot(x, y, linestyle='-', linewidth=1.5, color='blue')
x = [112.5, 112.5, 137.5, 137.5]
y = [300, 284.8, 284.8, 300]
plt.plot(x, y, linestyle='--', linewidth=1, color='red')

# En bas à droite.
x = [117.5, 117.5, 132.5, 132.5]
y = [0, 10.2, 10.2, 0]
plt.plot(x, y, linestyle='-', linewidth=1.5, color='blue')
x = [112.5, 112.5, 137.5, 137.5]
y = [0, 15.2, 15.2, 0]
plt.plot(x, y, linestyle='--', linewidth=1, color='red')

# Galerie d'art haute
x = [0, 8.5, 8.5, 0]
y = [183, 183, 255, 255]
plt.plot(x, y, linestyle='-', linewidth=1.5, color='blue')
x = [0, 13.5, 13.5, 0]
y = [178, 178, 260, 260]
plt.plot(x, y, linestyle='--', linewidth=1, color='red')

# Galerie d'art basse
x = [0, 8.5, 8.5, 0]
y = [117, 117, 45, 45]
plt.plot(x, y, linestyle='-', linewidth=1.5, color='blue')
x = [0, 13.5, 13.5, 0]
y = [122, 122, 40, 40]
plt.plot(x, y, linestyle='--', linewidth=1, color='red')

# Petit rectangle bas gauche
x = [0, 10.2, 10.2, 0]
y = [127.5, 127.5, 145, 145]
plt.plot(x, y, linestyle='-', linewidth=1.5, color='blue')
x = [0, 15.2, 15.2, 0]
y = [122.5, 122.5, 150, 150]
plt.plot(x, y, linestyle='--', linewidth=1, color='red')

# Petit rectangle haut gauche
x = [0, 10.2, 10.2, 0]
y = [172.5, 172.5, 157.5, 157.5]
plt.plot(x, y, linestyle='-', linewidth=1.5, color='blue')
x = [0, 15.2, 15.2, 0]
y = [177.5, 177.5, 152.5, 152.5]
plt.plot(x, y, linestyle='--', linewidth=1, color='red')

# La tige
x = [0, 60]
y = [150,150]
plt.plot(x, y, linestyle='-', linewidth=1.5, color='blue', label="Rectangles")
x = [0, 65, 65, 0]
y = [155, 155, 145, 145]
plt.plot(x, y, linestyle='--', linewidth=1, color='red')


plt.legend()
plt.show()


##################################################
"""plt.plot(attractive_y, label = 'attactive_y')
plt.plot(attractive_x, label = 'attactive_x')
plt.plot(repulsive_x, label = 'repulsive_x')
plt.plot(repulsive_y, label = 'repulsive_y')
plt.legend()
plt.show()

plt.title('Speed adaptation of the robot')
plt.plot(direct_speed*10, label = 'V_ref ([m/s])')
plt.plot(omega_speed, label = 'omega_ref ([rad/s])')
plt.xlabel('Number of iterations')
plt.ylabel('Amplitude')
plt.legend()
plt.show()"""
##################################################

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