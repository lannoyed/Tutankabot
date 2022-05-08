from cProfile import label
from turtle import color
import matplotlib.pyplot as plt
import numpy as np

PLOT_LIDAR_OPP = False

data = np.genfromtxt('data_log.txt',skip_footer=1, skip_header=1)
tracking = np.genfromtxt('data_State_log.txt',skip_footer=1, skip_header=1)
opponent = np.genfromtxt('myThread.txt', skip_footer=1, skip_header=1)

distanceOpp = []
calibration_point_x = []
stuck_point_x= []
action_point_x = []
return_point_x = []
calibration_point_y = []
stuck_point_y= []
action_point_y = []
return_point_y = []
atbase_x = []
atbase_y =[]
for i in range( len( tracking) ) :
    if tracking[i][0] == 0 :
        calibration_point_x.append(100* tracking[i][1])
        calibration_point_y.append(100* tracking[i][2])
    if tracking[i][0] == 2 :
        stuck_point_x.append(100*tracking[i][1])
        stuck_point_y.append(100*tracking[i][2])
    if tracking[i][0]== 3 :
        action_point_x.append(100*tracking[i][1])
        action_point_y.append(100*tracking[i][2])
    if tracking[i][0]== 4 :
        return_point_x.append(100*tracking[i][1])
        return_point_y.append(100*tracking[i][2])
    if tracking[i][0] == 5:
        atbase_x.append(100*tracking[i][1])
        atbase_y.append(100*tracking[i][2])

def limit_vref_omega(omega2):
    omega1 =  14.82
    Vref = 0.03/2 * (omega1 + omega2)
    Omega = 0.03/(2*0.090) * (omega1 - omega2)
    return Vref, Omega

n=1000
omega2 = np.linspace(-14.82, 14.82, n)
cara = np.zeros((n,2))
for i in range(n):
    cara[i] = limit_vref_omega(omega2[i])

#Première roue fixée à vitesse max et on fait varier l'autre et on observe l'évolution.
plt.ylabel("Amplitude [m/s ou rad/s]")
plt.xlabel("Vitesse de rotation de la seconde roue. Orange : angulaire et bleue : linéique.")
plt.plot(omega2, cara)
plt.show()

n = len(data)
x= np.zeros(n)
y = np.zeros(n)
repulsive_x = np.zeros(n)
attractive_x = np.zeros(n)
repulsive_y = np.zeros(n)
attractive_y= np.zeros(n)
direct_speed = np.zeros(n)
omega_speed = np.zeros(n)
distanceOpp = np.zeros(n) 

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
    
def plotOpponent(center, radius):
    plt.scatter(center[0], center[1], c='red', label='center of the opponent') # sample
    cercle = generateCercle((center[0], center[1]), radius, 30) #Zone d'influence.
    plt.scatter(cercle[:,0], cercle[:,1], c='pink', marker='+',label= 'hitbox of the opponent')

    
k = 0 
for i in range (n):
    x[i] = 100 * data[i,0]
    y[i] = 100 * data[i,1]
    repulsive_x[i]= data[i,2]
    repulsive_y[i]= data[i,3]
    attractive_x[i] = data[i,4]
    attractive_y[i] = data[i,5]
    direct_speed[i] = data[i,6]
    omega_speed[i] = data[i,7]
    if data[i,10] < 4 : 
        distanceOpp[k] = data[i,10]
        k += 1 
    
len_file = len(opponent)
opponent_x = []
opponent_y = []

for i in range (len_file):
    if np.abs(opponent[i,0]) < 4 and np.abs(opponent[i,1]) < 4:
        print(i)
        opponent_x.append(opponent[i,0] * 100)
        opponent_y.append(opponent[i,1] * 100)

if PLOT_LIDAR_OPP :         
    plt.scatter(opponent_x, opponent_y, color='green', label='Opponent', marker='*')
    
opponent_x_final = []
opponent_y_final = []

param = 10
longueur = len(opponent_x) // param
for i in range(longueur):
    donnee_x = 0
    donnee_y = 0
    for j in range(param):
        donnee_x += opponent_x[param*i + j]
        donnee_y += opponent_y[param*i + j]
    opponent_x_final.append(donnee_x / param)
    opponent_y_final.append(donnee_y / param)
        
if PLOT_LIDAR_OPP :  
    plt.scatter(opponent_x_final, opponent_y_final, color='red', label='Opponent', marker='*')

    
# AJOUTER LA CARTE DANS CE PÂTÉ
plt.plot(x,y, label = "Displacement of the robot in the map", color = 'purple')
#plt.scatter(150,225)
#plt.scatter(25,210)
plt.xlabel('x-axis [cm]')
plt.ylabel('y-axis [cm]')


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
x = [0, 30]
y = [150,150]
plt.plot(x, y, linestyle='-', linewidth=1.5, color='blue', label="Rectangles")
x = [0, 35, 35, 0]
y = [155, 155, 145, 145]
plt.plot(x, y, linestyle='--', linewidth=1, color='red')

if not PLOT_LIDAR_OPP : 
    plotOpponent((79.5, 210), 22)

# Tracking
plt.scatter(stuck_point_x, stuck_point_y, marker = '*', c = 'black')
#plt.scatter(79.5, 90.0, marker = '*', c= 'red')
#plt.scatter(calibration_point_x, calibration_point_y, marker='.', c='pink')
plt.scatter(action_point_x, action_point_y, marker = '+', c ='green')
plt.scatter(return_point_x, return_point_y, marker = '.', c ='red')



#plt.plot([180.1786,25.0000],[154.9165, 170.0000], c = 'green')
#plt.plot([180.1786,135.3083 ],[154.9165,19.30], c = 'red')

plt.axis('equal')
plt.legend()
plt.show()

# Une iteration = 1 ms
nombreIter = len(attractive_x)
timeStop = nombreIter * 10**(-3)
timeScale = np.linspace(0, timeStop, nombreIter)
##################################################
plt.plot(timeScale,attractive_y, label = 'attractive_y')
plt.plot(timeScale, attractive_x, label = 'attractive_x')
plt.plot(timeScale, repulsive_x, label = 'repulsive_x')
plt.plot(timeScale, repulsive_y, label = 'repulsive_y')

plt.xlabel("Time [s]")
plt.ylabel("Force [N]")
plt.legend()
plt.show()

plt.plot(timeScale, distanceOpp, label='distance opponent')
plt.ylabel('[m]')
plt.xlabel('time')
plt.show()

plt.title('Speed adaptation of the robot')
plt.plot(timeScale, direct_speed*10, label = 'V_ref ([m/s])')
plt.plot(timeScale, omega_speed, label = 'omega_ref ([rad/s])')
plt.xlabel('Time [s]')
plt.ylabel('Amplitude')
plt.legend()
plt.show()
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