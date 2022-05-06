# include <stdio.h>
# include <stdlib.h>
# include <math.h>
# include "MainController.h"

// Time of game
#define time_return 80.0
#define time_stop 90.0
#define periode_lidar 0.2


// Robot datas 

// Our robot is represented as a point in the middle of a circle of radius radius_robot, which is here 20 [cm].
// ça va représenter logiquement la vraie taille de notre robot (il fait plutôt 17 [cm], 20 est une upper bound).
#define radius_robot        0.2             
#define PI                  3.14159265

                                            // sur place, ou alors on la joue safe et on met la même dimension "max" pour tous les robots.
#define sampleRadius        0.075           // Rayon d'un sample.
#define precision           0.075           // Permet de savoir si on a atteint un goal.

// To know if we are stucks
#define minimumPositionStep 0.00001
#define minimumAngleStep    0.1

#define coef 1.0
//RECTANGLES
#define rho0_rectangle 0.8
#define krep_rectangle 1e-2*coef
//BORDERS
#define rho0_border 0.8
#define krep_border 1e-2*coef
//LA TIGE: on la met la plus faible possible.
#define rho0_tige 0.3
#define krep_tige 1e-2*coef
//SAMPLES
#define rho0_sample 0.8
#define krep_sample 1e-2*coef


//MAP
#define rho0_map    0.6
#define krep_map    1e-1*coef*0.19


//OPPONENT
// Tradeoff possible : lorsque le krep sera beaucoup plus petit, on devra peut-être augmenter le rho0. 
#define rho0_opponent   0.8
#define krep_opponent   0.02*coef
#define hitbox_opponent 0.05
#define numborOfPoints  20  
#define radiusModelisationOpponent 0.10 
//HITBOX GÉNÉRALE DES OBSTACLES : modifiable. TAG.
#define hitbox_obstacle 0.0

#define WEIGHT_GOAL 1.5

// LIMITEUR DE FORCE
#define LIMIT_ATTRACTIVE_FORCE 1.0
#define LIMIT_REPULSIVE_FORCE  1.1


// speed max define 
#define global_vMax 0.12
#define global_wMax 1.5


// controle du type de partie 
#define team_number 0 // 0 PURPLE or 1 YELLOW

#define MINMUM_DISTANCE 0.01
#define TEST_POTENTIAL 1

// hardcoded ennemy 
#define NON_LIDAR_DETECTION 0
#define HardOppX 0.79 
#define HardOppY 2.1


#define VISUALISATION_TEST 0

#define DONT_MOVE false // J'ai changé un peu le code pour qu'il se calibre quand meme mais qu'il ne bouge plus après (Le plus bg des codeurs) 
