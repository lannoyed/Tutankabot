# include <stdio.h>
# include <stdlib.h>
# include <math.h>
#include "MainController.h"


#ifdef SIMU_PROJECT

// Time of game
#define time_return 80.0
#define time_stop 90.0
#define periode_lidar 0.2


// Robot datas 
#define toCenterOfWheels    0.03445
#define radius_robot        0.05  // Our robot is represented as a point in the middle of a circle of radius radius_robot, which is here 15 [cm].

// Every obstacle in the map has the same distance of influence, as a first generalisation.
#define radiusOpponent      0.11  // L'opponent est modélisé par un cercle de rayon 15 [cm].
#define sampleRadius        0.075 // Rayon d'un sample.
#define precision           0.08

// To know if we are stucks
#define minimumPositionStep 0.00001
#define minimumAngleStep    0.1

// poids define
#define rho0_rectangle 0.1
#define krep_rectangle 8e-4 
#define krep_border 8e-4
#define rho0_border 0.1
#define rho0_tige 0.1
#define krep_tige 8e-4

// speed max define 
#define global_vMax 0.84      
#define global_wMax 4.6666

#else 

// Time of game
#define time_return 80.0
#define time_stop 90.0
#define periode_lidar 0.2


// Team number
#define team_number 0 // or 2 

// Robot datas 

// Our robot is represented as a point in the middle of a circle of radius radius_robot, which is here 20 [cm].
// ça va représenter logiquement la vraie taille de notre robot (il fait plutôt 17 [cm], 20 est une upper bound).
#define radius_robot        0.2             
#define PI                  3.14159265

#define radiusOpponent      0.15            // L'opponent est modélisé par un cercle de rayon 15 [cm], tout comme nous. Ce paramètre peut être ajustable à l'estimation lorsqu'on verra les robots opponents
                                            // sur place, ou alors on la joue safe et on met la même dimension "max" pour tous les robots.
#define sampleRadius        0.075           // Rayon d'un sample.
#define precision           0.15

// To know if we are stucks
#define minimumPositionStep 0.00001
#define minimumAngleStep    0.1


//RECTANGLES
#define rho0_rectangle 0.05
#define krep_rectangle 8e-7 
//BORDERS
#define rho0_border 0.05
#define krep_border 8e-7
//LA TIGE
#define rho0_tige 0.05
#define krep_tige 8e-7
//SAMPLES
#define rho0_sample 0.1
#define krep_sample 8e-4
//OPPONENT
// Tradeoff possible : lorsque le krep sera beaucoup plus petit, on devra peut-être augmenter le rho0. 
#define rho0_opponent   0.45
#define krep_opponent   1e-7 
#define hitbox_opponent 0.15    
//HITBOX GÉNÉRALE DES OBSTACLES : modifiable. TAG.
#define hitbox_obstacle 0.0 
// LIMITEUR DE FORCE
#define LIMIT_ATTRACTIVE_FORCE 2.999999999999999999999999999999999999999999999999999999999999999999999998
#define LIMIT_REPULSIVE_FORCE  2.999999999999999999999999999999999999999999999999999999999999999999999999


// speed max define 
#define global_vMax 0.1     
#define global_wMax 0.2



#endif
