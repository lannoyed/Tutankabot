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
#define global_vMax 0.1     
#define global_wMax 0.5



#endif



