# include <stdlib.h>
# include <stdio.h>
# include <chrono> 
# include <math.h>
# include "Speed_controller.h"

typedef struct{
	speedController* sc1 ;	// speedController of the wheel 1  
	speedController* sc2 ;  // speedController of the wheel 2 
	float x,y,theta ; 		// Position of the robot  
	float v_ref, w_ref ;	// Translation and rotation speed of the robot 
	
	double r ; 
	double l ;
	
	std::chrono::high_resolution_clock::time_point tL[2] ; 			// Time reference for the localization system 
	std::chrono::high_resolution_clock::time_point t_flag ; 		// Time reference fot the calibration 
	int calib_flag ; 
	int team ; 
 
  int state;
 	FILE* data ; 							///< File for data recording 

	// Global informations 
	std::chrono::high_resolution_clock::time_point t0;
	std::chrono::high_resolution_clock::time_point t1;
	std::chrono::duration<double> Dt;
	double time; 
	
} Controller ;

Controller* ControllerInit() ; 
void set_speed(Controller* ctrl, double v, double w) ; 
void speedConversion(Controller* ctrl) ;  	
void ControllerLoop(Controller* ctrl) ;
void ControllerFree(Controller* ctrl) ; 
void odometryLoop(Controller* ctrl) ;  
void odometryCalibration(Controller* ctrl) ;

void updateTime (Controller* cvs);