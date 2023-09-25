# include <stdlib.h>
# include <stdio.h>
# include <chrono> 
# include <math.h>
# include "Speed_controller.h"
# include "lidar.h"

typedef struct{
	speedController* sc1 ;	// speedController of the wheel 1  
	speedController* sc2 ;  // speedController of the wheel 2 
	float x,y,theta ; 		// Position of the robot  
	float v_ref, w_ref ;	// Translation and rotation speed of the robot 

	std::mutex LockLidarOpponentPosition;
	std::mutex LockLidarOurPosition;
	std::mutex LockLidarVWRef;
 
  double x_lidar;
  double y_lidar;
  double theta_lidar;

	double r ; 
	double l ;
	
	int state;
 	FILE* data ;
	
	double x_opp, y_opp ; 
	std::chrono::high_resolution_clock::time_point tL[2] ; 			// Time reference for the localization system 
	std::chrono::high_resolution_clock::time_point t_flag ; 		// Time reference fot the calibration 
	std::chrono::high_resolution_clock::time_point action_t_flag ; 		// Time reference fot the calibration 

	int calib_flag ; 

	int team ; // Notre numero d'equipe. 0 pour bleu, 2 pour jaune.

	double lidar_angles[8192], lidar_distance[8192], lidar_quality[8192] ; 
	std::chrono::high_resolution_clock::time_point last_lidar_update ;

	std::chrono::high_resolution_clock::time_point t0;
	std::chrono::high_resolution_clock::time_point t1;
	std::chrono::duration<double> Dt;
	int action_state ; 
	int action_state_workshed ;
	int action_state_statuette ;
	int action_state_vitrine ;
	int action_state_exc_square ;
	int action_state_one_exc_square ; 
	unsigned char score ;
	double alpha0 ; 
	int first_time ; 
	double time; 
	int cord_present ; 
	std::chrono::high_resolution_clock::time_point cord_t_flag;
	int opponent_on_my_way ; 
} Controller ;

Controller* ControllerInit() ; 
void update_cord(Controller* ctrl) ; 
void set_speed(Controller* ctrl, double v, double w) ; 
void speedConversion(Controller* ctrl) ;  	
void ControllerLoop(Controller* ctrl) ;
void ControllerFree(Controller* ctrl) ; 
void odometryLoop(Controller* ctrl) ;  
void odometryCalibration(Controller* ctrl) ;
void update_opponent_location(Controller* ctrl) ; 
void updateTime (Controller* cvs);
void make_angle(Controller* ctrl, double angle) ; 
void make_x(Controller* ctrl, double x) ; 
void make_y(Controller* ctrl, double y) ; 
void make_pos_forward(Controller* ctrl, double x, double y, double theta) ; 
void make_pos_forward_exc(Controller* ctrl, double x, double y, double theta) ;
void make_pos_backward(Controller* ctrl, double x, double y, double theta) ; 
void is_opponent_on_my_way(Controller* ctrl) ; 