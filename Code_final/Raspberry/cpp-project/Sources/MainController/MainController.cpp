# include "MainController.h"

enum {CALIB_START, CALIB_BACKWARD_1, CALIB_WALL_1, CALIB_FORWARD_1, CALIB_TURN_1, CALIB_BACKWARD_2, CALIB_WALL_2, CALIB_FORWARD_2, CALIB_TURN_2, CALIB_BACKWARD_3, CALIB_FINISH};
enum {PURPLE, YELLOW} ; 

enum {WORKSHED, STATUETTE, VITRINE, EXCAVATION_SQUARES, ACTION_FINISHED} ; // FSM action states 
enum {WS_START, WS_SETPOS1} ; //FSM action workshed states


Controller* ControllerInit(){
	// Initialises the main controller object
	Controller* ctrl = (Controller*)malloc(sizeof(Controller)) ; 
	std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now() ; 
	ctrl->sc1 = speedControllerInit(3, 0.2, 30, -30, 0.0, 5, 1) ; 
	ctrl->sc2 = speedControllerInit(3, 0.2, 30, -30, 0.0, 4, 2) ; 
	ctrl->x = 0 ; 
	ctrl->y = 0 ; 
	ctrl->v_ref = 0 ; 
	ctrl->w_ref = 0 ; 
	ctrl->tL[0] = t0;
	ctrl->tL[1] = ctrl->tL[0] ; 
	ctrl->r = 0.023 ; 
	ctrl->l = (0.18789/2) ; 
	ctrl->t_flag = t0 ;
	ctrl->action_t_flag = t0 ; 
	ctrl->calib_flag = 0;  
	ctrl->x_opp = 200.0 ; 
	ctrl->y_opp = 200.0 ; 
	ctrl->last_lidar_update = t0 ; 
	ctrl->t0 = std::chrono::high_resolution_clock::now();
	ctrl->t1 = std::chrono::high_resolution_clock::now();
	ctrl->Dt = std::chrono::duration_cast<std::chrono::duration<double>> (ctrl->t1-ctrl->t0);
	ctrl->time = ctrl->Dt.count() - 18;
	ctrl->state = 0 ; 
	ctrl->action_state = STATUETTE ; 
	ctrl->action_state_exc_square = 0;
	ctrl->action_state_statuette = 0 ;
	ctrl->action_state_vitrine = 0 ;
	ctrl->action_state_one_exc_square = 0 ; 
	ctrl->alpha0 = 0 ; 
	ctrl->first_time = 1 ; 
	ctrl->cord_present = 1;
	ctrl->cord_t_flag = std::chrono::high_resolution_clock::now();
	ctrl->score = 5 ; 
	ctrl->opponent_on_my_way ;
	return ctrl ; 
} 

void update_cord(Controller* ctrl){
	// This function checks whether the starting button is still in place 
	// To avoid the statrting of the robot due to vibration and misdetection of the starting button, a timer of 1 sec has been added.
	std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now() ;
	std::chrono::duration<double> Dt = std::chrono::duration_cast<std::chrono::duration<double>> (current_time - ctrl->cord_t_flag) ;
	if (getStartingButton()){
		ctrl->cord_present = 1 ; 
		ctrl->cord_t_flag = std::chrono::high_resolution_clock::now() ;
		return ; 
	} else {
		if (Dt.count() > 1.0){
			ctrl->cord_present = 0 ; 
			return ; 
		}
	}
}

void set_speed(Controller* ctrl, double v, double w){
	// This function changes the speed of the robot 
	ctrl->LockLidarVWRef.lock();
	ctrl->v_ref = v ; 
	ctrl->w_ref = w ; 
	ctrl->LockLidarVWRef.unlock();
}

void speedConversion(Controller* ctrl){
	// Mid level controller
	double r = ctrl->r ; 
	double l = ctrl->l ; 
	ctrl->LockLidarVWRef.lock();
	ctrl->sc1->speed_ref = 1/r * (ctrl->v_ref - l*ctrl->w_ref) ; 
	ctrl->sc2->speed_ref = 1/r * (ctrl->v_ref + l*ctrl->w_ref) ; 
	ctrl->LockLidarVWRef.unlock();
}

void ControllerLoop(Controller*  ctrl){
	// This function is used for debugging but has been replaced by the FSM loop function 
	speedConversion(ctrl) ; 
	speedControllerLoop(ctrl->sc1) ; 
	speedControllerLoop(ctrl->sc2) ; 
	odometryLoop(ctrl) ; 
	//update_lidar_data(ctrl->last_lidar_update, ctrl->lidar_angles, ctrl->lidar_distance, ctrl->lidar_quality) ; 
	//update_opponent_location(ctrl) ; 
}

void ControllerFree(Controller* ctrl){
	speedControllerFree(ctrl->sc1) ; 
	speedControllerFree(ctrl->sc2) ; 
	free(ctrl) ; 
}


void odometryLoop(Controller* ctrl){
	// This function actualises the position of the robot using odometry
	ctrl->tL[0] = ctrl->tL[1] ; 
	ctrl->tL[1] = std::chrono::high_resolution_clock::now() ; 
	std::chrono::duration<double> dtL = std::chrono::duration_cast<std::chrono::duration<double>>(ctrl->tL[1]-ctrl->tL[0]) ;  
	float wr = ctrl->sc2->speed_mes ; 
	float wl = ctrl->sc1->speed_mes ; 
	float r = ctrl->r ; 
	float l = ctrl->l ; 
	float dsr = wr*ctrl->r*dtL.count() ; 
	float dsl = wl*ctrl->r*dtL.count() ; 
	float ds = (dsr + dsl)/2.0 ;
	float dtheta = (dsr-dsl)/(2*ctrl->l) ; 
	ctrl->x += ds*cos(ctrl->theta+dtheta/2) ; 
	ctrl->y += ds*sin(ctrl->theta+dtheta/2) ; 
	ctrl->theta += dtheta ; 
	if (ctrl->theta > M_PI){
		ctrl->theta -= 2*M_PI ; 
	} else if (ctrl->theta < -M_PI){
		ctrl->theta += 2*M_PI ; 
	}
}

void odometryCalibration(Controller* ctrl){
	// FSM used for the calibration
	// Distance between back of the robot and center of the wheels : 145mm
	float val1, val2, val3, val4 ; 
	float v1, v2, v3 ; 
	float w1, w2, w3 ; 
	float calib1, calib2, calib3, calib4 ; 
	double time1, time2, time3, time4 ; 
	std::chrono::duration<double> dt ;
	std::chrono::high_resolution_clock::time_point t = std::chrono::high_resolution_clock::now(); 
	double w_ref ; 
	double kpw = 1.0 ; 
	switch (ctrl->team) {
		case PURPLE :
			val1 = 0-0.3 ; val2 = 0.65 ; val3 = -M_PI/2+0.3 ; val4 = 2.75 ; 
			calib1 = 3.0-0.145 ; calib2 = -M_PI/2 ; calib3 = 0.145 ; calib4 = 0.0 ; 
			time1 = 0.8 ; time2 = 5.0 ; 
			w1 = 1.0 ; w2 = -1.0 ;
			break ; 
		case YELLOW :
			val1 = 0+0.3 ; val2 = 0.65 ; val3 = M_PI/2-0.3 ; val4 = 0.15 ; 
			calib1 = 0.145 ; calib2 = M_PI/2 ; calib3 = 0.145 ; calib4 = 0.0 ; 
			time1 = 1.5 ; time2 = 5.0 ; 
			w1 = -1.0 ; w2 = 1.0 ;
			break ; 
	}
	switch(ctrl->calib_flag){
		case CALIB_START :
			set_speed(ctrl, 0.0, 0.0) ; 
			ctrl->calib_flag = CALIB_BACKWARD_1 ; 
			ctrl->t_flag = std::chrono::high_resolution_clock::now() ; 
			break ; 
		case CALIB_BACKWARD_1 :
			//printf("BACKWARD 1\n") ; 
			w_ref = kpw*(calib2 - ctrl->theta) ;
			set_speed(ctrl, -0.1, w_ref) ; 
			dt = std::chrono::duration_cast<std::chrono::duration<double>>(t - ctrl->t_flag) ; 
			if (dt.count() > 2.0){
				set_speed(ctrl, 0.0, 0.0) ;
				ctrl->calib_flag = CALIB_WALL_1 ; 
				ctrl->t_flag = std::chrono::high_resolution_clock::now() ;
			}
			break ; 
		case CALIB_WALL_1 :
			//printf("WALL 1\n") ; 
			set_speed(ctrl, 0.0, 0.0) ; 
			ctrl->y = calib1 ; 
			ctrl->theta = calib2 ; 
			ctrl->calib_flag = CALIB_FORWARD_1 ; 
			ctrl->t_flag = std::chrono::high_resolution_clock::now() ;
			break ; 
		case CALIB_FORWARD_1 :
			//printf("FORWARD 1\n") ; 
			w_ref = kpw*(calib2 - ctrl->theta) ; 
			set_speed(ctrl, 0.1, w_ref) ;  
			dt = std::chrono::duration_cast<std::chrono::duration<double>>(t - ctrl->t_flag) ; 
			if (dt.count() > time1){
				set_speed(ctrl, 0.0, 0.0) ; 
				ctrl->calib_flag = CALIB_TURN_1 ; 
				ctrl->t_flag = std::chrono::high_resolution_clock::now() ;
			}
			break ; 
		case CALIB_TURN_1 : 
			//printf("TURN 1\n") ; 
			dt = std::chrono::duration_cast<std::chrono::duration<double>>(t - ctrl->t_flag) ;
			make_angle(ctrl, calib4) ; 
			if (dt.count() > 4.0) {
				set_speed(ctrl, 0.0, 0.0) ; 
				ctrl->calib_flag = CALIB_BACKWARD_2 ; 
				ctrl->t_flag = std::chrono::high_resolution_clock::now() ;
			}
			break ; 
		case CALIB_BACKWARD_2 :
			//printf("BACKWARD 2 \n") ; 
			w_ref = kpw*(calib4-ctrl->theta) ;
			set_speed(ctrl, -0.1, w_ref) ; 
			dt = std::chrono::duration_cast<std::chrono::duration<double>>(t - ctrl->t_flag) ;
			if (dt.count() > time2) {
				set_speed(ctrl, 0.0, 0.0) ; 
				ctrl->calib_flag = CALIB_WALL_2 ; 
				ctrl->t_flag = std::chrono::high_resolution_clock::now() ;
			}
			break ;
		case CALIB_WALL_2 :
			//printf("WALL 2\n") ; 
			set_speed(ctrl, 0.0, 0.0) ; 
			ctrl->x = calib3 ; 
			ctrl->theta = calib4 ; 
			ctrl->calib_flag = CALIB_FORWARD_2 ; 
			break ; 
		case CALIB_FORWARD_2 :
			//printf("FORWARD 2 \n") ; 
			w_ref = kpw*(calib4-ctrl->theta) ; 
			set_speed(ctrl, 0.1, w_ref) ; 
			if (ctrl->x > val2){
				set_speed(ctrl, 0.0, 0.0) ; 
				ctrl->calib_flag = CALIB_TURN_2 ; 
				ctrl->t_flag = std::chrono::high_resolution_clock::now() ;
			}
			break ; 
		case CALIB_TURN_2 : 
			//printf("TURN 2\n") ; 
			dt = std::chrono::duration_cast<std::chrono::duration<double>>(t - ctrl->t_flag) ;
			make_angle(ctrl, calib2) ; 
			if (dt.count() > 4.0){
				set_speed(ctrl, 0.0, 0.0) ; 
				ctrl->calib_flag = CALIB_BACKWARD_3 ; 
				ctrl->t_flag = std::chrono::high_resolution_clock::now() ;
			}
			break ; 
		case CALIB_BACKWARD_3 : 
			//printf("BACKWARD 3\n") ; 
			w_ref = kpw*(calib2-ctrl->theta) ; 
			set_speed(ctrl, -0.1, 0.0) ; 
			if (ctrl->team == PURPLE){
				if (ctrl->y > val4){
					set_speed(ctrl, 0.0, 0.0) ; 
					ctrl->calib_flag = CALIB_FINISH ; 
					ctrl->t_flag = std::chrono::high_resolution_clock::now() ;
				}
			} else {
				if (ctrl->y < val4){
					set_speed(ctrl, 0.0, 0.0) ; 
					ctrl->calib_flag = CALIB_FINISH ; 
					ctrl->t_flag = std::chrono::high_resolution_clock::now() ;
				}
			}
			break ; 
		case CALIB_FINISH :
			//printf("FINISH \n") ; 
			printf("Calibrated\n") ; 
			set_speed(ctrl, 0.0, 0.0) ; 
			break ; 
		default : 
			//printf("Caca \n") ; 
			break ; 
	}
}

void update_opponent_location(Controller* ctrl){
	// This function computes the location of the opponent using the LiDAR datas. 
	size_t nb_lidar_data = sizeof(ctrl->lidar_angles)/sizeof(double) ;
	
	ctrl->LockLidarVWRef.lock();
	double v = ctrl->v_ref ; 
	double w = ctrl->w_ref ; 
	ctrl->LockLidarVWRef.unlock();
	
	double loc_opponent[nb_lidar_data][2] ;  
	int io1=0, io2=0 ; 
	double loc_opponent_final[2] ; 
	std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now() ;
	std::chrono::duration<double> dt_lidar = std::chrono::duration_cast<std::chrono::duration<double>>(ctrl->last_lidar_update-t0) ;
	double x_curr, y_curr ;
	double delta_x = -0.03638, delta_y = 0.0135 ; 
	double rl, thetal ;
	for (int i = 0 ; i < nb_lidar_data ; i++){
		if(ctrl->lidar_quality[i] > 0.0 && ctrl->lidar_distance[i] < 1.0 && ctrl->lidar_distance[i] > 0.10){
			double prop = ((double)(nb_lidar_data-i))/(double)(nb_lidar_data) ; 
			// Delta_lat = 111.96-104.96 = 7mm 
			// Delta_long = 161.53-196.53 = -35mm
			// We have to make sure that it is 5.5Hz 
			//printf("%f\t%f\n", ctrl->lidar_distance[i], ctrl->lidar_angles[i]) ; 
			v = 0.0 ; 
			w = 0.0 ; 
			rl = ctrl->lidar_distance[i] ; 
			thetal = ctrl->lidar_angles[i] ; 
			ctrl->LockLidarOurPosition.lock();
			x_curr = ctrl->x_lidar + delta_x*cos(ctrl->theta_lidar) - delta_y*sin(ctrl->theta_lidar) + rl*cos(thetal+ctrl->theta_lidar) ;
			y_curr = ctrl->y_lidar + delta_x*sin(ctrl->theta_lidar) + delta_y*cos(ctrl->theta_lidar) + rl*sin(thetal+ctrl->theta_lidar) ; 
			ctrl->LockLidarOurPosition.unlock();
			if (x_curr > 0.1 && x_curr < 1.9 && y_curr > 0.1 && y_curr < 2.9){
				//printf("Opp point detected in : %f\t%f\n", x_curr, y_curr) ; 
				loc_opponent[io1][0] = x_curr ; loc_opponent[io1][1] = y_curr ; 
				io1++ ; 
			}
		}
	}
	if (io1 != 0){
		for (int i = 0 ; i < io1 ; i++){
			loc_opponent_final[0] += loc_opponent[i][0] ; 
			loc_opponent_final[1] += loc_opponent[i][1] ; 
		}
		loc_opponent_final[0] /= io1 ; 
		loc_opponent_final[1] /= io1 ; 
	} else {
		loc_opponent_final[0] = 20 ;
		loc_opponent_final[1] = 20 ; 
	}		
	//printf("Opponent in x = %f\t y = %f\n", loc_opponent_final[0], loc_opponent_final[1]) ; 
	ctrl->LockLidarOpponentPosition.lock();
	ctrl->x_opp = loc_opponent_final[0] ; 
	ctrl->y_opp = loc_opponent_final[1] ; 
	ctrl->LockLidarOpponentPosition.unlock();
}

void triangulation(Controller* ctrl){
	// Not used finally
	size_t nb_lidar_data = sizeof(ctrl->lidar_angles)/sizeof(double) ;

	ctrl->LockLidarVWRef.lock();
	double v = ctrl->v_ref ; 
	double w = ctrl->w_ref ; 
	ctrl->LockLidarVWRef.unlock();

	double b1_radius[nb_lidar_data], b2_radius[nb_lidar_data], b3_radius[nb_lidar_data] ;
	double b1_angle[nb_lidar_data], b2_angle[nb_lidar_data], b3_angle[nb_lidar_data] ; 
	int ib1=0, ib2=0, ib3=0 ; 
	std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now() ;
	std::chrono::duration<double> dt_lidar = std::chrono::duration_cast<std::chrono::duration<double>>(ctrl->last_lidar_update-t0) ;
	double x_curr, y_curr ; 
	double ya, yb ; 
	double l_12 = 2.0, l_13 = 3.16, l_23 = 3.16 ; 
	for (int i = 0 ; i < nb_lidar_data ; i++){
		if(ctrl->lidar_quality[i] > 0.0 && ctrl->lidar_distance[i] < 4.0 && ctrl->lidar_distance[i] > 0.2){
			double prop = ((double)(nb_lidar_data-i))/(double)(nb_lidar_data) ; 
			// Delta_lat = 111.96-104.96 = 7mm 
			// Delta_long = 161.53-196.53 = -35mm
			// We have to make sure that it is 5.5Hz 
			x_curr = ctrl->lidar_distance[i]*cos(ctrl->lidar_angles[i]+ctrl->theta-w*dt_lidar.count() - w*prop/5.5 ) + ctrl->x + 0.035*cos(ctrl->theta-w*dt_lidar.count() - w*prop/5.0) - v*cos(ctrl->theta-w*dt_lidar.count() - w*prop/5.0)*(dt_lidar.count()+prop/5.0) + 0.007*sin(ctrl->theta-w*dt_lidar.count() - w*prop/5.0)*(dt_lidar.count()+prop/5.0) ; 
			y_curr = ctrl->lidar_distance[i]*sin(ctrl->lidar_angles[i]+ctrl->theta-w*dt_lidar.count() - w*prop/5.5 ) + ctrl->y - 0.035*sin(ctrl->theta-w*dt_lidar.count() - w*prop/5.0) - v*sin(ctrl->theta-w*dt_lidar.count() - w*prop/5.0)*(dt_lidar.count()+prop/5.0) + 0.007*cos(ctrl->theta-w*dt_lidar.count() - w*prop/5.0)*(dt_lidar.count()+prop/5.0);
			// Beacon1 est en face à gauche de la position de départ
			// Beacon2 est en face à droite de la position de départ 
			// Beacon3 est derrière au milieu de la position de départ 
			if(check_beacon1(x_curr, y_curr, ctrl->team)){ 
				b1_radius[ib1] = ctrl->lidar_distance[i] ; 
				b1_angle[ib1] = ctrl->lidar_angles[i] ;
				ib1++ ; 
			} else if(check_beacon2(x_curr, y_curr, ctrl->team)){ 
				b2_radius[ib2] = ctrl->lidar_distance[i] ; 
				b2_angle[ib2] = ctrl->lidar_angles[i] ;
				ib2++ ; 
			} else if(check_beacon3(x_curr, y_curr, ctrl->team)){ 
				b3_radius[ib3] = ctrl->lidar_distance[i] ; 
				b3_angle[ib3] = ctrl->lidar_angles[i] ;
				ib3++ ; 
			}
		}
	}
}



void updateTime (Controller* cvs){
	cvs->t1 = std::chrono::high_resolution_clock::now();
	cvs->Dt = std::chrono::duration_cast<std::chrono::duration<double>> (cvs->t1-cvs->t0);
	cvs->time = cvs->Dt.count() -22.0;
}

void make_angle(Controller* ctrl, double angle){
	// This function uses a proportional compensator acting on the rotation speed of the robot to align 
	// the robot with a desired angle. 
	double kp_angle = 7.0 ;

	double diff = angle - ctrl->theta;

	if (diff > M_PI){
		diff -= 2*M_PI ; 
	} else if (diff < -M_PI){
		diff += 2*M_PI ; 
	}

	double wref = kp_angle*diff;
	if (wref > 1.0){
		wref = 1.0 ; 
	} else if (wref < -1.0){
		wref = -1.0 ; 
	} 
	set_speed(ctrl, 0.0, wref) ;
}

void make_x(Controller* ctrl, double x){
	// This function uses a proportional compensator acting on the speed of the robot to align 
	// the robot with a desired x position. 
	double kp_dist = 2.0 ; 
	double vref = kp_dist*(ctrl->x-x) ; 
	if (vref > 0.2){
		vref = 0.2 ; 
	} else if (vref < -0.2){
		vref = -0.2 ; 
	}
	ctrl->LockLidarVWRef.lock();
	ctrl->v_ref = vref ; 
	ctrl->LockLidarVWRef.unlock();
}

void make_y(Controller* ctrl, double y){
	// This function uses a proportional compensator acting on the speed of the robot to align 
	// the robot with a desired y position. 
	double kp_dist = 2.0 ; 
	double vref = kp_dist*fabs(y-ctrl->y) ; 
	if (y > ctrl->y and ctrl->theta < 0 and ctrl->theta > -M_PI){
			vref *= -1 ;
	}else if (y < ctrl->y and ctrl->theta > 0 and ctrl->theta < M_PI){
		vref *= -1 ;
	}
	if (vref > 0.2){
		vref = 0.2 ; 
	} else if (vref < -0.2){
		vref = -0.2 ; 
	}
	ctrl->LockLidarVWRef.lock();
	ctrl->v_ref = vref ; 
	ctrl->LockLidarVWRef.unlock();
}

void make_pos_forward(Controller* ctrl, double x, double y, double angle){
	// This function implements the parking mode of motion os the robot used when the robot needs to perform 
	// accurate actions
	double rho = sqrt((ctrl->x-x)*(ctrl->x-x) + (ctrl->y-y)*(ctrl->y-y));
	double alpha = -ctrl->theta + atan2(y-ctrl->y, x-ctrl->x); 
	if (alpha > M_PI){
		alpha -= 2*M_PI ;
	}
	if (alpha < -M_PI){
		alpha += 2*M_PI ;
	}
	double beta = -(ctrl->theta + alpha - angle);
	if (beta > M_PI) {
		beta -= 2*M_PI ; 
	}
	if (beta < -M_PI) {
		beta += 2*M_PI ; 
	}
	double k_rho = 0.7; 
	double k_alpha = 3; 
	double k_beta = -2;

	double v_ref = k_rho*rho ;
	if (v_ref > 0.3){
		v_ref = 0.3 ; 
	}

	double w_ref = k_alpha*alpha + k_beta*beta;

	if (w_ref > 1.0){
		w_ref = 1.0 ; 
	} else if (w_ref < -1.0){
		w_ref = -1.0 ; 
	}

	ctrl->v_ref = v_ref ; 
	ctrl->w_ref = w_ref;
}

void make_pos_forward_exc(Controller* ctrl, double x, double y, double angle){
	// Not used 
	double rho = sqrt((ctrl->x-x)*(ctrl->x-x) + (ctrl->y-y)*(ctrl->y-y));
	double alpha = -ctrl->theta + atan2(y-ctrl->y, x-ctrl->x); 
	double beta = -(ctrl->theta + alpha - angle);

	double k_rho = 0.5; 
	double k_alpha = 5; 
	double k_beta = -4;

	double v_ref = k_rho*rho ;
	if (v_ref > 0.3){
		v_ref = 0.3 ; 
	}

	double w_ref = k_alpha*alpha + k_beta*beta;

	if (w_ref > 1.0){
		w_ref = 1.0 ; 
	} else if (w_ref < -1.0){
		w_ref = -1.0 ; 
	}

	ctrl->v_ref = v_ref ; 
	ctrl->w_ref = w_ref;
}

void make_pos_backward(Controller* ctrl, double x, double y, double angle){
	// Same as his forward homologue but in this case the robot goes backward
	double backward_theta = ctrl->theta + M_PI;               //on essaie de faire comme si on allait en avant
	if (backward_theta > M_PI){backward_theta -= 2*M_PI ; }

	double backward_angle = angle + M_PI;               //on adapte l'angle objectif aussi
	if (backward_angle > M_PI){backward_angle -= 2*M_PI ; }

	double rho = sqrt((ctrl->x-x)*(ctrl->x-x) + (ctrl->y-y)*(ctrl->y-y));
	double alpha = -backward_theta + atan2(y-ctrl->y, x-ctrl->x); 
	double beta = -(backward_theta + alpha - backward_angle);
	
	double k_rho = 0.5; 
	double k_alpha = 5; 
	double k_beta = -4;

	double v_ref = -k_rho*rho ; //vitesses négatives
	if (v_ref < -0.2){
		v_ref = -0.2 ; 
	}

	double w_ref = k_alpha*alpha + k_beta*beta;

	if (w_ref > 1.0){
		w_ref = 1.0 ; 
	} else if (w_ref < -1.0){
		w_ref = -1.0 ; 
	}

	printf("rho = %f, alpha = %f, beta = %f\n", rho, alpha, beta);

	ctrl->v_ref = v_ref ; 
	ctrl->w_ref = w_ref;
}

void is_opponent_on_my_way(Controller* ctrl){
	// Detects whether the location of the opponent is on the robot's way 
	ctrl->LockLidarOpponentPosition.lock();
	double x_opp = ctrl->x_opp ; 
	double y_opp = ctrl->y_opp ; 
	ctrl->LockLidarOpponentPosition.unlock();
	ctrl->LockLidarVWRef.lock();
	double v = ctrl->v_ref ; 
	double w = ctrl->w_ref ; 
	ctrl->LockLidarVWRef.unlock();
	double alpha = atan2(y_opp - ctrl->y, x_opp - ctrl->x) -ctrl->theta;
	double rho = sqrt((ctrl->x-x_opp)*(ctrl->x-x_opp) + (ctrl->y-y_opp)*(ctrl->y-y_opp)); 
	if (alpha > M_PI){
		alpha -= 2*M_PI ; 
	} 
	if (alpha < -M_PI){
		alpha += 2*M_PI ; 
	}
	ctrl->opponent_on_my_way = 0 ; 
	if (rho < 0.3) {
		if (v > 0 ){
			if (alpha < M_PI/3 && alpha > -M_PI/3){
				ctrl->opponent_on_my_way = 1 ; 
				printf("Je vais en avant et je vois un adversaire\n") ; 
			} 
		} else if (v < 0){
			if (alpha < -2*M_PI/3 && alpha > 2*M_PI/3){
				ctrl->opponent_on_my_way = 1 ; 
				printf("Je vais en arrière et je vois un adversaire\n") ; 
			}
		} else if ((alpha < M_PI/3 && alpha > -M_PI/3) || (alpha < -2*M_PI/3 && alpha > 2*M_PI/3)){
			ctrl->opponent_on_my_way = 1 ; 
			printf("Je suis à l'arrêt et je vois un adversaire\n") ; 
		}
	} 
}
