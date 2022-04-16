# include "MainController.h"

enum {CALIB_START, CALIB_BACKWARD_1, CALIB_WALL_1, CALIB_FORWARD_1, CALIB_TURN_1, CALIB_BACKWARD_2, CALIB_WALL_2, CALIB_FORWARD_2, CALIB_TURN_2, CALIB_BACKWARD_3, CALIB_FINISH};
enum {PURPLE, YELLOW} ; 

Controller* ControllerInit(){
	Controller* ctrl = (Controller*)malloc(sizeof(Controller)) ; 
	std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now() ; 
	ctrl->sc1 = speedControllerInit(3, 0.2, 40, -40, 0.0, 5, 1) ; 
	ctrl->sc2 = speedControllerInit(3, 0.2, 40, -40, 0.0, 4, 2) ; 
	ctrl->x = 0 ; 
	ctrl->y = 0 ; 
	ctrl->theta = 0 ; 
	ctrl->v_ref = 0 ; 
	ctrl->w_ref = 0 ; 
	ctrl->tL[0] = t0;
	ctrl->tL[1] = ctrl->tL[0] ; 
	ctrl->r = 0.023 ; 
	ctrl->l = 0.18789/2 ; 
	ctrl->t_flag = t0 ; 
	ctrl->calib_flag = CALIB_START ; 
	ctrl->team = YELLOW ; 
	ctrl->x_opp = 200.0 ; 
	ctrl->y_opp = 200.0 ; 
	ctrl->last_lidar_update = t0 ; 
	ctrl->t0 = std::chrono::high_resolution_clock::now();
	ctrl->t1 = std::chrono::high_resolution_clock::now();
	ctrl->Dt = std::chrono::duration_cast<std::chrono::duration<double>> (ctrl->t1-ctrl->t0);
	ctrl->time = ctrl->Dt.count() - 18;
	ctrl->state = 0 ; 
	return ctrl ; 
} 

void set_speed(Controller* ctrl, double v, double w){
	ctrl->v_ref = v ; 
	ctrl->w_ref = w ; 
}

void speedConversion(Controller* ctrl){
	double r = ctrl->r ; 
	double l = ctrl->l ; 
	ctrl->sc1->speed_ref = 1/r * (ctrl->v_ref - l*ctrl->w_ref) ; 
	ctrl->sc2->speed_ref = 1/r * (ctrl->v_ref + l*ctrl->w_ref) ; 
}

void ControllerLoop(Controller*  ctrl){
	speedConversion(ctrl) ; 
	speedControllerLoop(ctrl->sc1) ; 
	speedControllerLoop(ctrl->sc2) ; 
	odometryLoop(ctrl) ; 
	update_lidar_data(ctrl->last_lidar_update, ctrl->lidar_angles, ctrl->lidar_distance, ctrl->lidar_quality) ; 
	update_opponent_location(ctrl) ; 
}

void ControllerFree(Controller* ctrl){
	speedControllerFree(ctrl->sc1) ; 
	speedControllerFree(ctrl->sc2) ; 
	free(ctrl) ; 
}


void odometryLoop(Controller* ctrl){
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
}

void odometryCalibration(Controller* ctrl){
	// Distance between back of the robot and center of the wheels : 145mm
	float val1, val2, val3, val4 ; 
	float v1, v2, v3 ; 
	float w1, w2, w3 ; 
	float calib1, calib2, calib3, calib4 ; 
	double time1, time2, time3, time4 ; 
	std::chrono::duration<double> dt ;
	std::chrono::high_resolution_clock::time_point t = std::chrono::high_resolution_clock::now(); 
	switch (ctrl->team) {
		case PURPLE :
			val1 = 0-0.3 ; val2 = 0.65 ; val3 = -M_PI/2+0.3 ; val4 = 2.8 ; 
			calib1 = 3.0-0.145 ; calib2 = -M_PI/2 ; calib3 = 0.145 ; calib4 = 0.0 ; 
			time1 = 1.5 ; time2 = 5.0 ; 
			w1 = 1.0 ; w2 = -1.0 ;
			break ; 
		case YELLOW :
			val1 = 0+0.3 ; val2 = 0.65 ; val3 = M_PI/2-0.3 ; val4 = 0.2 ; 
			calib1 = 0.145 ; calib2 = M_PI/2 ; calib3 = 0.145 ; calib4 = 0.0 ; 
			time1 = 2.0 ; time2 = 5.0 ; 
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
			set_speed(ctrl, -0.1, 0.0) ; 
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
			set_speed(ctrl, 0.1, 0.0) ;
			t = std::chrono::high_resolution_clock::now() ;
			dt = std::chrono::duration_cast<std::chrono::duration<double>>(t - ctrl->t_flag) ;  
			if (dt.count() > time1){
				set_speed(ctrl, 0.0, 0.0) ; 
				ctrl->calib_flag = CALIB_TURN_1 ; 
				ctrl->t_flag = std::chrono::high_resolution_clock::now() ;
			}
			break ; 
		case CALIB_TURN_1 : 
			//printf("TURN 1\n") ; 
			set_speed(ctrl, 0.0, w1) ; 
			if (ctrl->team == PURPLE){
				//printf("coucou\n") ; 
				if (ctrl->theta > val1){
					set_speed(ctrl, 0.0, 0.0) ; 
					ctrl->calib_flag = CALIB_BACKWARD_2 ; 
					ctrl->t_flag = std::chrono::high_resolution_clock::now() ;
				}
			} else {
				if (ctrl->theta < val1){
					set_speed(ctrl, 0.0, 0.0) ; 
					ctrl->calib_flag = CALIB_BACKWARD_2 ; 
					ctrl->t_flag = std::chrono::high_resolution_clock::now() ;
				}
			}
			break ; 
		case CALIB_BACKWARD_2 :
			//printf("BACKWARD 2 \n") ; 
			set_speed(ctrl, -0.1, 0.0) ; 
			t = std::chrono::high_resolution_clock::now() ;
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
			set_speed(ctrl, 0.1, 0.0) ; 
			if (ctrl->x > val2){
				set_speed(ctrl, 0.0, 0.0) ; 
				ctrl->calib_flag = CALIB_TURN_2 ; 
				ctrl->t_flag = std::chrono::high_resolution_clock::now() ;
			}
			break ; 
		case CALIB_TURN_2 : 
			set_speed(ctrl, 0.0, w2) ;
			//printf("TURN 2\n") ; 
			if (ctrl->team == PURPLE){
				if (ctrl->theta < val3){
					set_speed(ctrl, 0.0, 0.0) ; 
					ctrl->calib_flag = CALIB_BACKWARD_3 ; 
					ctrl->t_flag = std::chrono::high_resolution_clock::now() ;
				}
			} else {
				if (ctrl->theta > val3){
					set_speed(ctrl, 0.0, 0.0) ; 
					ctrl->calib_flag = CALIB_BACKWARD_3 ; 
					ctrl->t_flag = std::chrono::high_resolution_clock::now() ;
				}
				
			}
			break ; 
		case CALIB_BACKWARD_3 : 
			//printf("BACKWARD 3\n") ; 
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
			set_speed(ctrl, 0.0, 0.0) ; 
			break ; 
		default : 
			//printf("Caca \n") ; 
			break ; 
	}
}

void update_opponent_location(Controller* ctrl){
	size_t nb_lidar_data = sizeof(ctrl->lidar_angles)/sizeof(double) ;
	double v = ctrl->v_ref ; 
	double w = ctrl->w_ref ; 
	double loc_opponent[nb_lidar_data][2] ;  
	double beacon1[nb_lidar_data][2], beacon2[nb_lidar_data][2], beacon3[nb_lidar_data][2] ;
	int ib1=0, ib2=0, ib3=0, io1=0, io2=0 ; 
	double loc_opponent_final[2] ; 
	std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now() ;
	std::chrono::duration<double> dt_lidar = std::chrono::duration_cast<std::chrono::duration<double>>(ctrl->last_lidar_update-t0) ;
	double x_curr, y_curr ; 
	for (int i = 0 ; i < nb_lidar_data ; i++){
		if(ctrl->lidar_quality[i] > 0.0 && ctrl->lidar_distance[i] < 7.5){
			double prop = ((double)(nb_lidar_data-i))/(double)(nb_lidar_data) ; 
			x_curr = ctrl->lidar_distance[i]*cos(ctrl->lidar_angles[i]+ctrl->theta-w*dt_lidar.count() - w*prop/5.0 ) + ctrl->x - 0.00294*cos(ctrl->theta-w*dt_lidar.count() - w*prop/5.0) - v*cos(ctrl->theta-w*dt_lidar.count() - w*prop/5.0)*(dt_lidar.count()+prop/5.0) ; 
			y_curr = ctrl->lidar_distance[i]*sin(ctrl->lidar_angles[i]+ctrl->theta-w*dt_lidar.count() - w*prop/5.0 ) + ctrl->y - 0.00294*sin(ctrl->theta-w*dt_lidar.count() - w*prop/5.0) - v*sin(ctrl->theta-w*dt_lidar.count() - w*prop/5.0)*(dt_lidar.count()+prop/5.0) ;
			if (check_beacon1(x_curr,y_curr)){
				beacon1[ib1][0] = x_curr ; beacon1[ib1][1] = y_curr ; 
				ib1++ ; 
			} else if (check_beacon2(x_curr,y_curr)){
				beacon2[ib2][0] = x_curr ; beacon2[ib2][1] = y_curr ; 
				ib2++ ;
			} else if (check_beacon3(x_curr,y_curr)){
				beacon3[ib3][0] = x_curr ; beacon3[ib3][1] = y_curr ; 
				ib3++ ; 
			} else if (x_curr > 0.0 && x_curr < 2.0 && y_curr > 0.0 && y_curr < 3.0){
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
	}
	ctrl->x_opp = loc_opponent_final[0] ; 
	ctrl->y_opp = loc_opponent_final[1] ; 
}

void updateTime (Controller* cvs){
	cvs->t1 = std::chrono::high_resolution_clock::now();
	cvs->Dt = std::chrono::duration_cast<std::chrono::duration<double>> (cvs->t1-cvs->t0);
	cvs->time = cvs->Dt.count() -18.0;
}