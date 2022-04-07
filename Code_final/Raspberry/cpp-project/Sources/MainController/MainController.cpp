# include "MainController.h"

enum {CALIB_START, CALIB_BACKWARD_1, CALIB_WALL_1, CALIB_FORWARD_1, CALIB_TURN_1, CALIB_BACKWARD_2, CALIB_WALL_2, CALIB_FORWARD_2, CALIB_TURN_2, CALIB_BACKWARD_3, CALIB_FINISH};
enum {PURPLE, YELLOW} ; 

Controller* ControllerInit(){
	Controller* ctrl = (Controller*)malloc(sizeof(Controller)) ; 
	std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now() ; 
	ctrl->sc1 = speedControllerInit(3, 0.5, 40, -40, 0.0, 5, 1) ; 
	ctrl->sc2 = speedControllerInit(3, 0.5, 40, -40, 0.0, 4, 2) ; 
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
 	ctrl->t0 = std::chrono::high_resolution_clock::now();
  ctrl->t1 = std::chrono::high_resolution_clock::now();
  ctrl->Dt = std::chrono::duration_cast<std::chrono::duration<double>> (ctrl->t1-ctrl->t0);
	ctrl->time = ctrl->Dt.count() - 18;
 
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
			time1 = 2.3 ; time2 = 5.0 ; 
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
			printf("BACKWARD 1\n") ; 
			set_speed(ctrl, -0.1, 0.0) ; 
			dt = std::chrono::duration_cast<std::chrono::duration<double>>(t - ctrl->t_flag) ; 
			if (dt.count() > 2.0){
				set_speed(ctrl, 0.0, 0.0) ;
				ctrl->calib_flag = CALIB_WALL_1 ; 
				ctrl->t_flag = std::chrono::high_resolution_clock::now() ;
			}
			break ; 
		case CALIB_WALL_1 :
			printf("WALL 1\n") ; 
			set_speed(ctrl, 0.0, 0.0) ; 
			ctrl->y = calib1 ; 
			ctrl->theta = calib2 ; 
			ctrl->calib_flag = CALIB_FORWARD_1 ; 
			ctrl->t_flag = std::chrono::high_resolution_clock::now() ;
			break ; 
		case CALIB_FORWARD_1 :
			printf("FORWARD 1\n") ; 
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
			printf("TURN 1\n") ; 
			set_speed(ctrl, 0.0, w1) ; 
			if (ctrl->team == PURPLE){
				printf("coucou\n") ; 
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
			printf("BACKWARD 2 \n") ; 
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
			printf("WALL 2\n") ; 
			set_speed(ctrl, 0.0, 0.0) ; 
			ctrl->x = calib3 ; 
			ctrl->theta = calib4 ; 
			ctrl->calib_flag = CALIB_FORWARD_2 ; 
			break ; 
		case CALIB_FORWARD_2 :
			printf("FORWARD 2 \n") ; 
			set_speed(ctrl, 0.1, 0.0) ; 
			if (ctrl->x > val2){
				set_speed(ctrl, 0.0, 0.0) ; 
				ctrl->calib_flag = CALIB_TURN_2 ; 
				ctrl->t_flag = std::chrono::high_resolution_clock::now() ;
			}
			break ; 
		case CALIB_TURN_2 : 
			set_speed(ctrl, 0.0, w2) ;
			printf("TURN 2\n") ; 
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
			printf("BACKWARD 3\n") ; 
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
			printf("FINISH \n") ; 
			set_speed(ctrl, 0.0, 0.0) ; 
			break ; 
		default : 
			printf("Caca \n") ; 
			break ; 
	}
}

void updateTime (Controller* cvs){
	cvs->t1 = std::chrono::high_resolution_clock::now();
	cvs->Dt = std::chrono::duration_cast<std::chrono::duration<double>> (cvs->t1-cvs->t0);
	cvs->time = cvs->Dt.count() -18.0;
}

