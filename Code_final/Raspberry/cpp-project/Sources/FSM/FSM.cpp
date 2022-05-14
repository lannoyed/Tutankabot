/*! 
 * \file ctrl_main_gr2.cc
 * \brief Initialization, loop and finilization of the controller written in C (but compiled as C++)
 */

// VMAX = 14.82rad/s

// Main Author : Nicolas Isenguerre, Diego Lannoye.
// Mechatronics Project - LELME2002. 2021-2022.
// For the reference in coordinate system : see the 'RepresentationCarte.png' file.

# include "FSM.h"

#define DONT_MOVE false // J'ai changé un peu le code pour qu'il se calibre quand meme mais qu'il ne bouge plus après (Le plus bg des codeurs) 

//# include "data.h"



/*! \brief initialize controller operations (called once)
 * 
 * \param[in] cvs controller main structure
 */



// FSM status robot info
int number_sample;      // nombre de samples portés par le robot
bool isFull;            // true si le robot porte 2 samples
bool action_finished;   // true si il a passer 1.5s sur le sample
bool returnBaseTime;    // true si le robot doit retourner à la base car le jeu est fini
bool returnBaseFull;    // true si action finished et que le nombre de samples porté par le robot vaut 2 
double time_wait_init;  // permet de savoir quand le robot est arrivé sur un sample
FILE* myFileTracking;            // track the state of the FSM
double time_wait_init_waiting_for_target; // permet de sortir le robot de la recherche d'un sample qui n'existe plus
double time_begin_calibration = 0.0;

// potential field
Potential_Field myPotentialField;        ///< Potential Field object >
FILE* myFile;                    ///<log file> track information about the potential field status
std::tuple<double, double> speedConsigne; //speed values send to the controller

// lidar smoothing part 
int len_stack = 1000;           // depth of the smoothing
std::deque<double> stack_1;     // 2 stacks that preserve len_stack values of the lidar measurements
std::deque<double> stack_2;
double opponent1_x_filtered;    // pointers to the filtered values
double opponent1_y_filtered;
double opponent2_x_filtered;
double opponent2_y_filtered;
double lidar_periode = 0.001;   // lidar periode
double time_last_update_lidar = -0.2;  // take the information of the laste update of the lidar value
std::tuple<double, double> positionOpponent1Averaged;
std::tuple<double, double> positionOpponent2Averaged;
FILE* lidar_smoothing;

bool targetDetected = true;


enum {STATE_CALIBRATION, STATE_GO2GOAL, STATE_STUCK, DO_ACTION, RETURN_BASE, AT_BASE, STOP};



void FSM_init(Controller *cvs){
    


    int max;
    max = len_stack + len_stack;
    for (int i =0; i < max ; i++){
        stack_1.push_front(0.0);
        stack_2.push_front(0.0);
    }    
	cvs->team = team_number ; 
	cvs->team = team_number;
	if (cvs->team == 1){
		cvs->theta = M_PI/2 ; 
	} else {
		cvs->theta = -M_PI/2 ;
		printf("Angle initialized purple\n") ; 
	}
    double opponent1_x_filtered = 0.0;
    double opponent1_y_filtered = 0.0;
    double opponent2_x_filtered = 0.0;
    double opponent2_y_filtered = 0.0;

    //TO DO ATTENTION INIT 
    
    //speed_regulation_init(cvs) ; 	// Initialize the speed controller
    //localization_init(cvs) ;
    
    cvs->data = fopen("data.txt", "w");        

    myPotentialField = initPotentialField();

	//visualisation_potential(-0.5, 2.5, -0.5, 3.5, 1000, &myPotentialField);
	std::cout<<"heya hahahah \n";

    myFile = fopen("data_log.txt", "w");
    fprintf(myFile, "[x] [y] [Fr_x] [Fr_y] [Fa_x] [Fa_y] [v] [w] [Speed_x] [Speed_y] [distanceOpp] \n");

    myFileTracking= fopen("data_State_log.txt \n", "w");
    fprintf(myFileTracking, "[state] [x] [y]");
    
    lidar_smoothing = fopen("lidar_smoothing.txt \n", "w");
    fprintf(lidar_smoothing, "[x_opp] [y_opp] [x_lidar] [y_lidar]\n" );
    
	
	cvs->state = 0;
	if (TEST_POTENTIAL){
		cvs->state = 1;
		printf("Test_potential_field\n\n\n\n\n\n\n\n\n") ;  
		initGoalsTest(&myPotentialField, cvs->team);
		if (cvs->team){
			cvs->theta = M_PI/2;
			cvs->x = 0.7;
			cvs->y = 0.25;
		}else{
			cvs->theta = -M_PI/2;
			cvs->x = 0.7;
			cvs->y = 2.75;
		}
	}
    cvs->team = team_number;
}

/*! \brief controller loop (called every timestep)
 * 
 * \param[in] cvs controller main structure
 */
void FSM_loop(Controller *cvs, double deltaT){
	update_cord(cvs) ; 
    std::chrono::high_resolution_clock::time_point t10 = std::chrono::high_resolution_clock::now() ; 
	std::chrono::high_resolution_clock::time_point t11 = std::chrono::high_resolution_clock::now() ; 
 	double deltaT_process = std::chrono::duration_cast<std::chrono::duration<double>>(t11-t10).count();
    is_opponent_on_my_way(cvs) ; 
    updateTime(cvs);
    //std::cout<< "update time" <<"\n";

    t10 = std::chrono::high_resolution_clock::now() ;     
    odometryLoop(cvs); // localization fait à chaques appels
	t11 = std::chrono::high_resolution_clock::now() ; 
	deltaT_process = std::chrono::duration_cast<std::chrono::duration<double>>(t11-t10).count();
    //std::cout<< "update odo : " <<deltaT_process <<"\n";
    
    t10 = std::chrono::high_resolution_clock::now() ; 
    updatePotentialField(&myPotentialField, cvs);
 	t11 = std::chrono::high_resolution_clock::now() ; 
	deltaT_process = std::chrono::duration_cast<std::chrono::duration<double>>(t11-t10).count();


	cvs->LockLidarOurPosition.lock();
	cvs->x_lidar = cvs->x;
	cvs->y_lidar = cvs->y;
	cvs->theta_lidar = cvs->theta;
	cvs->LockLidarOurPosition.unlock();
    //std::cout<< "update potential : " << deltaT_process <<"\n";
    

    
    t10 = std::chrono::high_resolution_clock::now() ; 
    //update_opponent_location(cvs) ; 
	t11 = std::chrono::high_resolution_clock::now() ; 
 	deltaT_process = std::chrono::duration_cast<std::chrono::duration<double>>(t10-t11).count();
	//std::cout<< "update oponent" << deltaT_process <<"\n";
    

    t10 = std::chrono::high_resolution_clock::now() ; 
    if (time_last_update_lidar + lidar_periode < cvs->time ){ // tuple de doubles.

        cvs->LockLidarOpponentPosition.lock();
        positionOpponent1Averaged = Filter(std::make_tuple((double) cvs-> x_opp, (double) cvs-> y_opp), &stack_1, &opponent1_x_filtered, &opponent1_y_filtered);
        cvs->LockLidarOpponentPosition.unlock();

        time_last_update_lidar = cvs->time;
    }
 	t11 = std::chrono::high_resolution_clock::now() ; 
 	deltaT_process = std::chrono::duration_cast<std::chrono::duration<double>>(t10-t11).count();
    //std::cout<< "update average position oponent" << deltaT_process <<"\n";
    
    cvs->LockLidarOpponentPosition.lock();
    fprintf(lidar_smoothing, "%d %d %f %f \n",  std::get<0>(positionOpponent1Averaged),std::get<1>(positionOpponent1Averaged), cvs->x_opp, cvs->y_opp) ;
    cvs->LockLidarOpponentPosition.unlock();
    //std::cout << "first print \n";
    
    /*if(cvs->state != STATE_CALIBRATION){
        myPotentialField.goalStolenByOpponent(positionOpponent1Averaged, positionOpponent2Averaged);
    }*/

    if (cvs->time > time_stop || (myPotentialField.listOfGoal.size() == 0 && cvs->state ==  STATE_GO2GOAL)  || (returnBaseTime && myPotentialField.GoalTest() && myPotentialField.currentGoal.goalType == true  && cvs->state != RETURN_BASE )) {
        cvs->state = STOP;
    }else if (cvs->time > time_return && !returnBaseTime) {             //\\mieux de faire un fonction calculant le temps de retour estimé
        returnBaseTime = true;
        cvs->state = RETURN_BASE;
    }
    
    //std::cout<< "at case \n";

    switch (cvs->state) // cvs->state is a state stored in the controller main structure
    {
        case STATE_CALIBRATION:
            //std::cout<<"CALIBRATION"<<"\n";
            //cvs->time = 0.0;
            
            odometryCalibration(cvs);
            number_sample = 0;
			if(cvs->cord_present==0){
				printf("Cordon plus là\n") ;
			}
            if (cvs->time >= 0 && cvs->cord_present==0 ) {
                myPotentialField.didntMove = 0;
                myPotentialField.didntRotate = 0;
                returnBaseTime = false;
				#ifdef  SIMU_PROJECT
                    initGoals(&myPotentialField, EQUIPE);
                #else
                    initGoalsTest(&myPotentialField, cvs->team);
					cvs->time = 0.0;
                #endif
                cvs->state = STATE_GO2GOAL;
            }
            break;

        case STATE_GO2GOAL:
            std::cout<<"TO GOAL"<<"\n";
            //std::cout << "Current goal at : (" << std::get<0>(myPotentialField.currentGoal.position) << "," << std::get<1>(myPotentialField.currentGoal.position) << ").\n";

            // iterate potential field
            
            t10 = std::chrono::high_resolution_clock::now() ; 
            speedConsigne = iterPotentialFieldWithLogFile(&myPotentialField, deltaT, myFile); 
        	  t11 = std::chrono::high_resolution_clock::now() ; 
         		deltaT_process = std::chrono::duration_cast<std::chrono::duration<double>>(t10-t11).count();
            //std::cout<< "potential field" << deltaT_process <<"\n";
            
            cvs->LockLidarVWRef.lock();
            cvs->v_ref = (float) std::get<0>(speedConsigne);
            cvs->w_ref = (float) std::get<1>(speedConsigne);
            cvs->LockLidarVWRef.unlock();

            double xx;
            double yy; 
            xx = std::get<0>(myPotentialField.current_position);
            yy = std::get<1>(myPotentialField.current_position);  


            
            /*if (myPotentialField.GoalTest() && returnBaseFull && (0.4 <= xx && xx <= 1.0) && (yy <= 0.4 || yy >= 2.6 ) ){
                myPotentialField.removeGoal();
                returnBaseFull = false;
                cvs->state = AT_BASE;
            }else */ 
            if (targetDetected && myPotentialField.GoalTest() && myPotentialField.currentGoal.goalType == true )
            {
                //number_sample += 1;
                myPotentialField.removeGoal();
                time_wait_init = cvs->time;
                cvs->state = DO_ACTION;
            }else if (myPotentialField.GoalTest() && myPotentialField.currentGoal.goalType == false ){
                myPotentialField.removeGoal();
                myPotentialField.nextGoal(WEIGHT_GOAL);               //\\ creer gere le cas de next goals si stuck peut-être enlever le remove
                myPotentialField.didntMove = 0;
                myPotentialField.didntRotate = 0;
                cvs->state = STATE_GO2GOAL;
            }

            else if (myPotentialField.isStuck) {
                cvs->state = STATE_STUCK;
            }/*else if (myPotentialField.GoalTest()){
                time_wait_init_waiting_for_target += 0.001;
                if (time_wait_init_waiting_for_target > 1.5){
                    myPotentialField.removeGoal();
                    myPotentialField.nextGoal(WEIGHT_GOAL);               //\\ creer gere le cas de next goals si stuck peut-être enlever le remove
                    myPotentialField.didntMove = 0;
                    myPotentialField.didntRotate = 0;
                    cvs->state = STATE_GO2GOAL;
                }
            }*/
            
            
            break;

        case STATE_STUCK:
            //std::cout<<"STUCK \n";
            
            // change goal correctly
            myPotentialField.nextGoalStuck(WEIGHT_GOAL);               //\\ creer gere le cas de next goals si stuck peut-être enlever le remove
            myPotentialField.didntMove = 0;
            myPotentialField.didntRotate = 0;
            cvs->state = STATE_GO2GOAL;
            time_wait_init_waiting_for_target = 0.0;

            break;


        case DO_ACTION:
            std::cout<<"ACTION"<<"\n";
            time_wait_init_waiting_for_target = 0.0;
            
            isFull = (number_sample == 2);
			if (TEST_POTENTIAL) {
				action_finished = true;
			}
			else {
				if (cvs->opponent_on_my_way == 1) {
					set_speed(cvs, 0.0, 0.0) ; 
					action_finished = 0 ; 
				} else {
					action_finished = FSM_action(cvs) ;
				}
			}
			if (action_finished){
				printf("Action finished\n") ; 
			}
            
            //cvs->v_ref = 0.0;
            //cvs->w_ref = 0.0;

            myPotentialField.didntMove = 0;
            myPotentialField.didntRotate = 0;
            // timer pour s'arreter 3s

            // actions related to state do action
            /*if (action_finished && isFull) {
                cvs->state = RETURN_BASE;
                returnBaseFull =true;
            } else */
            if (action_finished && myPotentialField.listOfGoal.size() == 0){
                cvs->state = RETURN_BASE;
				set_speed(cvs, 0.0, 0.0) ; 
                returnBaseTime = true;
				std::cout<<"Return Base \n";
            } else if (action_finished && myPotentialField.listOfGoal.size() != 0) {
				set_speed(cvs, 0.0, 0.0) ; 
                myPotentialField.nextGoal(WEIGHT_GOAL);
                cvs->state = STATE_GO2GOAL;
				std::cout<<"NextGoal " << myPotentialField.listOfGoal.size() << "\n";
            
            } else if (action_finished){
				set_speed(cvs, 0.0, 0.0) ; 
                myPotentialField.nextGoal(WEIGHT_GOAL);
                cvs->state = STATE_GO2GOAL;
            }
            

            break;

        case RETURN_BASE :
            std::cout<<"Return base"<<"\n";
            time_wait_init_waiting_for_target = 0.0;
            myPotentialField.nextGoalBase(myPotentialField.coordonneesBase, WEIGHT_GOAL);
            cvs->state = STATE_GO2GOAL;
            break;
        
        case AT_BASE :
            //std::cout<<"at base"<<"\n";
            time_wait_init_waiting_for_target = 0.0;
            #ifdef SIMU_PROJECT
            cvs->outputs->flag_release = 1;
            #endif
            number_sample = 0;
            myPotentialField.nextGoal(WEIGHT_GOAL);
            time_begin_calibration = cvs->time;
            //cvs->state = STATE_CALIBRATION;
            cvs->state = STATE_GO2GOAL;
            break;

        case STOP:
            //std::cout<<"stop"<<"\n";
            cvs->LockLidarVWRef.lock();
            cvs->v_ref = (float) 0.0;
            cvs->w_ref = (float) 0.0;
            cvs->LockLidarVWRef.unlock();
            #ifdef SIMU_PROJECT
            cvs->outputs->flag_release = 1;
            #endif
            break;

        default:
            printf("Error: unknown state : %d !\n", cvs->state);
            exit(EXIT_FAILURE);
    }
    
    //std::cout <<"vitesse tangentielle" << cvs-> v_ref <<"\n" ; 
    //std::cout<<"vitesse angulaire" << cvs-> w_ref << "\n";
    
    if (DONT_MOVE && cvs->state != 0) {
		cvs->LockLidarVWRef.lock();
		cvs->v_ref = 0.0;
		cvs->w_ref = 0.0;
		cvs->LockLidarVWRef.unlock();
    }
    
    t10 = std::chrono::high_resolution_clock::now() ; 
    speedConversion(cvs); 
	t11 = std::chrono::high_resolution_clock::now() ; 
 	deltaT_process = std::chrono::duration_cast<std::chrono::duration<double>>(t10-t11).count();
    //std::cout<< "speed convertion" << deltaT_process <<"\n";
    
    t10 = std::chrono::high_resolution_clock::now() ; 
   	speedControllerLoop(cvs->sc1) ; 
	t11 = std::chrono::high_resolution_clock::now() ; 
	deltaT_process = std::chrono::duration_cast<std::chrono::duration<double>>(t10-t11).count() * 2;
    //std::cout<< "update " << deltaT_process <<"\n";
    
    // Actualize the command of the motors to maintain a certain speed

	  speedControllerLoop(cvs->sc2) ;
     
    fprintf(myFileTracking, "%i %f %f \n",cvs->state, cvs->x, cvs->y ) ; 


}

/*! \brief last controller operations (called once)
 * 
 * \param[in] cvs controller main structure
 */


// FSM DES ACTIONS 

enum {WORKSHED, STATUETTE, VITRINE, ACTION_FINISHED, ONE_EXCAVATION_SQUARE} ; // FSM action states 
enum {WS_START, WS_SETPOS1, WS_SETPOS2, WS_YARMDOWN, WS_YARMDOWN2, WS_SETPOS3, WS_YARMUP, WS_YARMUP2, WS_SETPOS4, WS_SETPOS5, WS_SETPOS6, WS_SETPOS7, \
    WS_SETPOS8, WS_SETPOS9, WS_SETPOS10, WS_FINISH, GOALDOWN1, GOALDOWN2, GOALUP1, GOALUP2} ; //FSM action workshed states
enum {STAT_START, STAT_SETPOS1, STAT_SETPOS2, STAT_SETPOS3, STAT_SETPOS4, STAT_OPENGRIPPER, STAT_CLOSEGRIPPER, STAT_FINISH, STAT_SETPOS5, STAT_PUSH_REPLICA, STAT_SETPOS6} ;
enum {VIT_START, VIT_CLOSEGRIPPER, VIT_FINISH, VIT_OPENGRIPPER, VIT_SETPOS1, VIT_SETPOS2, VIT_SETPOS3, VIT_SETPOS4} ;
enum {ES_START, ES_BT1, ES_BT2, ES_ENDPOS, ES_FINISH, ES_FT2, ES_FT3, ES_FT4, ES_FT5, ES_FT6, ES_FT7, ES_SETPOS1, ES_PUSH} ;
enum {ONE_ES_START, ONE_ES_SETPOS1, ONE_ES_SETPOS2, ONE_ES_SETPOS3, ONE_ES_FINISH} ;

bool FSM_action(Controller* ctrl){
	sendScore(ctrl->score);
	switch (ctrl->action_state) {
		case WORKSHED : 
			if(FSM_action_workshed(ctrl)){
				ctrl->action_state = ACTION_FINISHED ; 
				return true ; //return true pour action finished
			} else {
				return false ; 
			}
		case STATUETTE : 
			printf("Statuette\n") ;
			if(FSM_action_statuette(ctrl)){
				ctrl->action_state = VITRINE ; 
				return true ; //return true pour action finished
			} else {
				return false ; 
			}
		case VITRINE :
			if(FSM_action_vitrine(ctrl)){
				ctrl->action_state = ONE_EXCAVATION_SQUARE ; 
				return true ; //return true pour action finished
			} else {
				return false ; 
			}
		case ONE_EXCAVATION_SQUARE :
			printf("One excavation square") ;
			if(FSM_action_One_excavation_square(ctrl)){
				ctrl->action_state_one_exc_square = ACTION_FINISHED ; 
				return true ; //return true pour action finished
			} else {
				return false ; 
			}
		case ACTION_FINISHED : 
			return true ; 
		default : 
			return true ; 
	}
}

bool FSM_action_statuette(Controller* ctrl){
	double x_target, y_target, theta_target ; 
	double val_SETPOS1_x, val_SETPOS1_y, val_SETPOS1_theta, val_SETPOS2_x, val_SETPOS2_y, val_SETPOS2_theta, val_SETPOS3_x, val_SETPOS3_y, val_SETPOS3_theta, val_SETPOS4_x, val_SETPOS4_y, val_SETPOS4_theta, val_SETPOS5_x, val_SETPOS5_y, val_SETPOS5_theta, val_SETPOS6_x, val_SETPOS6_y, val_SETPOS6_theta ;
	std::chrono::high_resolution_clock::time_point t_action = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> Dt = std::chrono::duration_cast<std::chrono::duration<double>> (t_action-ctrl->action_t_flag);
	if (ctrl->team == 0){ //Team purple
		val_SETPOS1_x = 1.5, 		val_SETPOS1_y = 2.5, 		val_SETPOS1_theta = M_PI/4 ;
		val_SETPOS2_x = 1.722,	 	val_SETPOS2_y = 2.663, 		val_SETPOS2_theta = M_PI/4 ;
		val_SETPOS3_x = 1.1, 		val_SETPOS3_y = 2.5, 		val_SETPOS3_theta = 0.2 ; //make_backward
		val_SETPOS5_x = 1.6141, 	val_SETPOS5_y = 2.6559,		val_SETPOS5_theta = -M_PI/4 ; //make forward
		val_SETPOS4_x = 1.6, 		val_SETPOS4_y = 2.3,		val_SETPOS4_theta = -3*M_PI/4 ;	//make backward	
	}else{
		val_SETPOS1_x = 1.5, 		val_SETPOS1_y = 0.5, 		val_SETPOS1_theta = -M_PI/4 ;
		val_SETPOS2_x = 1.63, 		val_SETPOS2_y = 0.245, 		val_SETPOS2_theta = -M_PI/4;
		val_SETPOS3_x = 1.8, 		val_SETPOS3_y = 0.8, 		val_SETPOS3_theta = -M_PI/2; //make_backward
		val_SETPOS5_x = 1.6142,	 	val_SETPOS5_y = 0.344,		val_SETPOS5_theta = -3*M_PI/4 ; //make forward
		val_SETPOS4_x = 1.8, 		val_SETPOS4_y = 0.8, 		val_SETPOS4_theta = -M_PI/2;
	}
	switch (ctrl->action_state_statuette) {
		case STAT_START :
            //sendScore(ctrl->score);
			printf("Statuette start\n") ; 
			ctrl->action_state_statuette = STAT_SETPOS1 ;			
			return false  ; 
		
		case STAT_SETPOS1 :
			printf("Statuette set theta\n") ;
			x_target = val_SETPOS1_x ; 
			y_target = val_SETPOS1_y ; 
			theta_target = val_SETPOS1_theta ;
			
			if ( fabs(ctrl->theta - theta_target) > 0.05){
				printf("x = %f\t y = %f\t theta = %f\n", ctrl->x, ctrl->y, ctrl->theta) ; 
				make_angle(ctrl, theta_target) ;
				return false  ; 
			}
			set_speed(ctrl, 0.0, 0.0) ; 
			ctrl->first_time = 1 ; 
			ctrl->action_state_statuette = STAT_OPENGRIPPER ;
			ctrl->action_t_flag = std::chrono::high_resolution_clock::now(); 
			return false ;
		
		case STAT_OPENGRIPPER :
			printf("Gripper open%f\n", Dt.count()) ; 
			if (Dt.count() > 0.5){
				ctrl->action_state_statuette = STAT_SETPOS2 ;
                ctrl->first_time = 1 ; 
				ctrl->action_t_flag = std::chrono::high_resolution_clock::now(); 
			}
			if (ctrl->first_time == 1){
				gripperOpen() ; 
				ctrl->first_time = 0 ; 
			}
			return false ; 
		
		case STAT_SETPOS2 : 
			printf("Pos2\n") ;  
			x_target = val_SETPOS2_x ; //1.62 marche
			y_target = val_SETPOS2_y ; 
			theta_target = val_SETPOS2_theta ;
			if (Dt.count() > 6.0){
				set_speed(ctrl, 0.0, 0.0) ;
				ctrl->action_state_statuette = STAT_CLOSEGRIPPER ;
				ctrl->first_time = 1 ;
				ctrl->action_t_flag = std::chrono::high_resolution_clock::now(); 
				return false ;
			}
			if ((fabs(ctrl->y-y_target) > 0.005 || fabs(ctrl->x - x_target) > 0.005 || fabs(ctrl->theta - theta_target) > 0.01)){
				make_pos_forward(ctrl,x_target, y_target, theta_target) ;
				printf("x = %f\t y = %f\t theta = %f\n", ctrl->x, ctrl->y, ctrl->theta) ; 
				return false  ; 
			}
			set_speed(ctrl, 0.0, 0.0) ;
			ctrl->action_state_statuette = STAT_CLOSEGRIPPER ;
			ctrl->first_time = 1 ;
			ctrl->action_t_flag = std::chrono::high_resolution_clock::now(); 
			return false ; 
		
		case STAT_CLOSEGRIPPER : 
            set_speed(ctrl, 0.0, 0.0) ; 
			if (ctrl->first_time == 1){
                gripperClose() ; 
				ctrl->score += 5 ;
				sendScore(ctrl->score) ;
                ctrl->first_time = 0 ; 
            }

			if (Dt.count() > 0.5){
				ctrl->action_state_statuette = STAT_SETPOS3 ; 
                ctrl->action_t_flag = std::chrono::high_resolution_clock::now();
                ctrl->first_time = 1 ;  //pr le goal
			}
			printf("Gripper closed%f\n", Dt.count()) ; 
			return false ;
			
		case STAT_SETPOS3 : 
			printf("Pos3\n") ;  
			x_target = val_SETPOS3_x ; 
			y_target = val_SETPOS3_y ; 
			theta_target = val_SETPOS3_theta ;
			if (Dt.count() > 7.0){
				ctrl->action_state_statuette = STAT_SETPOS5 ; 
                ctrl->action_t_flag = std::chrono::high_resolution_clock::now();
                ctrl->first_time = 1 ;  //pr le goal
			}
			if ((fabs(ctrl->y-y_target) > 0.01 || fabs(ctrl->x - x_target) > 0.01 || fabs(ctrl->theta - theta_target) > 0.1)){
                make_pos_backward(ctrl, x_target, y_target, theta_target) ;
				printf("x = %f\t y = %f\t theta = %f\n", ctrl->x, ctrl->y, ctrl->theta) ; 
				return false  ;  
			}
			set_speed(ctrl, 0.0, 0.0) ;
			ctrl->action_state_statuette = STAT_SETPOS5 ; 
			ctrl->action_t_flag = std::chrono::high_resolution_clock::now(); 
			ctrl->first_time = 1 ; 
			return false ; 
			
		case STAT_SETPOS5 :
			printf("Pos5 \n") ; 
            x_target = val_SETPOS5_x ; 
            y_target = val_SETPOS5_y ; 
            theta_target = val_SETPOS5_theta ;
			
			if (Dt.count() > 11.0){
				ctrl->action_state_statuette = STAT_PUSH_REPLICA ; 
                ctrl->action_t_flag = std::chrono::high_resolution_clock::now();
                ctrl->first_time = 1 ;  //pr le goal
			}
			if ( (fabs(ctrl->y-y_target) > 0.003 || fabs(ctrl->x - x_target) > 0.003 || fabs(ctrl->theta - theta_target) > 0.1) && (Dt.count() < 7.0)) {  //on attend 7 sec
				printf("x = %f\t y = %f\t theta = %f\n", ctrl->x, ctrl->y, ctrl->theta) ; 
				make_pos_forward(ctrl, x_target, y_target, theta_target) ;
				return false  ; 
			} 
			set_speed(ctrl, 0.0, 0.0) ; 
			ctrl->action_state_statuette = STAT_PUSH_REPLICA; 
            ctrl->action_t_flag = std::chrono::high_resolution_clock::now();
			ctrl->first_time = 1 ; 
			
			return false ;
			
		case STAT_PUSH_REPLICA :
			set_speed(ctrl, 0.0, 0.0) ; 
			if (ctrl->first_time == 1){
                pushReplica() ; 
				ctrl->score += 10 ;
				sendScore(ctrl->score) ;
                ctrl->first_time = 0 ; 
            }

			if (Dt.count() > 2.0){
				ctrl->action_state_statuette = STAT_SETPOS4 ; 
                ctrl->action_t_flag = std::chrono::high_resolution_clock::now();
                ctrl->first_time = 1 ;  //pr le goal
			}
			printf("Replica pushed%f\n", Dt.count()) ; 
			return false ;
			
        case STAT_SETPOS4 : 
			printf("Pos4 \n") ; 
            x_target = val_SETPOS4_x ; 
            y_target = val_SETPOS4_y ; 
            theta_target = val_SETPOS4_theta ;

			if ( (fabs(ctrl->y-y_target) > 0.05 || fabs(ctrl->x - x_target) > 0.05 || fabs(ctrl->theta - theta_target) > 0.1) && (Dt.count() < 5.0)) {  //on attend 7 sec
				printf("x = %f\t y = %f\t theta = %f\n", ctrl->x, ctrl->y, ctrl->theta) ;
				if (ctrl->team == 1) {
					make_pos_backward(ctrl,x_target, y_target, theta_target) ;
				} else {
					make_pos_forward(ctrl,x_target, y_target, theta_target) ;
				}
				return false  ; 
			} 
			set_speed(ctrl, 0.0, 0.0) ; 
			ctrl->action_state_statuette = STAT_FINISH; 
            ctrl->action_t_flag = std::chrono::high_resolution_clock::now();
            ctrl->first_time = 1 ;  //pr le goal
			
			return false ;

		case STAT_FINISH :
		    printf("Statuette finished\n") ; 
			return true ; 
			
		default : 
			printf("Aie aie aie \n") ; 
			return true ; 
	}
}

bool FSM_action_vitrine(Controller* ctrl){
	double x_target, y_target, theta_target ; 
	double val_SETPOS1_x, val_SETPOS1_y, val_SETPOS1_theta, val_SETPOS2_x, val_SETPOS2_y, val_SETPOS2_theta, val_SETPOS3_x, val_SETPOS3_y, val_SETPOS3_theta, val_SETPOS4_x, val_SETPOS4_y, val_SETPOS4_theta ;
	std::chrono::high_resolution_clock::time_point t_action = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> Dt = std::chrono::duration_cast<std::chrono::duration<double>> (t_action-ctrl->action_t_flag);
	if (ctrl->team == 0){
		val_SETPOS1_x = 0.5, 		val_SETPOS1_y = 2.5, 		val_SETPOS1_theta = -M_PI ;
		val_SETPOS2_x = 0.085, 		val_SETPOS2_y = 2.871, 		val_SETPOS2_theta = -M_PI ;
		val_SETPOS3_x = 0.3, 		val_SETPOS3_y = 2.84, 		val_SETPOS3_theta = -M_PI ;
		val_SETPOS4_x = 0.0, 		val_SETPOS4_y = 0.0,		val_SETPOS4_theta = 0.1 ;		
	}else{
		val_SETPOS1_x = 0.5, 		val_SETPOS1_y = 0.5, 		val_SETPOS1_theta = -M_PI; // Make angle 
		val_SETPOS2_x = 0.10, 		val_SETPOS2_y = 0.285, 		val_SETPOS2_theta = -M_PI; // Make pos 
		val_SETPOS3_x = 0.3, 		val_SETPOS3_y = 0.29, 		val_SETPOS3_theta = -M_PI;
		val_SETPOS4_x = 0, 			val_SETPOS4_y = 0, 			val_SETPOS4_theta = -M_PI/4;	
	}
	switch (ctrl->action_state_vitrine) {
		case VIT_START :
			printf("Vitrine start\n") ; 
			ctrl->action_state_vitrine = VIT_SETPOS1 ;	
			ctrl->action_t_flag = std::chrono::high_resolution_clock::now();
			return false  ; 
		
		case VIT_SETPOS1 :
			printf("Statuette set theta, %f\n", Dt.count()) ;
			x_target = val_SETPOS1_x ; 
			y_target = val_SETPOS1_y ; 
			theta_target = val_SETPOS1_theta ;
			if (Dt.count() > 4.0){
				set_speed(ctrl, 0.0, 0.0) ; 
				ctrl->first_time = 1 ; 
				ctrl->action_state_vitrine = VIT_SETPOS2 ;
				ctrl->action_t_flag = std::chrono::high_resolution_clock::now();
				return false ; 
			}
			printf("x = %f\t y = %f\t theta = %f\n", ctrl->x, ctrl->y, ctrl->theta) ;
			if ( fabs(ctrl->theta - theta_target) > 0.01){ 
				make_angle(ctrl, theta_target) ;
				return false  ; 
			}
			set_speed(ctrl, 0.0, 0.0) ; 
			ctrl->first_time = 1 ; 
			ctrl->action_state_vitrine = VIT_SETPOS2 ;
			ctrl->action_t_flag = std::chrono::high_resolution_clock::now(); 
			return false ;
		
		case VIT_SETPOS2 : 
			printf("Pos2\n") ;  
			x_target = val_SETPOS2_x ; //1.62 marche
			y_target = val_SETPOS2_y ; 
			theta_target = val_SETPOS2_theta ;
			if (Dt.count() > 8.0){
				set_speed(ctrl, 0.0, 0.0) ;
				ctrl->action_state_vitrine = VIT_OPENGRIPPER ;
				ctrl->first_time = 1 ;
				ctrl->action_t_flag = std::chrono::high_resolution_clock::now(); 
				return false ; 
			}
			if ((fabs(ctrl->y-y_target) > 0.003 || fabs(ctrl->x - x_target) > 0.003 || fabs(ctrl->theta - theta_target) > 0.01)){
				make_pos_forward(ctrl,x_target, y_target, theta_target) ;
				printf("x = %f\t y = %f\t theta = %f\n", ctrl->x, ctrl->y, ctrl->theta) ; 
				return false  ; 
			}
			set_speed(ctrl, 0.0, 0.0) ;
			ctrl->action_state_vitrine = VIT_OPENGRIPPER ;
			ctrl->first_time = 1 ;
			ctrl->action_t_flag = std::chrono::high_resolution_clock::now(); 
			return false ; 
			
		case VIT_OPENGRIPPER :
			printf("Gripper open%f\n", Dt.count()) ; 
			set_speed(ctrl, 0.0, 0.0) ; 
			if (Dt.count() > 0.5){
				ctrl->action_state_vitrine = VIT_SETPOS3 ;
                ctrl->first_time = 1 ; 
				ctrl->action_t_flag = std::chrono::high_resolution_clock::now(); 
			}
			if (ctrl->first_time == 1){
				gripperOpen() ; 
				ctrl->score += 20 ;
				sendScore(ctrl->score) ;
				ctrl->first_time = 0 ; 
			}
			return false ; 
			
		case VIT_SETPOS3 : 
			printf("Pos3\n") ;  
			x_target = val_SETPOS3_x ; 
			y_target = val_SETPOS3_y ; 
			theta_target = val_SETPOS3_theta ;
			if (Dt.count() > 2.0){
				set_speed(ctrl, 0.0, 0.0) ;
				ctrl->action_state_vitrine = VIT_CLOSEGRIPPER ; 
				ctrl->action_t_flag = std::chrono::high_resolution_clock::now(); 
				ctrl->first_time = 1 ; 
				return false ; 
			}
			if ((fabs(ctrl->y-y_target) > 0.01 || fabs(ctrl->x - x_target) > 0.01 || fabs(ctrl->theta - theta_target) > 0.01)){
                make_pos_backward(ctrl, x_target, y_target, theta_target);
				printf("x = %f\t y = %f\t theta = %f\n", ctrl->x, ctrl->y, ctrl->theta) ; 
				return false  ;  
			}
			set_speed(ctrl, 0.0, 0.0) ;
			ctrl->action_state_vitrine = VIT_CLOSEGRIPPER ; 
			ctrl->action_t_flag = std::chrono::high_resolution_clock::now(); 
			ctrl->first_time = 1 ; 
			return false ; 
		
		case VIT_CLOSEGRIPPER : 
            if (ctrl->first_time == 1){
                gripperClose() ; 
                ctrl->first_time = 0 ; 
            }

			if (Dt.count() > 0.5){
				ctrl->action_state_vitrine = VIT_SETPOS4 ; 
                ctrl->action_t_flag = std::chrono::high_resolution_clock::now();
                ctrl->first_time = 1 ;  //pr le goal
			}
			printf("Gripper closed%f\n", Dt.count()) ; 
			return false ;
		
		case VIT_SETPOS4 :
			printf("Pos4 set theta\n") ;
			x_target = val_SETPOS4_x ; 
			y_target = val_SETPOS4_y ; 
			theta_target = val_SETPOS4_theta ;
			if ( fabs(ctrl->theta - theta_target) > 0.05){
				printf("x = %f\t y = %f\t theta = %f\n", ctrl->x, ctrl->y, ctrl->theta) ; 
				make_angle(ctrl, theta_target) ;
				return false  ; 
			}
			set_speed(ctrl, 0.0, 0.0) ; 
			ctrl->first_time = 1 ; 
			ctrl->action_state_vitrine = VIT_FINISH ;
			ctrl->action_t_flag = std::chrono::high_resolution_clock::now(); 
			return false ;
		
		case VIT_FINISH :
			printf("Vitrine finished\n") ; 
			return true ; 
			
		default : 
			printf("Aie aie aie \n") ; 
			return true ; 
	}
}
bool FSM_action_One_excavation_square(Controller* ctrl){
	double x_target, y_target, theta_target ; 
	double val_SETPOS1_x, val_SETPOS1_y, val_SETPOS1_theta, val_SETPOS2_x, val_SETPOS2_y, val_SETPOS2_theta, val_SETPOS3_x, val_SETPOS3_y, val_SETPOS3_theta ;
	std::chrono::high_resolution_clock::time_point t_action = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> Dt = std::chrono::duration_cast<std::chrono::duration<double>> (t_action-ctrl->action_t_flag);
	if (ctrl->team == 0){ //purple
		val_SETPOS1_x = 1.2, 		val_SETPOS1_y = 2.15, 		val_SETPOS1_theta = 0.0 ;
		val_SETPOS2_x = 1.95, 		val_SETPOS2_y = 2.138, 		val_SETPOS2_theta = 0.0 ;
		val_SETPOS3_x = 1.2, 		val_SETPOS3_y = 2.15, 		val_SETPOS3_theta = M_PI/2 ;
	}else{
		val_SETPOS1_x = 1.2, 		val_SETPOS1_y = 0.85, 		val_SETPOS1_theta = 0.0 ;
		val_SETPOS2_x = 1.95, 		val_SETPOS2_y = 0.80, 	val_SETPOS2_theta = 0.0 ;
        val_SETPOS3_x = 1.2,      	val_SETPOS3_y = 0.85,      val_SETPOS3_theta = -M_PI/2 ;
	}
	switch (ctrl->action_state_one_exc_square) {
		case ONE_ES_START :
            sendScore(ctrl->score);
			printf("One excavation start\n") ; 
			ctrl->action_state_one_exc_square = ONE_ES_SETPOS1 ;	
			ctrl->action_t_flag = std::chrono::high_resolution_clock::now();
			return false  ; 
		
		case ONE_ES_SETPOS1 :
			printf("One excavation set theta, %f\n", Dt.count()) ;
			x_target = val_SETPOS1_x ; 
			y_target = val_SETPOS1_y ; 
			theta_target = val_SETPOS1_theta ;

			printf("x = %f\t y = %f\t theta = %f\n", ctrl->x, ctrl->y, ctrl->theta) ;
			if ( fabs(ctrl->theta - theta_target) > 0.05){ 
				make_angle(ctrl, theta_target) ;
				return false  ; 
			}
			set_speed(ctrl, 0.0, 0.0) ; 
			ctrl->first_time = 1 ; 
			ctrl->action_state_one_exc_square = ONE_ES_SETPOS2;
			ctrl->action_t_flag = std::chrono::high_resolution_clock::now(); 
			return false ;
		
		case ONE_ES_SETPOS2 : 
			printf("One excavation pos2\n") ;  
			x_target = val_SETPOS2_x ;
			y_target = val_SETPOS2_y ; 
			theta_target = val_SETPOS2_theta ;
			if (Dt.count() > 7.0){
				set_speed(ctrl, 0.0, 0.0) ;
				ctrl->action_state_one_exc_square = ONE_ES_SETPOS3 ;
				ctrl->first_time = 1 ;
				ctrl->action_t_flag = std::chrono::high_resolution_clock::now(); 
				return false ; 
			}
			if ((fabs(ctrl->y-y_target) > 0.001 || fabs(ctrl->x - x_target) > 0.001 || fabs(ctrl->theta - theta_target) > 0.01)){
				make_pos_forward(ctrl,x_target, y_target, theta_target) ;
				printf("x = %f\t y = %f\t theta = %f\n", ctrl->x, ctrl->y, ctrl->theta) ; 
				return false  ; 
			}
			set_speed(ctrl, 0.0, 0.0) ;
			ctrl->action_state_one_exc_square = ONE_ES_SETPOS3 ;
			ctrl->first_time = 1 ;
			ctrl->action_t_flag = std::chrono::high_resolution_clock::now(); 
			
			return false ; 
		
		case ONE_ES_SETPOS3 : 
			printf("One excavation pos3\n") ;  
			x_target = val_SETPOS3_x ; 
			y_target = val_SETPOS3_y ; 
			theta_target = val_SETPOS3_theta ;
			if (Dt.count() > 2.0){
				set_speed(ctrl, 0.0, 0.0) ;
				ctrl->action_state_one_exc_square = ONE_ES_FINISH ; 
				ctrl->action_t_flag = std::chrono::high_resolution_clock::now(); 
				ctrl->first_time = 1 ; 
				return false ; 
			}
			if ((fabs(ctrl->y-y_target) > 0.01 || fabs(ctrl->x - x_target) > 0.01 || fabs(ctrl->theta - theta_target) > 0.01)){
                make_pos_backward(ctrl, x_target, y_target, theta_target);
				printf("x = %f\t y = %f\t theta = %f\n", ctrl->x, ctrl->y, ctrl->theta) ; 
				return false  ;  
			}
			set_speed(ctrl, 0.0, 0.0) ;
			ctrl->action_state_one_exc_square = ONE_ES_FINISH ; 
			ctrl->action_t_flag = std::chrono::high_resolution_clock::now(); 
			ctrl->first_time = 1 ; 
			return false ; 
			
		case ONE_ES_FINISH :
			printf("One excavation finished\n") ; 
			ctrl->score += 11 ;
			sendScore(ctrl->score) ;
			return true ; 
			
		default : 
			printf("Aie aie aie \n") ; 
			return true ; 
	}
}
