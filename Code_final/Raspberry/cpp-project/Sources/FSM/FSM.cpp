/*! 
 * \file ctrl_main_gr2.cc
 * \brief Initialization, loop and finilization of the controller written in C (but compiled as C++)
 */

// VMAX = 14.82rad/s

// Main Author : Nicolas Isenguerre, Diego Lannoye.
// Mechatronics Project - LELME2002. 2021-2022.
// For the reference in coordinate system : see the 'RepresentationCarte.png' file.

# include "FSM.h"

#define DONT_MOVE false 

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
    double opponent1_x_filtered = 0.0;
    double opponent1_y_filtered = 0.0;
    double opponent2_x_filtered = 0.0;
    double opponent2_y_filtered = 0.0;

    //TO DO ATTENTION INIT 
    
    //speed_regulation_init(cvs) ; 	// Initialize the speed controller
    //localization_init(cvs) ;
    
    cvs->data = fopen("data.txt", "w");        

    myPotentialField = initPotentialField();

    myFile = fopen("data_log.txt", "w");
    fprintf(myFile, "[odo1] [odo2] [x] [y] \n");

    myFileTracking= fopen("data_State_log.txt \n", "w");
    fprintf(myFileTracking, "[state] [x] [y]");
    
    lidar_smoothing = fopen("lidar_smoothing.txt \n", "w");
    fprintf(lidar_smoothing, "[x opponent] [y opponent] \n");
    cvs->state = 0;
    
}

/*! \brief controller loop (called every timestep)
 * 
 * \param[in] cvs controller main structure
 */
void FSM_loop(Controller *cvs, double deltaT){

    std::chrono::high_resolution_clock::time_point t10 = std::chrono::high_resolution_clock::now() ; 
	  std::chrono::high_resolution_clock::time_point t11 = std::chrono::high_resolution_clock::now() ; 
 		double deltaT_process = std::chrono::duration_cast<std::chrono::duration<double>>(t11-t10).count();
    
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
    
      
    fprintf(lidar_smoothing, "%d %d %f %f \n",  std::get<0>(positionOpponent1Averaged),std::get<1>(positionOpponent1Averaged), cvs->x_opp, cvs->y_opp) ;
    //std::cout << "first print \n";
    
    /*if(cvs->state != STATE_CALIBRATION){
        myPotentialField.goalStolenByOpponent(positionOpponent1Averaged, positionOpponent2Averaged);
    }*/

    if (cvs->time > time_stop || (returnBaseTime && myPotentialField.GoalTest() && myPotentialField.currentGoal.goalType == true  && cvs->state != RETURN_BASE )) {
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
            
            odometryCalibration(cvs);
            number_sample = 0;

            if (cvs->time - time_begin_calibration >= 10 && cvs->time >=15){
                myPotentialField.didntMove = 0;
                myPotentialField.didntRotate = 0;
                returnBaseTime = false;
                cvs->state = STATE_GO2GOAL;
            }

            else if (cvs->time >= 0 && cvs->time < 10 ) {
                myPotentialField.didntMove = 0;
                myPotentialField.didntRotate = 0;
                returnBaseTime = false;
                #ifdef  SIMU_PROJECT
                    initGoals(&myPotentialField, EQUIPE);
                #else
                    initGoalsTest(&myPotentialField, cvs->team);
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
            action_finished = (cvs->time - time_wait_init) > 3;
            
	        cvs->LockLidarVWRef.lock();
            cvs->v_ref = 0.0;
            cvs->w_ref = 0.0;
            cvs->LockLidarVWRef.unlock();

            myPotentialField.didntMove = 0;
            myPotentialField.didntRotate = 0;
            // timer pour s'arreter 3s

            // actions related to state do action
            /*if (action_finished && isFull) {
                cvs->state = RETURN_BASE;
                returnBaseFull =true;
            } else */
            if (action_finished && myPotentialField.numberOfGoals == 0){
                cvs->state = RETURN_BASE;
                returnBaseTime = true;
            } else if (action_finished && myPotentialField.numberOfGoals != 0) {
                myPotentialField.nextGoal(WEIGHT_GOAL);
                cvs->state = STATE_GO2GOAL;
            
            } else if (action_finished){
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
    
    if (DONT_MOVE ) {
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
     
    fprintf(myFileTracking, "%i %f %f \n",cvs->state, cvs->x, cvs->y );
    
    /*
    =======
	if (cvs->inputs->t < 0){
		odometry_calibration(cvs) ;
		
	} else if (cvs->inputs->t > 1.0 && cvs->inputs->t < 4.0){
		speed_set(cvs, 0.3, 0.3) ;
	} else if (cvs->inputs->t > 4.0 && cvs->inputs->t < 7.0){
		speed_set(cvs, 0.3, -0.5) ;
	} else {
		speed_set(cvs, 0.0, 0.0) ; 
	}
	speed_regulation(cvs) ; 
	localization_loop(cvs) ; 



	/*
	float dist1 = sqrt((x - beacon1[0]) * (x - beacon1[0]) + (y - beacon1[1]) * (y - beacon1[1]));
	float angle1 = 90 - atan2(beacon1[1] - y, beacon1[0] - x) * 180 / M_PI;
	//printf("dist1 = %f, angle1 = %f\t", dist1, angle1);

	float dist2 = sqrt((x - beacon2[0]) * (x - beacon2[0]) + (y - beacon2[1]) * (y - beacon2[1]));
	float angle2 = 90 - atan2(beacon2[1] - y, beacon2[0] - x) * 180 / M_PI;
	//printf("x = %f, y = %f", x, y);
	//printf("dist2 = %f, angle2 = %f\t", dist2, angle2);

	float dist3 = sqrt((x - beacon3[0]) * (x - beacon3[0]) + (y - beacon3[1]) * (y - beacon3[1]));
	float angle3 = 90 - atan2(beacon3[1] - y, beacon3[0] - x) * 180/M_PI;
	//printf("dist3 = %f, angle3 = %f\t", dist3, angle3 + theta);
     >>>>>>> main
     */


}

/*! \brief last controller operations (called once)
 * 
 * \param[in] cvs controller main structure
 */

void FSM_finish(Controller *cvs){
	  // TO DO ATTENTION FINITSH
    //fclose(cvs->data) ; 
	  //speed_regulation_finish(cvs) ;	// Sets all command at 0
    fclose(lidar_smoothing);
    fclose(myFile);
    fclose(myFileTracking);
}





void FSM_loop_update_before (Controller* cvs){
    updateTime(cvs);
    odometryLoop(cvs); // localization fait à chaques appels
    
    updatePotentialField( &myPotentialField, cvs);
    
    
    update_lidar_data(cvs->last_lidar_update, cvs->lidar_angles, cvs->lidar_distance, cvs->lidar_quality) ; 
	  update_opponent_location(cvs) ; 

    if (time_last_update_lidar + lidar_periode < cvs->time ){ // tuple de doubles.
        positionOpponent1Averaged = Filter(std::make_tuple((double) cvs-> x_opp, (double) cvs-> y_opp), &stack_1, &opponent1_x_filtered, &opponent1_y_filtered);
        time_last_update_lidar = cvs->time;
    }

    fprintf(lidar_smoothing, "%d %d %d %d \n",  std::get<0>(positionOpponent1Averaged),std::get<1>(positionOpponent1Averaged), cvs->x_opp, cvs->y_opp) ;
    
    /*if(cvs->state != STATE_CALIBRATION){
        myPotentialField.goalStolenByOpponent(positionOpponent1Averaged, positionOpponent2Averaged);
    }*/

    if (cvs->time > time_stop || (returnBaseTime && myPotentialField.GoalTest() && myPotentialField.currentGoal.goalType == true  && cvs->state != RETURN_BASE )) {
        cvs->state = STOP;
    }else if (cvs->time > time_return && !returnBaseTime) {             //\\mieux de faire un fonction calculant le temps de retour estimé
        returnBaseTime = true;
        cvs->state = RETURN_BASE;
    }

}

void FsmCalibrationLoop(Controller* cvs){

    odometryCalibration(cvs);
    number_sample = 0;

    if (cvs->time - time_begin_calibration >= 10 && cvs->time >=5){
        myPotentialField.didntMove = 0;
        myPotentialField.didntRotate = 0;
        returnBaseTime = false;
        cvs->state = STATE_GO2GOAL;
    }

    else if (cvs->time >= 0 && cvs->time < 5 ) {
        myPotentialField.didntMove = 0;
        myPotentialField.didntRotate = 0;
        returnBaseTime = false;
        #ifdef  SIMU_PROJECT
            initGoals(&myPotentialField, EQUIPE); // HARDCODÉ, à changer.
        #else
            initGoalsTest(&myPotentialField, cvs->team);
        #endif
        cvs->state = STATE_GO2GOAL;
    }
}

void FsmToGoalLoop(Controller* cvs){
    // iterate potential field
    speedConsigne = iterPotentialFieldWithLogFile(&myPotentialField, 0.1, myFile);
    cvs->v_ref = (float) std::get<0>(speedConsigne);
    cvs->w_ref = (float) std::get<1>(speedConsigne);

    double xx;
    double yy; 
    xx = std::get<0>(myPotentialField.current_position);
    yy = std::get<1>(myPotentialField.current_position);  


    
    /*if (myPotentialField.GoalTest() && returnBaseFull && (0.4 <= xx && xx <= 1.0) && (yy <= 0.4 || yy >= 2.6 ) ){
        myPotentialField.removeGoal();
        returnBaseFull = false;
        cvs->state = AT_BASE;
    }else */ if  (targetDetected && myPotentialField.GoalTest() && myPotentialField.currentGoal.goalType == true )
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
    } /*else if (myPotentialField.GoalTest()){
        time_wait_init_waiting_for_target += 0.001;
        if (time_wait_init_waiting_for_target > 1.5){
            myPotentialField.removeGoal();
            myPotentialField.nextGoal(WEIGHT_GOAL);               //\\ creer gere le cas de next goals si stuck peut-être enlever le remove
            myPotentialField.didntMove = 0;
            myPotentialField.didntRotate = 0;
            cvs->state = STATE_GO2GOAL;
        }
    }*/
}

void FsmStuckLoop(Controller* cvs){

    // change goal correctly
    myPotentialField.nextGoalStuck(WEIGHT_GOAL);               //\\ creer gere le cas de next goals si stuck peut-être enlever le remove
    myPotentialField.didntMove = 0;
    myPotentialField.didntRotate = 0;
    cvs->state = STATE_GO2GOAL;
    time_wait_init_waiting_for_target = 0.0;

}


void FsmDoActionLoop(Controller* cvs){
    time_wait_init_waiting_for_target = 0.0;
    
    isFull = (number_sample == 2);
    action_finished = (cvs->time - time_wait_init) > 3.0;
    
    cvs->v_ref = 0.0;
    cvs->w_ref = 0.0;

    myPotentialField.didntMove = 0;
    myPotentialField.didntRotate = 0;
    // timer pour s'arreter 3s

    // actions related to state do action
    if (action_finished && isFull) {
        cvs->state = RETURN_BASE;
        returnBaseFull = true;
    } else if (action_finished && myPotentialField.numberOfGoals == 0){
        cvs->state = RETURN_BASE;
        returnBaseTime = true;
    } else if (action_finished && myPotentialField.numberOfGoals != 0) {
        myPotentialField.nextGoal(WEIGHT_GOAL);
        cvs->state = STATE_GO2GOAL;
    
    } else if (action_finished){
        myPotentialField.nextGoal(WEIGHT_GOAL);
        cvs->state = STATE_GO2GOAL;
    }
    #ifdef SIMU_PROJECT
    cvs->outputs->flag_release = 0;
    #endif
    
}

void FsmReturnBaseLoop (Controller* cvs){

    time_wait_init_waiting_for_target = 0.0;
    myPotentialField.nextGoalBase(myPotentialField.coordonneesBase, WEIGHT_GOAL);
    cvs->state = STATE_GO2GOAL;
}

void FsmAtBaseLoop(Controller* cvs){
    time_wait_init_waiting_for_target = 0.0;
    #ifdef SIMU_PROJECT
    cvs->outputs->flag_release = 1;
    #endif
    number_sample = 0;
    myPotentialField.nextGoal(10.0);
    time_begin_calibration = cvs->time;
    //cvs->state = STATE_CALIBRATION;
    cvs->state = STATE_GO2GOAL;
}

void FsmStopLoop(Controller* cvs){
    cvs->v_ref = (float) 0.0;
    cvs->w_ref = (float) 0.0;
    #ifdef SIMU_PROJECT
    cvs->outputs->flag_release = 1;
    #endif
}

void Fsm_loop_send_after(Controller* cvs){
    speedConversion(cvs);           // Actualize the command of the motors to maintain a certain speed
   	speedControllerLoop(cvs->sc1) ; 
	speedControllerLoop(cvs->sc2) ;
}   

