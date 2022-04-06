/*! 
 * \file ctrl_main_gr2.cc
 * \brief Initialization, loop and finilization of the controller written in C (but compiled as C++)
 */

// VMAX = 14.82rad/s

// Main Author : Nicolas Isenguerre, Diego Lannoye.
// Mechatronics Project - LELME2002. 2021-2022.
// For the reference in coordinate system : see the 'RepresentationCarte.png' file.

# include "FSM.h"

# include "Speed_controller.h"
# include "Odometry.h"
# include "Potential_Field_Forces_gr4.h"
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
std::ofstream myFileTracking;            // track the state of the FSM
double time_wait_init_waiting_for_target; // permet de sortir le robot de la recherche d'un sample qui n'existe plus
double time_begin_calibration = 0.0;

// potential field
Potential_Field myPotentialField;        ///< Potential Field object >
std::ofstream myFile;                    ///<log file> track information about the potential field status
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
std::ofstream lidar_smoothing;

bool targetDetected = true;


enum {STATE_CALIBRATION, STATE_GO2GOAL, STATE_STUCK, DO_ACTION, RETURN_BASE, AT_BASE, STOP};

void controller_init(Controller *cvs){
    


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

    speed_regulation_init(cvs) ; 	// Initialize the speed controller
    localization_init(cvs) ;
    cvs->data = fopen("data.txt", "w");

    #ifdef  SIMU_PROJECT
    cvs->outputs->lidar_frequency = 5;
    #endif

    myPotentialField = initPotentialField();

    myFile.open("data_log.txt", std::ios::out);

    myFileTracking.open("data_State_log.txt", std::ios::out);

    lidar_smoothing.open("lidar_smoothing.txt", std::ios::out);

    cvs->state = 0;
}

/*! \brief controller loop (called every timestep)
 * 
 * \param[in] cvs controller main structure
 */
void controller_loop(Controller* *cvs){
    //set_plot((double)cvs->state, "State");
    //set_plot((double)number_sample, "Number Sample");
    //set_plot((double)returnBaseTime, "Return Base Time");
    //set_plot((double) time > time_return, "should return" );
    //set_plot((double)returnBaseFull*4, "Return Base Full");

    #ifdef  SIMU_PROJECT
    set_plot((double)cvs->inputs->target_detected *2, "Target detected");
    set_plot((double) myPotentialField.GoalTest()*3, "Goal Test");
    set_plot((double) myPotentialField.numberOfGoals, "number of goals");
    //set_plot((double) myPotentialField.listOfGoal.size(), "real number of goals");
    set_plot((double) cvs-> loc_opponent1[0], "lidar x");
    set_plot((double) cvs-> loc_opponent1[1], "lidar y");
    #endif


    #ifdef SIMU_PROJECT
    targetDetected = cvs->inputs->target_detected;
    #endif


    updateTime(cvs);
    localization_loop(cvs); // localization fait à chaques appels
    myPotentialField.updatePotentialField(cvs);

    

    if (time_last_update_lidar + lidar_periode < cvs->time ){ // tuple de doubles.
        positionOpponent1Averaged = Filter(std::make_tuple((double) cvs-> loc_opponent1[0], (double) cvs-> loc_opponent1[1]), &stack_1, &opponent1_x_filtered, &opponent1_y_filtered);
        positionOpponent2Averaged = Filter(std::make_tuple((double) cvs-> loc_opponent2[0], (double) cvs-> loc_opponent2[1]), &stack_2, &opponent2_x_filtered, &opponent2_y_filtered);
        time_last_update_lidar = cvs->time;
    }

    #ifdef  SIMU_PROJECT
    set_plot((double) std::get<0>(positionOpponent1Averaged), "x opponent");
    set_plot((double) std::get<1>(positionOpponent1Averaged), "y opponent");
    #endif

    lidar_smoothing << std::get<0>(positionOpponent1Averaged) << "\t" << std::get<1>(positionOpponent1Averaged) << "\t" << cvs-> loc_opponent1[0] << "\t" << cvs-> loc_opponent1[1] <<"\n";
  
    if(cvs->state != STATE_CALIBRATION){
        myPotentialField.goalStolenByOpponent(positionOpponent1Averaged, positionOpponent2Averaged);
    }
    
    //vérifier que l adversaire n'est pas sur un goal
    #ifdef  SIMU_PROJECT
    for (int i = 0 ; i < cvs->inputs->nb_lidar_data ; i++){
        if (cvs->inputs->last_lidar_radius[i] < 4) {
            fprintf(cvs->data, "%f\t", cvs->inputs->last_lidar_angle[i]);
        }
    }
    fprintf(cvs->data, "\n") ;
    for (int i = 0 ; i < cvs->inputs->nb_lidar_data ; i++){
        if (cvs->inputs->last_lidar_radius[i] < 4) {
            fprintf(cvs->data, "%f\t", cvs->inputs->last_lidar_radius[i]);
        }
    }
    fprintf(cvs->data, "\n");
    #endif


    if (cvs->time > time_stop || (returnBaseTime && myPotentialField.GoalTest() && myPotentialField.currentGoal.goalType == true  && cvs->state != RETURN_BASE )) {
        cvs->state = STOP;
    }else if (cvs->time > time_return && !returnBaseTime) {             //\\mieux de faire un fonction calculant le temps de retour estimé
        returnBaseTime = true;
        cvs->state = RETURN_BASE;
    }

    switch (cvs->state) // cvs->state is a state stored in the controller main structure
    {
        case STATE_CALIBRATION:
            myFileTracking << STATE_CALIBRATION << "\t"<< cvs->loc[0] <<"\t"<< cvs->loc[1] << "\n";
            odometry_calibration(cvs);
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
                    initGoals(&myPotentialField, cvs->inputs->robot_id);
                #else
                    initGoals(&myPotentialField, team_number);
                #endif
                cvs->state = STATE_GO2GOAL;
            }
            break;

        case STATE_GO2GOAL:

            myFileTracking << STATE_GO2GOAL << "\t"<< cvs->loc[0] <<"\t"<< cvs->loc[1] << "\n";
            // iterate potential field
            speedConsigne = iterPotentialFieldWithLogFile(&myPotentialField, 0.1, myFile);
            cvs->robot_speeds[0] = (float) std::get<0>(speedConsigne);
            cvs->robot_speeds[1] = (float) std::get<1>(speedConsigne);

            double xx;
            double yy; 
            xx = std::get<0>(myPotentialField.current_position);
            yy = std::get<1>(myPotentialField.current_position);  


            
            if (myPotentialField.GoalTest() && returnBaseFull && (0.4 <= xx && xx <= 1.0) && (yy <= 0.4 || yy >= 2.6 ) ){
                myPotentialField.removeGoal();
                returnBaseFull = false;
                cvs->state = AT_BASE;
            }else if (targetDetected && myPotentialField.GoalTest() && myPotentialField.currentGoal.goalType == true )
            {
                number_sample += 1;
                myPotentialField.removeGoal();
                time_wait_init = cvs->time;
                cvs->state = DO_ACTION;
            }else if (myPotentialField.GoalTest() && myPotentialField.currentGoal.goalType == false ){
                myPotentialField.removeGoal();
                myPotentialField.nextGoal(10.0);               //\\ creer gere le cas de next goals si stuck peut-être enlever le remove
                myPotentialField.didntMove = 0;
                myPotentialField.didntRotate = 0;
                cvs->state = STATE_GO2GOAL;
            }

            else if (myPotentialField.isStuck) {
                cvs->state = STATE_STUCK;
            }else if (myPotentialField.GoalTest()){
                time_wait_init_waiting_for_target += 0.001;
                if (time_wait_init_waiting_for_target > 1.5){
                    myPotentialField.removeGoal();
                    myPotentialField.nextGoal(10.0);               //\\ creer gere le cas de next goals si stuck peut-être enlever le remove
                    myPotentialField.didntMove = 0;
                    myPotentialField.didntRotate = 0;
                    cvs->state = STATE_GO2GOAL;
                }
            }
            break;

        case STATE_STUCK:
            myFileTracking << STATE_STUCK << "\t"<< cvs->loc[0] <<"\t"<< cvs->loc[1] << "\n";

            // change goal correctly
            myPotentialField.nextGoalStuck(10.0);               //\\ creer gere le cas de next goals si stuck peut-être enlever le remove
            myPotentialField.didntMove = 0;
            myPotentialField.didntRotate = 0;
            cvs->state = STATE_GO2GOAL;
            time_wait_init_waiting_for_target = 0.0;

            break;


        case DO_ACTION:
            time_wait_init_waiting_for_target = 0.0;
            myFileTracking << DO_ACTION << "\t"<< cvs->loc[0] <<"\t"<< cvs->loc[1] << "\n";
            isFull = (number_sample == 2);
            action_finished = (cvs->time - time_wait_init) > 1.5;
            

            cvs->robot_speeds[0] = 0.0;
            cvs->robot_speeds[1] = 0.0;

            myPotentialField.didntMove = 0;
            myPotentialField.didntRotate = 0;
            // timer pour s'arreter 3s

            // actions related to state do action
            if (action_finished && isFull) {
                cvs->state = RETURN_BASE;
                returnBaseFull =true;
            } else if (action_finished && myPotentialField.numberOfGoals == 0){
                cvs->state = RETURN_BASE;
                returnBaseTime = true;
            } else if (action_finished && myPotentialField.numberOfGoals != 0) {
                myPotentialField.nextGoal(10.0);
                cvs->state = STATE_GO2GOAL;
            
            } else if (action_finished){
                myPotentialField.nextGoal(10.0);
                cvs->state = STATE_GO2GOAL;
            }
            cvs->outputs->flag_release = 0;
            break;

        case RETURN_BASE :
            time_wait_init_waiting_for_target = 0.0;
            myFileTracking << RETURN_BASE << "\t"<< cvs->loc[0] <<"\t"<< cvs->loc[1] << "\n";
            myPotentialField.nextGoalBase(myPotentialField.coordonneesBase, 10.0);
            cvs->state = STATE_GO2GOAL;
            break;
        
        case AT_BASE :
            time_wait_init_waiting_for_target = 0.0;
            myFileTracking << AT_BASE << "\t"<< cvs->loc[0] <<"\t"<< cvs->loc[1] << "\n";
            cvs->outputs->flag_release = 1;
            number_sample = 0;
            myPotentialField.nextGoal(10.0);
            time_begin_calibration = cvs->time;
            //cvs->state = STATE_CALIBRATION;
            cvs->state = STATE_GO2GOAL;
            break;

        case STOP:
            myFileTracking << STOP << "\t"<< cvs->loc[0] <<"\t"<< cvs->loc[1] << "\n";
            cvs->robot_speeds[0] = (float) 0.0;
            cvs->robot_speeds[1] = (float) 0.0;
            cvs->outputs->flag_release = 1;
            break;

        default:
            printf("Error: unknown state : %d !\n", cvs->state);
            exit(EXIT_FAILURE);
    }
    speed_regulation(cvs);            // Actualize the command of the motors to maintain a certain speed

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

void controller_finish(Controller *cvs){
	
    //fclose(cvs->data) ; 
	speed_regulation_finish(cvs) ;	// Sets all command at 0
    lidar_smoothing.close();
    myFile.close();
    myFileTracking.close();

}

NAMESPACE_CLOSE();
