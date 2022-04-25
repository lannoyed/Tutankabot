#include <stdlib.h>

//# include "Odometry.h"
# include "Potential_Field_Forces_gr4.h"

// init la fsm, le controlleur, les odomètres, le lidar, ...
void FSM_init(Controller *cvs);

// loop complète de la fsm 
void FSM_loop(Controller *cvs, double deltaT);

// clear les pointeurs
void FSM_finish(Controller *cvs);


/*
     Section dédiée à la décomposition de la boucle FSM
*/

// update le time, le lidar les roues odométiques et les entrées du potential field
void FSM_loop_update_before (Controller* cvs);

// effectue les actions relatives au state : STATE_CALIBRATION
void FsmCalibrationLoop(Controller* cvs);

// effectue les acrion relatives au state : STATE_GO2GOAL
void FsmToGoalLoop(Controller* cvs);

// effectue les acrion relatives au state : STATE_STUCK
void FsmStuckLoop(Controller* cvs);

// effectue les acrion relatives au state : DO_ACTION
void FsmDoActionLoop(Controller* cvs);

// effectue les acrion relatives au state : RETURN_BASE
void FsmReturnBaseLoop (Controller* cvs);

// effectue les acrion relatives au state : AT_BASE
void FsmAtBaseLoop(Controller* cvs);

// effectue les acrion relatives au state : STOP
void FsmStopLoop(Controller* cvs);

// envoie les données au controlleur 
void Fsm_loop_send_after(Controller* cvs);


// FSM DES ACTIONS
bool FSM_action(Controller* ctrl) ; 
bool FSM_action_workshed(Controller* ctrl) ; 
bool FSM_action_statuette(Controller* ctrl) ;
bool FSM_action_vitrine(Controller* ctrl) ;