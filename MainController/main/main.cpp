# include <stdlib.h>
# include <stdio.h>
# include <unistd.h>
# include <time.h> 
# include <bitset>
# include <iostream>
# include "../CanCom/canCom.cpp"
# include "../SPICom/SPICom.cpp"
# include "../SpeedController/Speed_controller.cpp"
# include "MainController.cpp"

int main(){
	//Initialization 
	SPI_init() ; 
	CAN_init() ; 
	sendTheta(0,1) ; 
	sendTheta(0,2) ; 
	Controller* ctrl ; 
	ctrl = ControllerInit() ; 
	set_speed(ctrl, 0.01, 0.01) ; 
	for (int i = 0 ; i < 20000 ; i++){
		ControllerLoop(ctrl) ;  
		printf("i = %d\t", i) ; 
		printf("%f\t%f\t", ctrl->sc1->speed_mes, ctrl->sc1->speed_ref) ; 
		printf("%f\t%f\n", ctrl->sc2->speed_mes, ctrl->sc2->speed_ref) ;  
		//printf("%f\t", get_speed(4)) ; 
		//printf("%f\n", get_speed(5)) ;
		sleep(0.2) ; 
	} 
	
	sendTheta(0,1) ; 
	sendTheta(0,2) ;
	ControllerFree(ctrl) ; 
	return 0 ; 
}

