# include <stdlib.h>
# include <stdio.h>
# include <unistd.h>
# include <time.h> 
# include <bitset>
# include <iostream>
# include "../CanCom/canCom.cpp"
# include "../SPICom/SPICom.cpp"
# include "../SpeedController/Speed_controller.cpp"

int main(){
	//Initialization 
	SPI_init() ; 
	CAN_init() ; 
	speedController* sc1 ; 
	sc1 = speedControllerInit(20.0, 1.0, 30, -30, time(NULL), 0.0, 5, 1);
	speedController* sc2 ; 
	sc2 = speedControllerInit(20.0, 1.0, 30, -30, time(NULL), 0.0, 4, 2);
	
	sc1->speed_ref = 1.0 ; 
	sc2->speed_ref = 1.0 ;  
	//sendTheta(10,1) ; 
	//sendTheta(10,2) ; 
	//sendTheta(0,1) ; 
	//sendTheta(0,2) ; 
	for (int i = 0 ; i < 20000 ; i++){
		speedControllerLoop(sc1, time(NULL)) ; 
		speedControllerLoop(sc2, time(NULL)) ; 
		printf("i = %d\t", i) ; 
		printf("%f\t%f\t%f\t", sc1->speed_mes, sc1->speed_ref, sc1->command) ; 
		printf("%f\t%f\t%f\n", sc2->speed_mes, sc2->speed_ref, sc2->command) ;  
		//printf("%f\t", get_speed(4)) ; 
		//printf("%f\n", get_speed(5)) ;
		sleep(0.2) ; 
	} 
	sendTheta(0,1) ; 
	sendTheta(0,2) ; 
	speedControllerFree(sc1) ; 
	speedControllerFree(sc2) ; 
	return 0 ; 
}

