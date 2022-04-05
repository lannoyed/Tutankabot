#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "lidar.h"
#include "SPICom.h"
#include "canCom.h"
#include "MainController.h"

char data_file[] = "prout.csv";

int main(int argc, char* argv){
    /*connectLidar();

    lidarToFile(data_file);

    disconnectLidar();*/
	SPI_init() ; 
	CAN_init() ; 
	sendTheta(0,1) ; 
	sendTheta(0,2) ; 
	Controller* ctrl ; 
	ctrl = ControllerInit() ; 
	std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now() ; 
	std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now() ; 
	std::chrono::duration<double> Dt = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t0) ; 
	set_speed(ctrl, 0.0, 0.0) ; 
	while(Dt.count() < 20.0){
		ControllerLoop(ctrl) ;
		odometryCalibration(ctrl) ; 
		t1 = std::chrono::high_resolution_clock::now() ; 
		Dt = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t0) ;
		printf("t = %f\t%f\t%f\t%f\t%f\t%f\n", Dt.count(), ctrl->sc2->speed_mes, ctrl->sc1->speed_mes, ctrl->x, ctrl->y, ctrl->theta) ;  
		//printf("%f\t", get_speed(4)) ; 
		//printf("%f\n", get_speed(5)) ;
	} 
	
	sendTheta(0,1) ; 
	sendTheta(0,2) ;
	ControllerFree(ctrl) ;
	
}