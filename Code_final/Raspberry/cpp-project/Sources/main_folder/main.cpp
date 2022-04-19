#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "lidar.h"
#include "SPICom.h"
#include "canCom.h"
#include "FSM.h"

char data_file[] = "prout.csv";


int main(int argc, char* argv){ 
	//
	FILE* mainLog;
	mainLog = fopen("main_log2.txt", "w");
	fprintf(mainLog, "[odo1] [odo2] [x] [y] \n");  
	//
  
	connectLidar();
	std::cout<< "Lidar initialized"<<"\n";
  
	SPI_init() ;
	std::cout<< "SPI initialized"<<"\n";  

	CAN_init() ; 
	std::cout<< "CAN initialized"<<"\n";
  
	sendTheta(0,1) ; 
	sendTheta(0,2) ; 
	std::cout<< "0 0 send to motors"<<"\n";   
 
	Controller* ctrl ; 
	ctrl = ControllerInit() ;
	std::cout<< "Controleur initialized"<<"\n"; 
  
	FSM_init(ctrl);
	std::cout<< "FSM initialized"<<"\n"; 
  
	std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now() ; 
	std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now() ; 
	std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now() ;
	std::chrono::duration<double> Dt = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t0) ;
	std::cout<<"set speed"<<"\n"; 
	set_speed(ctrl, 0.0, 0.0);
 
	double deltaT = 0.0;
	while(Dt.count() < 27.0){
		if(Dt.count() < 27.0){
			ControllerLoop(ctrl) ;
			printf("%f     %f\n", ctrl->x, ctrl->y) ;	
		}
		//FSM_loop(ctrl, 0.17);
		//std::cout<<" here\n";
   

		//
		//
		//fprintf(mainLog,"%f %f %f %f\n",ctrl->sc1->speed_mes, ctrl->sc2->speed_mes, ctrl->x, ctrl->y);
		t2 = t1;
		t1 = std::chrono::high_resolution_clock::now() ; 
		deltaT = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t2).count();
		//std::cout<<"local time:   \t" << deltaT << "\n"; 
		//std::cout<<"globale time: \t" << Dt.count() << "\n";
		Dt = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t0) ;
		//std::cout<<"sleep \n";
    
    
		/*double time_to_wait = 0.028 - deltaT;
		if ( time_to_wait < 0) {
			std::cout  << "negative sleep : " << time_to_wait << "\n";
			time_to_wait = 0.0;
		}
       
		sleep(time_to_wait); 
    
		//*/
	}  
	t1 = std::chrono::high_resolution_clock::now() ; 
	Dt = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t0) ;
	std::cout<< "end phase "<<"\n";
	sendTheta(0,1) ; 
	sendTheta(0,2) ;
	std::cout<< "before free"<<"\n";
	ControllerFree(ctrl) ;
	std::cout <<"before end lidar"<<"\n";
	disconnectLidar();
	fclose(mainLog);
}