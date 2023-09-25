#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "lidar.h"
#include "SPICom.h"
#include "canCom.h"
#include "FSM.h"
# include "arduino-comm.h"

char data_file[] = "prout.csv";

// attention réactiver lidar ^^^^^^




void lidar_loop (std::atomic_bool& running,  Controller* ctrl){
 
 std::chrono::high_resolution_clock::time_point last_lidar_update = std::chrono::high_resolution_clock::now();

 FILE * myThread;
 myThread = fopen("myThread.txt", "w"); 
 fprintf(myThread, "iteration init \n" );
 
 int i = 0;
 while (running )
 {
	
	update_lidar_data(ctrl->last_lidar_update, ctrl->lidar_angles, ctrl->lidar_distance, ctrl->lidar_quality);
	
	//std::cout<< "lock opo \n";
		
	update_opponent_location(ctrl);
		


	//std::cout<< "lock our \n";
	ctrl->LockLidarOurPosition.lock();
	ctrl->LockLidarOpponentPosition.lock();
	fprintf(myThread, "%f %f %f %f \n", ctrl->x_opp, ctrl->y_opp, ctrl->x_lidar, ctrl->y_lidar);
	ctrl->LockLidarOpponentPosition.unlock();
	ctrl->LockLidarOurPosition.unlock();

  ctrl->LockLidarOurPosition.lock();
  ctrl->LockLidarOpponentPosition.lock();
  fprintf(myThread, "%f %f %f %f \n", ctrl->x_opp, ctrl->y_opp, ctrl->x_lidar, ctrl->y_lidar);
  ctrl->LockLidarOpponentPosition.unlock();
  ctrl->LockLidarOurPosition.unlock();

	i++;

 }
 
 fprintf(myThread, "end threads \n");

 //fclose(myThread);
 
}

int main(int argc, char* argv){ 
	//
	FILE* mainLog;
	mainLog = fopen("main_log2.txt", "w");
	fprintf(mainLog, "[odo1] [odo2] [x] [y] \n");  
	//
	
	arduinoSerialConnect() ;
	
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
	printf("Theta_initial = %f", ctrl->theta) ; 
	std::cout<< "FSM initialized"<<"\n"; 
  
	std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now() ; 
	std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now() ; 
	std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now() ;
	std::chrono::duration<double> Dt = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t0) ;

	set_speed(ctrl, 0.0, 0.0);
	std::cout<<"set speed"<<"\n";  
 
	double deltaT = 0.0;

	std::atomic<bool> running (true) ;
	std::thread lidar_thread(lidar_loop, std::ref(running), ctrl);
 
	while(Dt.count() < 100*60.0 && !(ctrl->state == 6) ){
 
		//FSM_loop(ctrl, 0.17); 
		//printf("x = %f\t y = %f\t theta = %f\n", ctrl->x, ctrl->y, ctrl->theta) ; 
		fprintf(mainLog,"%f %f %f %f\n",ctrl->sc1->speed_mes, ctrl->sc2->speed_mes, ctrl->x, ctrl->y);
		t2 = t1;
		t1 = std::chrono::high_resolution_clock::now() ; 
		deltaT = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t2).count();
		//std::cout<<"local time:   \t" << deltaT << "\n"; 
		//std::cout<<"globale time: \t" << Dt.count() << "\n";
		Dt = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t0) ;
		
		// NICO NICO
		//set_speed(ctrl, 0.0, 0.1);
		//ControllerLoop(ctrl);
		if (Dt.count() < 1.0) {
			set_speed(ctrl, 0.0, 0.0) ; 
		} else if (Dt.count() < 6.0){
			set_speed(ctrl, 0.1, 0.0) ; 
		} else {
			set_speed(ctrl, 0.0, 0.0) ; 
		}
		ControllerLoop(ctrl) ; 
		printf("%f\t%f\t%f\t%f\t%f\n", Dt.count(), ctrl->sc1->speed_ref, ctrl->sc1->speed_mes, ctrl->sc2->speed_ref, ctrl->sc2->speed_mes) ; 
		//
	}  
 
  std::cout <<" before running \n";
	running = false;
  std::cout <<" before joint \n";
  
	lidar_thread.join();
  std::cout <<"joint \n";
  
	t1 = std::chrono::high_resolution_clock::now() ; 
	Dt = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t0) ;
	std::cout<< "end phase "<<"\n";
	sendTheta(0,1) ; 
	sendTheta(0,2) ;
	std::cout<< "before free"<<"\n";
	ControllerFree(ctrl) ;
	std::cout <<"before end lidar"<<"\n";
	disconnectLidar();
	arduinoSerialDisconnect() ; 
	fclose(mainLog);
}



