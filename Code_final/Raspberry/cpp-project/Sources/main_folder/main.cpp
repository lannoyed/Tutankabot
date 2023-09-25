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
 while (running)
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
	FILE* SonarLog;
	SonarLog = fopen("SonarLog.txt", "w");
	fprintf(SonarLog, "[Sonar1] [Sonar2] [Sonar3] [Sonar4] [Sonar5]");
	

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


	printf("The program is launch in the following configuration \n");
	#if NON_LIDAR_DETECTION  
	printf("non lidar detection \n");
	#endif

	#if VISUALISATION_TEST
	printf("visualisation test \n");
	#endif

	#if !MoveByHand
		set_speed(ctrl, 0.0, 0.0);
	#else 
		Brake();
		printf("move by hand \n");
	#endif

	#if DONT_MOVE
		printf("Don't move \n");
	#endif

	#if TEST_POTENTIAL
		printf("Test potential \n");
	#endif

  #if SONAR
    printf("Sonars \n");
  #endif
  
	if (! (NON_LIDAR_DETECTION || VISUALISATION_TEST || DONT_MOVE || TEST_POTENTIAL || !MoveByHand)){
		printf("Normal mode \n");
	}


	double deltaT = 0.0;

	std::atomic<bool> running (true) ;
	std::thread lidar_thread(lidar_loop, std::ref(running), ctrl);
  
  int state_motor = 0;
	while(Dt.count() < 200*1.0 && !(ctrl->state == 6) ){
		
		t1 = std::chrono::high_resolution_clock::now() ; 

		#if Sonar
			double sonar1 = get_distance(8);
			double sonar2 = get_distance(9);
			double sonar3 = get_distance(10);
			double sonar4 = get_distance(11);
			double sonar5 = get_distance(12);
			fprintf(SonarLog, "%f %f %f %f %f \n", sonar1, sonar2, sonar3, sonar4, sonar5); 
			if (Dt.count() > 2*60.0) {ctrl->state = 6;}
		#endif
 
		#if FSM
		FSM_loop(ctrl, 0.17); 
		//printf("x = %f\t y = %f\t theta = %f\n", ctrl->x, ctrl->y, ctrl->theta) ; 
		fprintf(mainLog,"%f %f %f %f\n",ctrl->sc1->speed_mes, ctrl->sc2->speed_mes, ctrl->x, ctrl->y);
		#endif 


		t2 = t1;
		t1 = std::chrono::high_resolution_clock::now() ; 
		deltaT = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t2).count();
		//std::cout<<"local time:   \t" << deltaT << "\n"; 
		//std::cout<<"globale time: \t" << Dt.count() << "\n";
		Dt = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t0) ;
		
		// NICO NICO
		//sendTheta(Dt.count()/15.0 *50, 0);
		//sendTheta(Dt.count()/15.0 *50, 1);
   
		//ControllerLoop(ctrl);
   
    /*
    if (  (int) Dt.count() / 5 % 2 == 0  && state_motor ==0) {
        probesUp();
        state_motor = 1;
    } 
    else if (  (int) Dt.count() / 5 % 2 == 1  && state_motor ==1) {
       state_motor =0;
       probesDown();
    }*/

    
		double time_to_wait = 0.028 - deltaT;
		if ( time_to_wait < 0) {
			//std::cout  << "negative sleep : " << time_to_wait << "\n";
			time_to_wait = 0.0;
		}
       
		sleep(time_to_wait); 
    
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



