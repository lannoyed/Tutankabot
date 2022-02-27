# include <stdlib.h>
# include <stdio.h>
# include <unistd.h> 
# include <string.h>
# include <iostream> 

int thetaToCan(float theta){
	float a = (100+theta)/2 ;
	a = a*22/50 ;  
	int b = (int)a ;  
	return b ; 
}

void sendTheta(float theta, int motor){
	char command[50] ; 
	int thetaCan = thetaToCan(theta) ;
	char theta_string[2] ; 
	sprintf(theta_string, "%d", thetaCan) ;
	if (motor == 1){
 		strcpy(command, "cansend can0 708#25FFXX") ; 
	} else if (motor = 2){
		strcpy(command, "cansend can0 708#25FFXX") ; // TODO Remplacer par la bonne adresse pour le moteur 2
	}
	command[21] = theta_string[0] ; 
	command[22] = theta_string[1] ; 
	system(command) ; 
}

void canInit(){
	char command[50] ; 
	strcpy(command, "cansend can0 708#1EFF40") ; // Allumage de la led 
	system(command) ; 
	sleep(2) ;
	strcpy(command, "cansend can0 708#1EFF00") ; // Extinction de la led 
	system(command) ; 
	strcpy(command, "cansend can0 708#1CFF80") ; // Initialisation moteur 1 
	system(command) ; 
	strcpy(command, "cansend can0 708#1DFF80") ; // Initialisation moteur 2
	system(command) ; 
}