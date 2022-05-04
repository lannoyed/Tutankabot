# include "canCom.h"

int toHexadecimal(double dutyCycle, char* speed){
  /* Pré : 
     dutyCycle : un double variant de 0 à 100 
     speed : pointeur de type char [2]
     Post: renvoie la valeur conrrespondante en hexadécimal sous forme de string à l'adresse speed
	
    commentaire : long mais éfficace attention des modules suplémentaires sont nécéssaire pour enable les nombres à virgules 
    ou convertir des valeurs suplémentaires à 127
  */
  
  double temp;
	speed[0] = 'F';
	speed[1] = 'F';
  if (dutyCycle > 100) {
    printf("overflow");
    return 0;
  }
  temp = dutyCycle;
  //first hexadeimal byte
	temp = dutyCycle - 64;
	if (temp >= 0){
		temp -= 32;
		if (temp>=0 ){ 
			temp -= 16;
			if (temp>=0){  //max111
				speed[0] = '7';
			}
      else{
        temp+= 16;
        speed[0] = '6'; 
      }
		}
   else {
   temp += 16;
     if (temp >= 0) {
       speed[0] = '5';
     }
     else{
       speed[0] = '4';
       temp+= 16;
     }
   }
	}
  else{
  		temp += 32;
		if (temp>=0 ){ 
			temp -= 16;
			if (temp>=0){  //max111
				speed[0] = '3';
			}
      else{
        temp+= 16;
        speed[0] = '2'; 
      }
		}
   else {
   temp += 16;
     if (temp >= 0) {
       speed[0] = '1';
     }
     else{
       speed[0] = '0';
       temp+= 16;
     }
   }
  }
  //printf("temp = %d \n",temp);
  //second hexadecimal terme
	temp -= 8;
	if (temp >= 0) {
		temp -= 4;
		if (temp >= 0) {
			temp -= 2;
			if (temp >= 0) {
				temp -= 1;
				if (temp >= 0) {
					speed[1] = 'F';
				}
				else { // 
					speed[1] = 'E';
				}
			}
			else {
				temp += 1;
				if (temp >= 0) {
					speed[1] = 'D';
				}
				else {
					speed[1] = 'C';
				}
			}
		}
		else {
			temp += 2;
			if (temp >= 0) {
				temp -= 1;
				if (temp >= 0) {
					speed[1] = 'B';
				}
				else {
					speed[1] = 'A';
				}
			}
			else {
				temp += 1;
				if (temp >= 0) {
					speed[1] = '9';
				}
				else {
					speed[1] = '8';
				}
			}
		}
	}
	else {
		temp += 4;
		if (temp >= 0) {
			temp -= 2;
			if (temp >= 0) {
				temp -= 1;
				if (temp >= 0) {
					speed[1] = '7';
				}
				else {  
					speed[1] = '6';
				}
			}
			else {
				temp += 1;
				if (temp >= 0) {
					speed[1] = '5';
				}
				else {
					speed[1] = '4';
				}
			}
		}
		else {
			temp += 2;
			if (temp >= 0) {
				temp -= 1;
				if (temp >= 0) {
					speed[1] = '3';
				}
				else {
					speed[1] = '2';
				}
			}
			else {
				temp += 1;
				if (temp >= 0) {
					speed[1] = '1';
				}
				else {
					speed[1] = '0';
				}
			}
		}
	}
	return 0;
}

double thetaToCan(float theta){
	double a = (100+theta)/2.0 ;  
	return a*0.68 ; 
}

void sendTheta(float theta, int motor){
	// Theta est la commande de la roue et peut aller de -100 à 100 
	
	char command[50] ; 
	int thetaCan ; 
	if (motor == 1 ) {	
		thetaCan = thetaToCan(-theta) ;
	} else{ 
		thetaCan = thetaToCan(theta) ;
	}
	char theta_string[2] ; 
	toHexadecimal(thetaCan, theta_string) ; 
	if (motor == 1){
 		strcpy(command, "cansend can0 708#25FFXX") ; // Template de commande pour le moteur 1 
	} else if (motor = 2){
		strcpy(command, "cansend can0 708#26FFXX") ; // Template de commande pour le moteur 2
	}
	command[21] = theta_string[0] ; 
	command[22] = theta_string[1] ; 
	system(command) ; 
}

void Brake (){
	char command[50] ;
	strcpy(command, "cansend can0 708#1EFF30");
	system(command);
}

void CAN_init(){
	char command[50] ; 

	strcpy(command, "cansend can0 708#1EFF30");
	strcpy(command, "cansend can0 708#1CFF80"); // Initialisation moteur 1 
	system(command); 
	strcpy(command, "cansend can0 708#1DFF80"); // Initialisation moteur 2
	system(command);
	strcpy(command, "cansend can0 708#25FF23") ;
	system(command);
	strcpy(command, "cansend can0 708#26FF23") ;
	system(command); 
	strcpy(command, "cansend can0 708#1EFF40"); // Allumage de la led 
	system(command) ; 
	sleep(2);
	strcpy(command, "cansend can0 708#1EFF00"); // Extinction de la led 
	system(command) ;

}

