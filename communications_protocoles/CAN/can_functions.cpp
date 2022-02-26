# include <stdio.h>
# include <stdlib.h>
# include <string.h>
# include <iostream>
# include <time.h>
# include <unistd.h>

int sendMotorMessage(char speed0, char speed1 , char * command) {
    /*  Pré : 
        speed0 et speed1 : caractères comprenant l'information de duty cycle sous forme de char 
        command : pointeur comprenant la commande de type duty cycle à la quelle speed0 et speed1 doivent être update
        Post :
        envoie la commande can via le terminal
    */
		command[21] = speed0;
		command[22] = speed1;
		system (command);
		//printf("commande %c%c \n", speed0, speed1);
 		return 0;
}

int toHexadecimal(double dutyCycle, char* speed){
  /* Pré : 
     dutyCycle : un double variant de 0 à 100 
     speed : pointeur de type char [2]
     Post: renvoie la valeur conrrespondante en hexadécimal sous forme de string à l'adresse speed
	
    commentaire : long mais éfficace attention des modules suplémentaires sont nécéssaire pour enable les nombres à vitgules 
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


int main (){	
	//initialisation
	char command1[50];
	char command2[50];
	char speed[85];
  char hexa[2];
  hexa[0] = '0';
  hexa[1] = '0';
	strcpy (speed,"22232425262728292A2B2C2D2E2F30313233343536373835363738393A3B3C3D3E3F4041424344403022");
	strcpy (command1,"cansend can0 708#1EFF40");
	system (command1);
	printf("Allumage de la led\n");

	strcpy(command1,"cansend can0 708#1CFF80");
	system(command1);
	strcpy(command2,"cansend can0 708#1DFF80");
	system (command2);
	printf("initialisation des moteurs check\n");
	
	strcpy(command1,"cansend can0 708#25FFXX");
	strcpy(command2,"cansend can0 708#25FFXX");

	//boucle des messages
  clock_t t;
  t = clock();
  int n = 200;
	for (int i = 0 ; i < n ; i ++){
		 
    toHexadecimal(i%100, hexa);
		sendMotorMessage(hexa[0], hexa[1] , command1);
		//sleep(3); printf(1);
	}
	t = clock() - t;
  double time_taken = ((double) t ) / CLOCKS_PER_SEC;
  printf("%i itérations en %f seconds", n, time_taken );
	strcpy(command1,"cansend can0 708#1EFF00");
	system(command1);
	printf("Extinction des feux\n");
	return 0;
	
}
