# include <stdio.h>
# include <stdlib.h>
# include <string.h>
# include <iostream>
# include <time.h>
# include <unistd.h>

/*
//methode abandonnée car problem de compatibilité avec printf

int toHexadecimal(int dutyCycle){
	printf(std::cout << std::hex << dutyCycle);
	return 0;
}

*/

int toHexadecimal(double dutyCycle, char* speed){
  // Pré : 
  // dutyCycle : un double variant de 0 à 100 
  // speed : pointeur de type char [2]
  // Post: renvoie la valeur conrrespondante en hexadécimal sous forme de string à l'adresse speed
	
  //commentaire : long mais éfficace attention des modules suplémentaires sont nécéssaire pour enable les nombres à vitgules 
  //ou convertir des valeurs suplémentaires à 127
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


int main(){
  int value = 0;
  char speed[2];
  for (int i = 0; i <= 101; i++){  
    toHexadecimal(i, speed);
    printf("int %i becomme   ", i);
    printf(speed);
    printf("\n");
  }
  return 0;
}

  


