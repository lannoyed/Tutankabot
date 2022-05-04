# include <stdlib.h>
# include <stdio.h>
# include <unistd.h> 
# include <string.h>
# include <iostream> 

int toHexadecimal(double dutyCycle, char* speed) ; 
double thetaToCan(float theta) ; 
void sendTheta(float theta, int motor) ; 
void CAN_init() ; 
void Brake ();
