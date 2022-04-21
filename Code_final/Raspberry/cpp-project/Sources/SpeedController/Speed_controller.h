# include <stdlib.h> 
# include <stdio.h> 
# include <unistd.h>
# include <chrono>
# include "SPICom.h"
# include "canCom.h"
# include <math.h>

typedef struct{
	float e ; 		// input of the compensator
	float kp ; 		// Proportional coefficient 
	float ti ; 		// Integral time constant 
	float out ; 	// Output of the compensator 
	float I ; 		// Integral term
} PI ; 

typedef struct{
	PI* pi ; 				// PI compensator 
	float speed_ref ; 		// reference speed
	float speed_mes ; 		// measured speed
	float command ; 		// Output command  (e.g. a command going from -100 to 100) 
	std::chrono::high_resolution_clock::time_point t1 ; 				// Time used for integration
	std::chrono::high_resolution_clock::time_point t2 ; 		
	float lim_up, lim_down ;	// Upper and lower limits of the command 
	float kphi ; 			// Back emf coefficient 
	int spi_number ; 
	int motor_number ; 
} speedController ;

float computePIOutput(PI* pi) ; 
void speedControllerLoop(speedController* sc) ; 
PI* PIInit(float kp, float ti) ; 
speedController* speedControllerInit(float kp, float ti, float lim_up, float lim_down, float kphi, int spi_number, int motor_number) ;  
void speedControllerFree(speedController* sc) ; 