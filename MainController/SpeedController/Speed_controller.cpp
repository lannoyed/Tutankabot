# include <stdlib.h> 
# include <stdio.h> 
# include <unistd.h> 

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


float computePIOutput(PI* pi){
	return pi->e*pi->kp + pi->I*pi->kp/pi->ti ; 
}

void speedControllerLoop(speedController *sc){
	sc->t1 = sc->t2 ; 
	sc->t2 = std::chrono::high_resolution_clock::now() ; 
	std::chrono::duration<double> dt = std::chrono::duration_cast<std::chrono::duration<double>>(sc->t2-sc->t1) ; 
	float speed_mes = get_speed(sc->spi_number) ; 
	if (fabs(speed_mes-sc->speed_mes) < 5.0){
		sc->speed_mes = speed_mes ;
	}		
	float e = sc->speed_ref - sc->speed_mes ; 
	sc->pi->e = e ; 
	float output = computePIOutput(sc->pi) ; 
	if (output > sc->lim_up){
		output = sc->lim_up ; 
	} else if (computePIOutput(sc->pi) < sc->lim_down){
		output = sc->lim_down ; 
	} else {
		sc->pi->I += (dt.count()) * e ; 
	}
	sc->command = output ; 
	sendTheta(output, sc->motor_number) ; 
}


PI* PIInit(float kp, float ti){
	PI* pi = (PI*)malloc(sizeof(PI)) ; 
	pi->kp = kp ; 
	pi->ti = ti ; 
	pi->I = 0 ; 
	pi->out = 0 ; 
	return pi ; 
}

speedController* speedControllerInit(float kp, float ti, float lim_up, float lim_down, float kphi, int spi_number, int motor_number){
	speedController* sc = (speedController*)malloc(sizeof(speedController)) ; 
	std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now() ; 
	PI* pi = PIInit(kp, ti) ; 
	sc->pi = pi ; 
	sc->lim_up = lim_up ; 
	sc->lim_down = lim_down ; 
	sc->t1 = t0 ; 
	sc->t2 = t0 ; 
	sc->command = 0 ; 
	sc->speed_ref = 0 ; 
	sc->speed_mes = 0 ; 
	sc->kphi = kphi ; 
	sc->spi_number = spi_number ; 
	sc->motor_number = motor_number ; 
	return sc ; 
}

void speedControllerFree(speedController* sc){
	free(sc->pi) ; 
	free(sc) ; 
}
