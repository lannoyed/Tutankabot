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
	float t1 ; 				// Time used for integration
	float t2 ; 		
	float lim_up, lim_down 	// Upper and lower limits of the command 
	float kphi ; 			// Back emf coefficient 
} speedController ; 


float computePIOutput(PI* pi){
	return e*pi->kp + I*pi->kp/pi->ti ; 
}

void speedControllerLoop(SpeedController sc, float t ){
	sc->t1 = sc->t2 ; 
	sc->t2 = t ; 
	float e = sc->speed_ref - sc->speed_mes ; 
	sc->pi->e = e ; 
	float output = computePIOutput(sc->pi) ; 
	if (output > sc->lim_up){
		output = sc->lim_up ; 
	} else if (computePIOutput(scs->pi) < sc->lim_down){
		output = sc->lim_down ; 
	} else {
		sc->pi->I += (sc->t2-sc->t1) * e ; 
	}
	sc->command = output ; 
}

PI* PIInit(float kp, float ti){
	PI* pi = (PI*)malloc(sizeof(PI)) ; 
	pi->kp = kp ; 
	pi->ti = ti ; 
	pi->I = 0 ; 
	pi->out = 0 ; 
}

speedController* speedControllerInit(float kp, float ti, float lim_up, float lim_down, float t, float kphi){
	SpeedController* sc = (SpeedController*)malloc(sizeof(SpeedController)) ; 
	PI* pi = PIInit(kp, ti) ; 
	sc->pi = pi ; 
	sc->lim_up = lim_up ; 
	sc->lim_down = lim_down ; 
	sc->t1 = t ; 
	sc->t2 = t ; 
	sc->command = 0 ; 
	sc->speed_ref = 0 ; 
	sc->speed_mes = 0 ; 
	sc->kphi = kphi ; 
}

void speedControllerFree(SpeedController* sc){
	free(sc->pi) ; 
	free(sc) ; 
}
