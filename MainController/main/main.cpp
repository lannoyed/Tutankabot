# include <stdlib.h>
# include <stdio.h>
# include <unistd.h>
# include <time.h> 
# include "../CanCom/canCom.cpp"

int main(){
	float theta = 10 ; 
	canInit() ; 
	sendTheta(theta, 1) ;
	sendTheta(theta, 2) ; 
	sleep(5) ; 
	sendTheta(0.0, 	 1) ; 
	sendTheta(0.0,   2) ; 
	return 0 ; 
}
