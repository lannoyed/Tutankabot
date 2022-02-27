# include <stdlib.h>
# include <stdio.h>
# include <unistd.h>
# include <time.h> 
# include "../CanCom/canCom.cpp"

int main(){
	float theta = 20 ; 
	canInit() ; 
	sendTheta(theta, 1) ;
	sleep(5) ; 
	sendTheta(0.0, 	 1) ; 
	return 0 ; 
}
