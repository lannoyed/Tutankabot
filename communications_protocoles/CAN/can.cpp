# include <stdio.h>
# include <stdlib.h>
# include <string.h>
# include <iostream>
# include <time.h>
# include <unistd.h>

int main (){	
	//initialisation
	char command1[50];
	char command2[50];
	char speed[85];
	strcpy (speed,"22232425262728292A2B2C2D2E2F30313233343536373835363738393A3B3C3D3E3F4041424344403022");
	strcpy (command1,"cansend can0 708#1EFF40");
	system (command1);
	printf("Allumage de la led\n");
	sleep(2);

	strcpy(command1,"cansend can0 708#1CFF80");
	system(command1);

	
	strcpy(command2,"cansend can0 708#1DFF80");
	system (command2) ;
	strcpy(command1,"cansend can0 708#25FFXX");
	//strcpy (commandi,cansend can0 708#25FFXX );
	for (int i = 0 ; i < 85 ; i +=2){
		command1[21] = speed[i] ;
		command1[22] = speed[i+1];
		system (command1);
		printf("commande %i \n", i/2) ;
		sleep(3);
	}
	
	strcpy(command1,"cansend can0 708#1EFF00");
	system(command1);
	printf("Extinction des feux\n");
	return 0;
	
}
