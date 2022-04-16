#include "arduino-comm.h"

int fd;

int arduinoSerialConnect(){
	if((fd=serialOpen("/dev/ttyACM0",9600)) < 0){
		printf("Unable to connect to Arduino\n");
		return -1;
	}
	else{
		printf("Arduino connected (port ttyACM0)\n");
	}
	return 0;
}

int arduinoSerialDisconnect(){
	fflush (stdout);
	serialClose(fd);
}

void sendSerialChar(unsigned char c){
	serialPutchar(fd, c);
}

int sendScore(unsigned char score){
	
	if((int) score >= 200){
		printf("Unvalid score value\n");
		return -1;
	}

	serialPutchar(fd, score);

	int response = serialGetchar(fd);

	if (response < 0){
		printf("No response from the arduino\n");
		return -1;
	}
	else if(response != 1){
		printf("Wrong response from the arduino\n");
		return -1;
	}
	return 0;
}
