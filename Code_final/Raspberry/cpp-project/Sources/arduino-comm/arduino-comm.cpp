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
	return 0;
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

int pushReplica(){
	serialPutchar(fd, (unsigned char) 246);

	int response = serialGetchar(fd);

	if (response < 0){
		printf("No response from the arduino\n");
		return -1;
	}
	else if(response != 246){
		printf("Wrong response from the arduino\n");
		return -1;
	}
	return 0;
}

int pushExcavationSquare(){
	serialPutchar(fd, (unsigned char) 247);

	int response = serialGetchar(fd);

	if (response < 0){
		printf("No response from the arduino\n");
		return -1;
	}
	else if(response != 247){
		printf("Wrong response from the arduino\n");
		return -1;
	}
	return 0;
}

int goalUp(){
	serialPutchar(fd, (unsigned char) 248);

	int response = serialGetchar(fd);

	if (response < 0){
		printf("No response from the arduino\n");
		return -1;
	}
	else if(response != 248){
		printf("Wrong response from the arduino\n");
		return -1;
	}
	return 0;
}

int goalDown(){
	serialPutchar(fd, (unsigned char) 249);

	int response = serialGetchar(fd);

	if (response < 0){
		printf("No response from the arduino\n");
		return -1;
	}
	else if(response != 249){
		printf("Wrong response from the arduino\n");
		return -1;
	}
	return 0;
}

int yarmUp(){
	serialPutchar(fd, (unsigned char) 250);

	int response = serialGetchar(fd);

	if (response < 0){
		printf("No response from the arduino\n");
		return -1;
	}
	else if(response != 250){
		printf("Wrong response from the arduino\n");
		return -1;
	}
	return 0;
}

int yarmDown(){
	serialPutchar(fd, (unsigned char) 251);

	int response = serialGetchar(fd);

	if (response < 0){
		printf("No response from the arduino\n");
		return -1;
	}
	else if(response != 251){
		printf("Wrong response from the arduino\n");
		return -1;
	}
	return 0;
}

int probesUp(){
	serialPutchar(fd, (unsigned char) 252);

	int response = serialGetchar(fd);

	if (response < 0){
		printf("No response from the arduino\n");
		return -1;
	}
	else if(response != 252){
		printf("Wrong response from the arduino\n");
		return -1;
	}
	return 0;
}

int probesDown(){
	serialPutchar(fd, (unsigned char) 253);

	int response = serialGetchar(fd);

	if (response < 0){
		printf("No response from the arduino\n");
		return -1;
	}
	else if(response != 253){
		printf("Wrong response from the arduino\n");
		return -1;
	}
	return 0;
}

int gripperClose(){
	serialPutchar(fd, (unsigned char) 254);

	int response = serialGetchar(fd);

	if (response < 0){
		printf("No response from the arduino\n");
		return -1;
	}
	else if(response != 254){
		printf("Wrong response from the arduino\n");
		return -1;
	}
	return 0;
}

int gripperOpen(){
	serialPutchar(fd, (unsigned char) 255);

	int response = serialGetchar(fd);

	if (response < 0){
		printf("No response from the arduino\n");
		return -1;
	}
	else if(response != 255){
		printf("Wrong response from the arduino\n");
		return -1;
	}
	return 0;
}

int readResistance(){
	serialPutchar(fd, (unsigned char) 245);

	int received = serialGetchar(fd);

	if(received < 0){
		printf("No response from the arduino\n");
		return -1;
	}
	else{
		float resistance = (float) received;

	    float result = (resistance/255)*10000;

	    return((int) result);
	}
}



