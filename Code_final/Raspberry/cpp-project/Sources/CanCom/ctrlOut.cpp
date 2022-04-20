/*!
 * \file ctrlOut.cpp
 * \brief Structures defining the outputs of the robot controller
 *
 * CAN Source for communication with socketCAN
 * https://github.com/craigpeacock/CAN-Examples/blob/master/cantransmit.c
 */
 
#include "ctrlOut.h"

ctrlOut * ctrlOut_init()
{
    ctrlOut *outputs = (ctrlOut*) malloc(sizeof(ctrlOut)) ; 
    outputs->M1 = 0.0;
    outputs->M2 = 0.0;

    //outputs->can.init();
    CAN_init(outputs);
    motors_init(outputs);
    return outputs; 
}

void CAN_init(ctrlOut *outputs)
{
    system("sudo ifconfig can0 down");
    system("sudo ip link set can0 type can bitrate 125000");
    system("sudo ifconfig can0 up");
    usleep(1000);

    if ((outputs->s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Socket");
    }
    strcpy(outputs->ifr.ifr_name, "can0");

    if (ioctl(outputs->s, SIOCGIFINDEX, &(outputs->ifr)) < 0) {
        perror("ioctl");
    }
    memset(&(outputs->addr), 0, sizeof(outputs->addr));
    outputs->addr.can_family = AF_CAN;
    outputs->addr.can_ifindex = outputs->ifr.ifr_ifindex;
    if (bind(outputs->s, (struct sockaddr *)&(outputs->addr), sizeof(outputs->addr)) < 0) {
        perror("Bind");
    }
    outputs->frame.can_id = 0x708;
    outputs->frame.can_dlc = 3; // 3 bytes transmitted
}


void motors_init(ctrlOut *outputs)
{
    outputs->frame.data[0] = 0x1E;
    outputs->frame.data[1] = 0xFF;
    outputs->frame.data[2] = 0x40;

    if (write(outputs->s, &(outputs->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write LED ON");
    }
    printf("Write OK : LED ON\n");

    outputs->frame.data[0] = 0x1E;
    outputs->frame.data[1] = 0xFF;
    outputs->frame.data[2] = 0x00;

    if (write(outputs->s, &(outputs->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write LED OFF");
    }
    printf("Write OK : LED OFF\n");

    outputs->frame.data[0] = 0x1C;
    outputs->frame.data[1] = 0xFF;
    outputs->frame.data[2] = 0x80;

    if (write(outputs->s, &(outputs->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write INIT M_1");
    }
    printf("Write OK : INIT M_1\n");

    outputs->frame.data[0] = 0x1D;
    outputs->frame.data[1] = 0xFF;
    outputs->frame.data[2] = 0x80;

    if (write(outputs->s, &(outputs->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write INIT M_2");
    }
    printf("Write OK : INIT M_2\n");
}



void send_commands(ctrlOut *outputs)
{
    int M1 = thetaToCan(-outputs->M1) ;
    int M2 = thetaToCan(outputs->M2);

    //std::cout << "Motor conversion \n";
    
    unsigned char M1_hex = (unsigned char) M1;
    unsigned char M2_hex = (unsigned char) M2;
    
    //std::cout << "To hex \n";

    outputs->frame.data[0] = 0x25;
    outputs->frame.data[1] = 0xFF;
    outputs->frame.data[2] = M1_hex;

    if (write(outputs->s, &(outputs->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write M1");
    }
    
    //std::cout << "M1 \n";
    
    //std::cout<< M1 <<" hex " << "\n";
    //std::cout<< M2 <<" hex " << "\n";

    outputs->frame.data[0] = 0x26;
    outputs->frame.data[1] = 0xFF;
    outputs->frame.data[2] = M2_hex;

    if (write(outputs->s, &(outputs->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write M2");
    }
    //std::cout << "M2 \n";
}


void motors_stop(ctrlOut *outputs)
{
    outputs->frame.data[0] = 0x25;
    outputs->frame.data[1] = 0xFF;
    outputs->frame.data[2] = 0x22;

    if (write(outputs->s, &(outputs->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write STOP M_1");
    }

    outputs->frame.data[0] = 0x26;
    outputs->frame.data[1] = 0xFF;
    outputs->frame.data[2] = 0x22;

    if (write(outputs->s, &(outputs->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write STOP M_2");
    }
}

void can_free(ctrlOut *outputs)
{
    if (close(outputs->s) < 0) {
        perror("Close");
    }
}




// venant de l ancien module
int toHexadecimal(double dutyCycle, char* speed){
  /* Pré : 
     dutyCycle : un double variant de 0 à 100 
     speed : pointeur de type char [2]
     Post: renvoie la valeur conrrespondante en hexadécimal sous forme de string à l'adresse speed
	
    commentaire : long mais éfficace attention des modules suplémentaires sont nécéssaire pour enable les nombres à virgules 
    ou convertir des valeurs suplémentaires à 127
  */
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

double thetaToCan(float theta){
	double a = (100+theta)/2.0 ;  
	return a*0.68 ; 
}


void sendTheta(float theta, int motor){
	// Theta est la commande de la roue et peut aller de -100 à 100 
	
	char command[50] ; 
	int thetaCan ; 
	if (motor == 1 ) {	
		thetaCan = thetaToCan(-theta) ;
	} else{ 
		thetaCan = thetaToCan(theta) ;
	}
	char theta_string[2] ; 
	toHexadecimal(thetaCan, theta_string) ; 
	if (motor == 1){
 		strcpy(command, "cansend can0 708#25FFXX") ; // Template de commande pour le moteur 1 
	} else if (motor = 2){
		strcpy(command, "cansend can0 708#26FFXX") ; // Template de commande pour le moteur 2
	}
	command[21] = theta_string[0] ; 
	command[22] = theta_string[1] ; 
	system(command) ; 
}

