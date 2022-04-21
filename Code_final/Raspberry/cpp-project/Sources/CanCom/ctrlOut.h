/*!
 * \file ctrlOut.h
 * \brief Structure defining the outputs of the robot controller
 */




#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

typedef struct ctrlOut
{
    float M1; ///< Command [-] of the right wheel : bounded in [-35 ; 35]
    float M2; ///< Command [-] of the left wheel : bounded in [-35 ; 35]

    //CAN can; ///< class to control motors with CAN bus
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

} ctrlOut;

ctrlOut * ctrlOut_init();
void motors_init(ctrlOut *outputs);
void send_commands(ctrlOut *outputs);
void motors_stop(ctrlOut *outputs);
void can_free(ctrlOut *outputs);

int toHexadecimal(double dutyCycle, char* speed) ; 
double thetaToCan(float theta) ; 
void sendTheta(float theta, int motor) ; 
void CAN_init(ctrlOut *outputs) ; 

