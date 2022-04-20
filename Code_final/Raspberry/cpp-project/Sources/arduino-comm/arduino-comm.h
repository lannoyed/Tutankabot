#include <stdio.h>
#include <string.h>
#include <wiringSerial.h>

int arduinoSerialDisconnect();

int arduinoSerialConnect();

int arduinoSerialDisconnect();

void sendSerialChar(unsigned char c);

int sendScore(unsigned char score);

int pushReplica();

int pushExcavationSquare();

int goalUp();

int goalDown();

int yarmUp();

int yarmDown();

int probesUp();

int probesDown();

int gripperClose();

int gripperOpen();

int readResistance();