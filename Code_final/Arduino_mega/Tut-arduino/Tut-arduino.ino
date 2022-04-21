#include "Arduino.h"
#include <AX12A.h>
#include <Servo.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>

//off and on definition removed from AX12
//OFF = 0 and ON = 1

//Serial
int incomingByte = 0;

//Display 
#define HARDWARE_TYPE MD_MAX72XX::ICSTATION_HW
#define MAX_DEVICES 4
#define CLK_PIN   10
#define CS_PIN    9
#define DATA_PIN  8

MD_Parola scoreDisplay = MD_Parola(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);
String displayString;

//Dynamixels
#define DirectionPin   (10u)
#define BaudRate      (57142ul)

//Servos
Servo gripper; //pin 4
Servo probes; //pin 7
Servo goal;   //pin 11
Servo yarm;  //pin 12 

//ADC
float analog_value = 0.0;
float vout = 0.0;
float rmeas = 0.0;


void setup(void)
{
  //Serial
  Serial.begin(9600); 
  
  //Display
  scoreDisplay.begin();
  scoreDisplay.displayClear();
  scoreDisplay.setTextAlignment(PA_CENTER);

  //Dynamixels
  ax12a.begin(BaudRate, DirectionPin, &Serial3);
  ax12a.setEndless(9u, 1);                          //Dynamixel carré de fouille
  ax12a.setEndless(2u, 1);                          //Dynamixel réplique

  //Servos
  gripper.attach(4);
  probes.attach(7);
  goal.attach(11);
  yarm.attach(12);

  gripperClose();
  probesUp();
  goalUp();
  yarmUp();
   
}

void loop() {

  if(Serial.available() > 0){

    incomingByte = Serial.read();
    
    if((int) incomingByte < 200){
      Serial.write(1); //send confirmation of reception
      displayScore((int) incomingByte);
    }

    else if((int) incomingByte == 246){
      Serial.write(246);
      pushReplica();    
    }

    else if((int) incomingByte == 247){
      Serial.write(247);
      pushExcavationSquare();
    }

    else if((int) incomingByte == 248){
      Serial.write(248);
      goalUp();
    }

    else if((int) incomingByte == 249){
      Serial.write(249);
      goalDown();    
    }

    else if((int) incomingByte == 250){
      Serial.write(250);
      yarmUp();     
    }

    else if((int) incomingByte == 251){
      Serial.write(251);
      yarmDown();      
    }

    else if((int) incomingByte == 252){
      Serial.write(252);
      probesUp();
    }

    else if((int) incomingByte == 253){
      Serial.write(253);
      probesDown();            
    }

    else if((int) incomingByte == 254){
      Serial.write(254);
      gripperClose();      
    }

    else if((int) incomingByte == 255){
      Serial.write(255);
      gripperOpen();       
    }

    else if((int) incomingByte == 245){
      float resistorValue = readResistor();
      float toSend = (resistorValue/10000) * 255;
      Serial.write((int) toSend);
    }
  }
}

int displayScore(int score){
  displayString = String(score) + "pts";
  scoreDisplay.print(displayString);
  return 0;
}

int pushReplica(){ //246  //vitesses et timings à définir

  ax12a.turn(2u, RIGHT, 600);
  delay(750);
  ax12a.turn(2u, RIGHT, 0);
  delay(200);
  ax12a.turn(2u, LEFT, 600);
  delay(765);
  ax12a.turn(2u, LEFT, 0);
}

int pushExcavationSquare(){  //247  //check si on peut s'en sortir avec des positions (non ducoup)

  ax12a.turn(9u, RIGHT,800);
  delay(1050);
  ax12a.turn(9u, RIGHT, 0);
  delay(200);
  ax12a.turn(9u, LEFT, 800);
  delay(1090);
  ax12a.turn(9u, LEFT, 0);
}


int goalUp(){  //248
  goal.write(88);
  return 0;
}


int goalDown(){  //249
  goal.write(15);
  return 0;
}


int yarmUp(){  //250
  yarm.write(180);
  return 0;
}


int yarmDown(){ //251
  yarm.write(110);
  return 0;
}


int probesUp(){  //252
  probes.write(175);
  return 0;
}


int probesDown(){  //253
  probes.write(130);
  return 0;
}


int gripperClose(){  //254
  gripper.write(30);
  return 0;

}


int gripperOpen(){  //255
  gripper.write(170);
  return 0;
}

float readResistor(){ //245
  
   analog_value = analogRead(A7);
   vout = (analog_value * 5) / 1023;

   if(vout > 4.9){
    rmeas = 10000;
   }
   else{
    rmeas = (vout*1180) / (5 - vout);
   }

   return rmeas;
}
