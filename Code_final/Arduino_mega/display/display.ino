#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>


//Serial
int incomingByte = 0;

//Display 
#define HARDWARE_TYPE MD_MAX72XX::ICSTATION_HW
#define MAX_DEVICES 4
#define CLK_PIN   11
#define DATA_PIN  13
#define CS_PIN    12
MD_Parola scoreDisplay = MD_Parola(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);
String displayString;

void setup(void)
{
  //Serial
  Serial.begin(9600); 
  
  //Display
  scoreDisplay.begin();
  scoreDisplay.displayClear();
  scoreDisplay.setTextAlignment(PA_CENTER);
}

void loop() {

  if(Serial.available() > 0){

    incomingByte = Serial.read();
    
    if((int) incomingByte < 200){
      displayScore((int) incomingByte);
      Serial.write(1); //send confirmation of action
    }
  }
}

int displayScore(int score){
  displayString = String(score) + "pts";
  scoreDisplay.print(displayString);
  return 0;
}
