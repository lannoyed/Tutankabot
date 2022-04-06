#include <FastLED.h>

#define NUM_LEDS 24
#define DATA_PIN 3
#define SENSORPIN 4

int sensorState = 0;
unsigned long lastFree;
unsigned long present;
int on = 0;

CRGB leds[NUM_LEDS];

void setup() { 

    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

    pinMode(SENSORPIN, INPUT);
    digitalWrite(SENSORPIN, HIGH); //pullup (HIGH = OFF)

    Serial.begin(9600);
}

void loop() {

  sensorState = digitalRead(SENSORPIN);

  if (sensorState == HIGH) {
    lastFree = millis();
    //Serial.print("Lastfree: ");
    //Serial.println(lastFree);
    for(int i = 0; i < 24; i++){
      leds[i] = CRGB::Black;
    }
    FastLED.show();
    on = 0;
  } 
  else {
    present = millis();
    //Serial.print("Present: ");
    //Serial.println(present);
    //Serial.print("Lastfree: ");
    //Serial.println(lastFree);
    //Serial.println(present - lastFree);
    
    if (present - lastFree > 1500){
      for(int i = 0; i < 12; i++){
        leds[11-i] = CRGB::White;
        leds[12+i] = CRGB::White;
        FastLED.show();
        delay(50);
        }

      if(on == 0){
        for(int j = 0; j < 5; j++){
          for(int i = 0; i < 24; i++){
            leds[i] = CRGB::Black;
            }
          FastLED.show();
          delay(100);

          for(int i = 0; i < 24; i++){
            leds[i] = CRGB::White;
            }
          FastLED.show();
          delay(100);
        }
        on  = 1;
      }
    }
  }
}
