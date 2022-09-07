#define BUTTON1 2       // Assigns Button1 to pin 2 of ESP32.
#define BUTTON2 15      // Assigns Button2 to pin 15 of ESP32.
#define BUTTON3 13      // Assigns Button3 to pin 13 of ESP32.
#include"Arduino.h"

void setup() {
  // put your setup code here, to run once:
    pinMode(BUTTON1,INPUT_PULLUP);   
    pinMode(BUTTON2,INPUT_PULLUP);
    pinMode(BUTTON3,INPUT_PULLUP);    // Making the mouse buttons to be in PULLUP state.
    Serial.begin(9600);
}

void loop() {
  Serial.print(digitalRead(BUTTON1));
  Serial.print(digitalRead(BUTTON2));
  Serial.println(digitalRead(BUTTON3));
  // put your main code here, to run repeatedly:

}