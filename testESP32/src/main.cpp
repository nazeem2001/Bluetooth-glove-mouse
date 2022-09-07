/**
 * This example turns the ESP32 into a Bluetooth LE keyboard and mouse that writes 
*/
 #include"Arduino.h"
#include <BleCombo.h>

void setup() {
  Serial.begin(115200);
  Serial.println("Starting work!");
  Keyboard.begin();
  Mouse.begin();
}

void loop() {
  if(Keyboard.isConnected()) {
    Serial.println("Sending 'Hello world'");
    Keyboard.println("Hello World");

    
    unsigned long startTime;

    Serial.println("Move mouse pointer up");
    startTime = millis();
    while(millis()<startTime+1000) {
      Mouse.move(0,-1);
      delay(5);
    }
    Serial.println("Move mouse pointer left");
    startTime = millis();
    while(millis()<startTime+1000) {
      Mouse.move(-1,0);
      delay(5);
    }

    Serial.println("Move mouse pointer down");
    startTime = millis();
    while(millis()<startTime+1000) {
      Mouse.move(0,1);
      delay(5);
    }

    Serial.println("Move mouse pointer right");
    startTime = millis();
    while(millis()<startTime+1000) {
      Mouse.move(1,0);
      delay(5);
    }
    
    
  }
  
  Serial.println("Waiting 2 seconds...");
  delay(2000);
}
