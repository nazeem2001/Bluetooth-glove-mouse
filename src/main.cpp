#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <BleCombo.h>
#include<BleComboMouse.h>
#define BUTTON1 2
#define BUTTON2 15
#define BUTTON3 13
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t pax=0, pay=0, paz=0;
int16_t gx, gy, gz;



#define OUTPUT_READABLE_ACCELGYRO
#include <BleCombo.h>
#define LED_PIN 22


bool blinkState = false;
bool b1V=false,b2V=false,b3V=false;
bool b1intruptAct=true,b2intruptAct=true,b3intruptAct=true;



bool leftprevious=true, rightprevious=true, middleprevious=true;
bool leftpresent,rightpresent,middlepresent;






void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(4,0,uint32_t(400000));
    
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
     Keyboard.begin();
     Mouse.begin();
    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);
    
    
    pinMode(BUTTON1,INPUT_PULLUP);
    pinMode(BUTTON2,INPUT_PULLUP);
    pinMode(BUTTON3,INPUT_PULLUP);
 

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values
    
    Serial.println("Updating internal sensor offsets...");
   
  Serial.println("Starting work!");

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
}
int ox,oy;
void loop() {
    if(Keyboard.isConnected()){
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        int down=(int)map(ay,-32768,32768,-2,2);
        if(down!=1){
            leftpresent = digitalRead(BUTTON1);
            rightpresent = digitalRead(BUTTON2);
            middlepresent = digitalRead(BUTTON3);
            if(leftprevious!=leftpresent)
            {   
                leftprevious=leftpresent;
                delay(100);
                if(leftpresent==false)
                {
                    Mouse.press(MOUSE_LEFT);
                }
                else
                {
                    Mouse.release(MOUSE_LEFT);
                }
            }
            if(rightprevious!=rightpresent)
            {
                rightprevious=rightpresent;
                delay(100);
                if(rightpresent==false)
                {
                    Mouse.press(MOUSE_RIGHT);
                }
                else
                {
                    Mouse.release(MOUSE_RIGHT);
                }
            }
            if(middleprevious!=middlepresent)
            {
                middleprevious=middlepresent;
                delay(100);
                if(middlepresent==false)
                {
                    Mouse.press(MOUSE_MIDDLE);
                }
                else
                {
                    Mouse.release(MOUSE_MIDDLE);
                }
            }


            // read raw accel/gyro measurements from device
            
            //gy=gy/200;
            //gx=gx/200;
            //ox=wrappedDelta(pax, gx);
            //oy=wrappedDelta(pay, gy); 
        // if(((-1000>gx)|(gx>1000))&((-1000>gz)|(gz>1000))){
            Mouse.move(-gz/1000,gx/1000);
            pax=gx;
            pay=gy;
            //}
            // these methods (and a few others) are also available
            //accelgyro.getAcceleration(&ax, &ay, &az);
            blinkState = !blinkState;
            digitalWrite(LED_PIN, blinkState);
            
            Serial.print(gz);Serial.print("\t");Serial.println(gy);
            // if (b1intruptAct==false){
            // delay(70);
            // b1intruptAct=true;Serial.print("wjdjlkjj\n");
            // attachInterrupt(digitalPinToInterrupt(BUTTON1),B1_isr,CHANGE);      
            // }
            // if (b2intruptAct==false){
            // delay(70);
            // b2intruptAct=true;Serial.print("wjdjlkjj\n");
            // attachInterrupt(digitalPinToInterrupt(BUTTON2),B2_isr,CHANGE);      
            // }
            // if (b3intruptAct==false){
            // delay(70);
            // b3intruptAct=true;Serial.print("wjdjlkjj\n");
            // attachInterrupt(digitalPinToInterrupt(BUTTON3),B3_isr,CHANGE);      
            // }
        }
    }
    else 
        delay (2000);Serial.print("wjdj\n");
    
}
