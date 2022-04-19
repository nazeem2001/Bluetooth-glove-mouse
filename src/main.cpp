#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <BleCombo.h>
#include "esp_sleep.h"
#define ESP_DRD_USE_LITTLEFS    true
#define ESP_DRD_USE_SPIFFS      false
#define ESP_DRD_USE_EEPROM      false

#define DOUBLERESETDETECTOR_DEBUG      true

#include <ESP_DoubleResetDetector.h>

#include<Pangodream_18650_CL.h>
// Number of seconds after reset during which a 
// subseqent reset will be considered a double reset.
#define DRD_TIMEOUT 2

// RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS 0

DoubleResetDetector* drd;

#define BUTTON1 2
#define BUTTON2 15
#define BUTTON3 13
#define SCL 0
#define SDA 4
MPU6050 accelgyro;
Pangodream_18650_CL BL;

int16_t ax, ay, az;

int16_t gx, gy, gz;




#define LED_PIN 22
#define mpu1 17


bool blinkState = false;
bool leftprevious, rightprevious, middleprevious;
bool leftpresent,rightpresent,middlepresent;

void setup() {
    #if (DOUBLERESETDETECTOR_DEBUG)
        Serial.begin(38400);
    #endif
    drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);
    pinMode(mpu1,OUTPUT);
    digitalWrite(mpu1,0);
    if (drd->detectDoubleReset()==false) 
    {
        gpio_hold_en(GPIO_NUM_17);
        gpio_deep_sleep_hold_en();
        delay(2000);
          drd->loop();
        esp_deep_sleep_start();
    }
    digitalWrite(mpu1,1);
    delay(1000);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin(SDA,SCL,uint32_t(400000));
    
    Keyboard.begin();
    Mouse.begin();
    #if (DOUBLERESETDETECTOR_DEBUG)
        Serial.print("Charge level: "+BL.getBatteryChargeLevel());
    #endif
    Keyboard.setBatteryLevel(BL.getBatteryChargeLevel());
    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    
    
    
    pinMode(BUTTON1,INPUT_PULLUP);
    pinMode(BUTTON2,INPUT_PULLUP);
    pinMode(BUTTON3,INPUT_PULLUP);
    leftprevious = digitalRead(BUTTON1);
    rightprevious = digitalRead(BUTTON2);
    middleprevious = digitalRead(BUTTON3);
            

    // initialize device
    #if (DOUBLERESETDETECTOR_DEBUG)
        Serial.println("Initializing I2C devices...");
    #endif
    accelgyro.initialize();

    // verify connection
    #if (DOUBLERESETDETECTOR_DEBUG)
        Serial.println("Testing device connections...");
        Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    
    // use the code below to change accel/gyro offset values
    
        Serial.println("Updating internal sensor offsets...");
   
        Serial.println("Starting work!");
    #endif
    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
}
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
                Keyboard.setBatteryLevel(BL.getBatteryChargeLevel());
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
                Keyboard.setBatteryLevel(BL.getBatteryChargeLevel());
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
                Keyboard.setBatteryLevel(BL.getBatteryChargeLevel());
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


            
            Mouse.move(-gz/1000,gx/1000); 
            //cursor moves with resolution of -gz/1000 and gx/1000 on x-axis and y-axis respectively
            blinkState = !blinkState;
            digitalWrite(LED_PIN, blinkState);
            #if (DOUBLERESETDETECTOR_DEBUG)
            Serial.print(gz);Serial.print("\t");Serial.println(gy);
            #endif
        }
        
    }
    else {
        delay (2000);
        #if (DOUBLERESETDETECTOR_DEBUG)
            Serial.print("Host not found\n");
        #endif
    }
    drd->loop();
}
