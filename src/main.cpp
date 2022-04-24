#include "I2Cdev.h"   //This library makes it easy to connect with I2C devices like ESP32/arduino without having to write any code.
#include "MPU6050.h"  //6-axis accelerometer or gyroscope library allows us to easily get the 6-D motion of the MPU6050.
#include "Wire.h"     //Allows to communicate with I2C devices (consists of some advanced functions)
#include <BleCombo.h> //Allows us to make the ESP32 act as bluetooth keyboard and mouse.
#include "esp_sleep.h" //  Allows to make ESP32 to hold values in pins when CPU is powered down.
#define ESP_DRD_USE_LITTLEFS    true   // Activates LITTLEFS file system in library DOUBLERESETDETECTOR.h.
#define ESP_DRD_USE_SPIFFS      false  //Deactivates SPIFFS file system in library DOUBLERESETDETECTOR.h.
#define ESP_DRD_USE_EEPROM      false  //Deactivates SEP_DRD_USE_EEPROM in library DOUBLERESETDETECTOR.h.

#define DOUBLERESETDETECTOR_DEBUG      true  //Builds the code in DEBUG mode.

#include <ESP_DoubleResetDetector.h>    // Library used to detect doublereset.

#include<Pangodream_18650_CL.h>      // Library used to convert battery votage to percentage.

#define DRD_TIMEOUT 2     // Number of seconds after reset during which a subseqent reset will be considered a double reset.

#define DRD_ADDRESS 0     // RTC Memory Address for the DoubleResetDetector to use

DoubleResetDetector* drd;   //Pointer to class DoubleResetDetector

#define BUTTON1 2       // Assigns Button1 to pin 2 of ESP32.
#define BUTTON2 15      // Assigns Button2 to pin 15 of ESP32.
#define BUTTON3 13      // Assigns Button3 to pin 13 of ESP32.
#define SCL 0           // Replaces with 0 where ever SCL is present.
#define SDA 4            // Replaces with 4 where ever SDA is present.
MPU6050 accelgyro;       //Creates Object accelgyro to class MPU6050.
Pangodream_18650_CL BL;  //Creates Object BL to class Pangodream_18650_CL.

int16_t ax, ay, az;     //Creating variables of 16 bit int data type to read 3-D accelerometer values.

int16_t gx, gy, gz;     //Creating variables of 16 bit int data type to read 3-D gyroscope values.


int32_t timeOut;
bool downD=true,upD=false;
int num=0;

#define LED_PIN 22     // Replaces with 22 whereever LED_PIN is present
#define mpu1 17        // Replaces with 17 whereever mpu1 is present


bool blinkState = false;    // variable which represents the blinkstate.
bool leftprevious, rightprevious, middleprevious;   //variables whichstores the previous states of mouse buttons.
bool leftpresent,rightpresent,middlepresent;        //variables whichstores the present states of mouse buttons.

void setup() {
    #if (DOUBLERESETDETECTOR_DEBUG)
        Serial.begin(38400);
        // initialize serial communication
        // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
        // it's really up to you depending on your project)
    #endif
    drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);  //Assigns the converted DRD_TIMEOUT and DRD_ADDRESS to the drd object.
    pinMode(mpu1,OUTPUT);                                     // Assigns output to PIN 17.
    digitalWrite(mpu1,0);                                     //Turns off the MPU6050.
    if (drd->detectDoubleReset()==false)                      // Checks whether has occured or not.
    {
        gpio_hold_en(GPIO_NUM_17);
        gpio_deep_sleep_hold_en();                            // Enables ESP32 to hold the values of PIN 17 when ESP32 is in sleep mode.
        delay(2000);                                          // Introducing delay equal to timeout.
          drd->loop();                                        // Clears the flag when time laps more than TIMEOUT
        esp_deep_sleep_start();                               // ESP32 is made to sleep.
    }
    digitalWrite(mpu1,1);                                     // TURNS ON MPU6050.
    delay(1000);                                              // DELAY is introduced to stabilize the MPU6050.
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin(SDA,SCL,uint32_t(400000));                     // Starts Communication between MPU6050 and ESP32.
    
    Keyboard.begin();
    Mouse.begin();                                            // Makes ESP32 visible as bluetooth Keyboard and Mouse to near by devices.
    #if (DOUBLERESETDETECTOR_DEBUG)
        Serial.print("Charge level: ");
        Serial.println(BL.getBatteryChargeLevel());
    #endif
    Keyboard.setBatteryLevel(BL.getBatteryChargeLevel());
    //First converts the battery voltage level to percentage and then it is made to visible on screen as percentage
    
    
    
    
    pinMode(BUTTON1,INPUT_PULLUP);   
    pinMode(BUTTON2,INPUT_PULLUP);
    pinMode(BUTTON3,INPUT_PULLUP);    // Making the mouse buttons to be in PULLUP state.
    leftprevious = digitalRead(BUTTON1);
    rightprevious = digitalRead(BUTTON2);
    middleprevious = digitalRead(BUTTON3);  //reading the buttons state and assigning it to previous state.
            

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
    if(Keyboard.isConnected())    //Checks the connectivity of Keyboard.
    {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);    //Reads all the 6-D motions of MPU6050.
        int down=(int)map(ay,-32768,32768,-4,4);               //Maps the ay value to the range -2 to 2.
        if(down>=-1)                                           //Checks whether the mouse is in rest state or not.
        {
            leftpresent = digitalRead(BUTTON1);         
            rightpresent = digitalRead(BUTTON2);
            middlepresent = digitalRead(BUTTON3);      // Reads the state of mouse buttons and assigns the to present state.
            if(leftprevious!=leftpresent)              // Checks wheteher both states are same or not executes if there is a state change.
            {   
                Keyboard.setBatteryLevel(BL.getBatteryChargeLevel());  //Updates the battery level of the mouse.
                leftprevious=leftpresent;                              // Assigns present state to previous state.
                delay(100);                                            //Introducing delay to avoid the noise due to switch debouncing.
                if(leftpresent==false)                                 // Executes when the button is pressed.
                {
                    Mouse.press(MOUSE_LEFT);                           // Performs left click action.
                }
                else
                {
                    Mouse.release(MOUSE_LEFT);                         // Releases the button. 
                }
            }
            if(rightprevious!=rightpresent)            // Checks wheteher both states are same or not executes if there is a state change.
            {
                Keyboard.setBatteryLevel(BL.getBatteryChargeLevel());   //Updates the battery level of the mouse.
                rightprevious=rightpresent;                             // Assigns present state to previous state.
                delay(100);                                             //Introducing delay to avoid the noise due to switch debouncing.
                if(rightpresent==false)                                 // Executes when the button is pressed.
                {
                    Mouse.press(MOUSE_RIGHT);                           // Performs Right click action.   
                }
                else
                {
                    Mouse.release(MOUSE_RIGHT);                         // Releases the button. 
                }
            }
            if(middleprevious!=middlepresent)            // Checks wheteher both states are same or not executes if there is a state change.      
            {
                Keyboard.setBatteryLevel(BL.getBatteryChargeLevel());      //Updates the battery level of the mouse.
                middleprevious=middlepresent;                               // Assigns present state to previous state.
                delay(100);                                                //Introducing delay to avoid the switch debouncing.
                if(middlepresent==false)                                    // Executes when the button is pressed.
                {
                    Mouse.press(MOUSE_MIDDLE);                               // Performs Right click action.
                }
                else
                {
                    Mouse.release(MOUSE_MIDDLE);                             // Releases the button. 
                }
            }
            if (down>=1){
                if (downD){
                downD=false;
                upD=true;
                num++;
                }
            }
            if(num>1){
                if(timeOut<millis()){
                    //Keyboard.print(num);
                    if (num==2){
                        Keyboard.press(KEY_LEFT_GUI);
                        Keyboard.release(KEY_LEFT_GUI);
                        delay(700);
                        Keyboard.print("chrome");
                        delay(700);
                        Keyboard.write(KEY_RETURN);
                        delay(2000);
                        Keyboard.print("https://www.youtube.com/");
                    }
                    if (num==3){
                        Keyboard.press(KEY_LEFT_GUI);
                        Keyboard.release(KEY_LEFT_GUI);
                        delay(700);
                        Keyboard.print("chrome");
                        delay(700);
                        Keyboard.write(KEY_RETURN);
                        delay(2000);
                        Keyboard.print("https://www.google.co.in/");
                    }
                    if (num==4){
                        Keyboard.press(KEY_LEFT_GUI);
                        Keyboard.release(KEY_LEFT_GUI);
                        delay(700);
                        Keyboard.print("chrome");
                        delay(700);
                        Keyboard.write(KEY_RETURN);
                        delay(2000);
                        Keyboard.print("https://web.whatsapp.com/");
                    }
                    delay(1000);
                    Keyboard.write(KEY_RETURN);
                        
                    num=0;
                    upD=false;
                }
            }
   
            
            Mouse.move(-gz/800,-gx/800);             //cursor moves with resolution of -gz/1000 and -gx/1000 on x-axis and y-axis respectively
            blinkState = !blinkState;                 // changes  blinksate
            digitalWrite(LED_PIN, blinkState);        // LED_PIN blinks LED based on blink state.
            #if (DOUBLERESETDETECTOR_DEBUG)
            Serial.print(gz);Serial.print("\t");Serial.println(gy);
            #endif
        }
        else{
          if (down<-1){
            if (upD){
              if(timeOut>millis()){
                downD=true;
              }
              else{
                timeOut=millis()+5000;
                downD=true;
                num=0;
              }
            }
            else{
              downD=true;
              timeOut=millis()+5000;

            }
          }
        }
        
    }
    else {
        delay (2000);
        #if (DOUBLERESETDETECTOR_DEBUG)
            Serial.print("Host not found\n");
        #endif
    }
    drd->loop();          // Clears the flag in when time laps more than TIMEOUT.
}