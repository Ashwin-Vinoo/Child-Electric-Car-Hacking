/* Social Mobility Lab - Powered Cart Code
 * 
 * Description: This code was created to automate button press data logging
 * on micro SD cards so that researchers in the lab can study how effectively
 * infants learn to use the carts under varying external stimuli.
 * 
 * Author: Ashwin Vinoo 
 * Date: 1/24/2020
 * 
 * Default Circuit Wiring (Arduino Uno):
 * 
 * SD Card CS - Pin 8
 * SD Card SCK - Pin 13
 * SD Card MOSI - Pin 11
 * SD Card MISO - Pin 12
 * SD Card VCC - 5V
 * SD Card GND - GND
 * 
 * RTC SCL - SCL or Pin A5
 * RTC SDA - SDA or Pin A4
 * RTC VCC - 5V
 * RTC GND - GND
 * 
 * Motor Driver +12 (Also 6V Battery +) - Not Connected
 * Motor Driver +5V (Power Supply) - VIN
 * Motor Driver GND (Also 6V Battery -) - GND
 * Motor Driver IN1 - Pin 10 (pwm)
 * Motor Driver IN2 - Pin 9 (pwm)
 * Motor Driver IN3 - Not Connected
 * Motor Driver IN4 - Not Connected
 * Motor Driver OUT1 (Also Motor +) - Not Connected
 * Motor Driver OUT2 (Also Motor -) - Not Connected
 * Motor Driver OUT3 - Not Connected
 * Motor Driver OUT4 - Not Connected 
 * 
 * Push Button Wire 1 - Pin 4
 * Push Button Wire 2 - Pin 2
 * 
 * Potentiometer End Pin One - ICSP 5V
 * Potentiometer End Pin Two - ICSP GND
 * Potentiometer Middle Pin - Pin A0
 * 
 * Common Cathode LED (Common Cathode) - Pin 7
 * Common Cathode LED (Red Anode) - Pin 3
 * Common Cathode LED (Green Anode) - Pin 5
 * 
 * Battery Level RC Network Middle - Pin A2
 * Battery Level RC Network +6V (Also 6V Battery +) - Not Connected
 * Battery Level RC Network GND (Also 6V Battery -) - Not Connected
 * 
 */

//*************** IMPORTING THE NECESSARY MODULES ****************

// We import all the necessary libraries
#include <SPI.h>
#include <SD.h>
#include "RTClib.h"

//*************** DEFINING THE PINS TO BE USED ****************

// Helps select the active SPI interface to communicate through if multiple SPI devices are available
const byte sd_chip_select_pin = 8;
// Helps send a ground signal through wire 1 of the push button
const byte push_button_write_pin = 4;
// Helps read the signal coming from wire 2 of the push button
const byte push_button_read_pin = 2;
// The motor driver input pin one (two pins control one motor)
const byte motor_driver_input_1_pin = 10;
// The motor driver input pin two (two pins control one motor)
const byte motor_driver_input_2_pin = 9;
// The analog read pin to be used to read the potentiometer values for regulating motor speed
const byte motor_speed_control_pin = 0;
// The analog read pin to be used to read the battery Level
const byte battery_level_pin = 2;
// The common cathode LED green lighting anode pin
const byte led_common_cathode_pin = 7;
// The common cathode LED red lighting anode pin
const byte led_red_anode_pin = 3;
// The common cathode LED green lighting anode pin
const byte led_green_anode_pin = 5;

//*************** HYPERPARAMETERS ****************

// The name of the text file to which we store the button press data (Must be 8.3 format)
String sd_data_file_name = "CartData.txt";
// Whether we should append data to the end of the previous file or clear its contents and start from scratch
bool sd_data_file_append = true;
// Set RTC timestamp as compile time
bool rtc_set_compile_time = false;

//*************** GLOBAL VARIABLES ****************

// The object for the RTC DS3231 module is created
RTC_DS3231 rtc;
// Whether the push button is released
bool push_button_release = true;
// Helps track the duration of the button presses
unsigned long button_press_timer;

//*************** SETTING UP DEVICES AND INTERFACES ****************

// Setup is executed only once
void setup() 
{
  // Open serial communications - Helps debug code execution:
  Serial.begin(115200);

  //*****PIN SPECIFICATIONS*****
  // We specify that the push button write pin is an output
  pinMode(push_button_write_pin, OUTPUT); 
  // We specify that the push button read pin is an input with pullup resistors activated
  pinMode(push_button_read_pin, INPUT_PULLUP);
  // We specify that the motor driver input one pin is an output
  pinMode(motor_driver_input_1_pin, OUTPUT); 
  // We specify that the motor driver input two pin is an output
  pinMode(motor_driver_input_2_pin, OUTPUT); 
  // We specify that the common cathode of the LED is an output
  pinMode(led_common_cathode_pin, OUTPUT);
  // We specify that the red anode of the LED is an output
  pinMode(led_red_anode_pin, OUTPUT); 
  // We specify that the green anode of the LED is an output
  pinMode(led_green_anode_pin, OUTPUT); 
  // The common cathode of the LED is always kept at GND
  digitalWrite(led_common_cathode_pin, LOW);

  //*****SD CARD CHECK*****
  // We check if the SD card is available to be used
  if(!SD.begin(sd_chip_select_pin)) 
  { 
    // We print an error message stating that the SD card is not inserted
    Serial.println("ERROR - SD card not detected. Waiting till it's inserted...");
    // We wait till the SD card is detected as there is no point going ahead without it
    while(!SD.begin(sd_chip_select_pin)) 
    {
      // We flash the led red to indicate an error
      flash_led_red_for_error();
    }
  }
  // We inform the user that the SD card is intialized
  Serial.println("SD card initialized");
  // We check if we have to remove the file stored in the SD card and start afresh
  if(!sd_data_file_append)
  {
    // We remove the specified file
    SD.remove(sd_data_file_name);
    // We inform the user that we have removed the specified file from the SD card
    Serial.println("Removed the specified file from the SD card");
  }

  //*****RTC CHECK*****
  // We check if the real time clock is available to be used
  if(!rtc.begin()) 
  { 
    // We print an error message stating that the RTC is not detected
    Serial.println("ERROR - RTC not detected. Waiting till it's detected...");
    // We wait till the RTC is detected as there is no point going ahead without it
    while(!rtc.begin())
    {
      // We flash the led red to indicate an error
      flash_led_red_for_error();
    }
  }
  // We inform the user that the RTC is intialized
  Serial.println("RTC initialized");
  // We check if we are to set the RTC timestamp as the compile time
  if(rtc_set_compile_time)
  {
    // The following line sets the RTC to the datetime this sketch was compiled at
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  // We check if the RTC module lost power and thus it's time
  if(rtc.lostPower()) 
  {
    // We inform the user that the RTC has lost its power and that its time should be reset
    Serial.println("ERROR - RTC lost its power and thus it's time. Please reset the time");
    // We run an infinite loop as there is no point continuing
    while(true)
    {
      // We flash the led red to indicate an error
      flash_led_red_for_error();
    }
  }
}

//*************** THE MAIN CODE EXECUTION LOOP ****************

// This is an infinite loop that runs forever
void loop() 
{
  //*****MONITORING BATTERY LEVEL VALUE*****
  // We record the battery level from 0 (corresponds to 5.75V) to 255 (corresponds to 6.6V)
  byte battery_level = byte(min(max(analogRead(battery_level_pin)-588.225,0)*2.933, 255));
  // We activate the red anode of the common cathode LED based on the battery level
  analogWrite(led_red_anode_pin, 255-battery_level);
  // We activate the green anode of the common cathode LED based on the battery level
  analogWrite(led_green_anode_pin, battery_level);

  //*****DRIVING MOTOR IF BUTTON IS PRESSED*****

  // We check if the button is pressed
  if(!push_button_release)
  {
    // We read the value of the speed control potentiometer
    byte motor_speed = floor(analogRead(motor_speed_control_pin)/4.0);
    // We configure the motor driver to activate the cart motor
    analogWrite(motor_driver_input_1_pin, 0);
    analogWrite(motor_driver_input_2_pin, motor_speed);
  }

  //***** RESPONDING TO BUTTON PRESS/RELEASE*****

  // We check if push button was toggled
  if(push_button_release != digitalRead(push_button_read_pin))
  {
    // We update the push button status
    push_button_release = !push_button_release;
    // We inform the user that we are opening up the file in the SD card
    Serial.println("SD card - Opening File");
    // Opens the SD card file to which we write data. Creates one if not existing beforehand
    File sd_data_file = SD.open(sd_data_file_name, FILE_WRITE);
    // We check if the SD card file is accessible
    if(sd_data_file) 
    {
      // We obtain the current datetime from the RTC module
      DateTime current_time = rtc.now();
      // We inform the user that we are writing data to the SD card
      Serial.println("SD card - Writing Data");
      // We check if the push button was pressed
      if(!push_button_release)
      {
        // We inform the user that the push button was pressed
        Serial.println("Push Button - Pressed");
        // The SD data entry for the case of the push button being pressed
        String sd_data_entry = "\nButton Pressed," + String(current_time.year()) + "/" + String(current_time.month()) + "/" + 
                               String(current_time.day()) + "," + String(current_time.hour()) + ":" + 
                               String(current_time.minute()) + ":" + String(current_time.second());
        // We record the arduino internal time in which the button was pressed
        button_press_timer = millis();
        // We write a new line with the button press data to the SD card
        sd_data_file.print(sd_data_entry);
      }
      // The push button was released
      else
      {
        // We inform the user that the push button was released
        Serial.println("Push Button - Released");
        // We configure the motor driver to stop the cart motor
        analogWrite(motor_driver_input_1_pin, 0);
        analogWrite(motor_driver_input_2_pin, 0);
        // The SD data entry for the case of the push button being released
        String sd_data_entry = ", Press Duration," + String(abs(millis() - button_press_timer)) + " milliseconds";
        // We write a new line with the button press data to the SD card
        sd_data_file.print(sd_data_entry);
      }
      // We inform the user that we are closing the SD card file
      Serial.println("SD card - File Closed");
      // We close the SD card file after writing data to it
      sd_data_file.close();
    }
    // if the SD card file can't be open
    else 
    {
      // We inform the user that the required SD card file can't be opened
      Serial.println("ERROR - Can't open the required SD card file");
      // We run an infinite loop as there is no point continuing
      while(true)
      {
        // We flash the led red to indicate an error
        flash_led_red_for_error();
      }
    }  
  }
}

//*************** FUNCTION DEFINITIONS ****************

// This function should be called when an error is encountered. Flashes the red anode of the LED
void flash_led_red_for_error()
{
  // We deactivate the green anode of the common cathode LED
  digitalWrite(led_green_anode_pin, LOW);
  // We activate the red anode of the common cathode LED
  digitalWrite(led_red_anode_pin, HIGH);
  // We wait for half a second
  delay(250);
  // We deactivate the red anode of the common cathode LED  
  digitalWrite(led_red_anode_pin, LOW);
  // We wait for half a second
  delay(250);
}
