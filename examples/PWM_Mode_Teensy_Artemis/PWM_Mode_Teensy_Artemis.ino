/*
Date: 5/2021
Author: Elias Santistevan @ SparkFun Electronics
PWM Mode made easy with built in Teensy and Artemis functions.
Library. 

The Haptic Driver IC requires that the PWM signal frequency given to GPI0/PWM pin is at
least 10kHz. The default PWM methods of analogWrite does not provide a method of controlling 
the frequency of the PWM signal, however both the Teensy and Artemis based
boards do.

This vibrates *extremely* vigurously, adhere the motor to something or it will
produce a fault and stop functioning. 

*/

#include <Wire.h>
#include "Haptic_Driver.h"

Haptic_Driver hapDrive;

//int pwmPin = 5; // Teensy
int pwmPin = 2; // Artemis
int power = 20;
int event = 0;

void setup(){

  pinMode(pwmPin, OUTPUT);
  
  //When using Teensy***************************************
  //analogWriteFrequency(pwmPin, 10000); // Set to 10kHz
  //When using Artemis***************************************
  //analogWriteResolution(8); //This example assumes that analogWriteResolution() is set to 8-bits on the Artemis, uncomment this line if needed
  analogWriteFrameWidth(1100); //Set to 10kHz (e.g. 10909Hz = 12000000/1100); too close to the boundary and the DA7280 will fault

  Wire.begin();
  Serial.begin(115200);

  if( !hapDrive.begin())
    Serial.println("Could not communicate with Haptic Driver.");
  else
    Serial.println("Qwiic Haptic Driver DA7280 found!");

  if( !hapDrive.defaultMotor() ) 
    Serial.println("Could not set default settings.");
  else
    Serial.println("Ready.");

  // Frequency tracking is done by the IC to ensure that the motor is hitting
  // its resonant frequency. I found that restricting the PCB (squeezing)
  // raises an error which stops operation because it can not reach resonance.
  // I disable here to avoid this error. 
  hapDrive.enableFreqTrack(false); 

  analogWrite(pwmPin, power); // Apply the signal before entering PWM mode. 
  delay(10); // An abundance of caution here =P
  hapDrive.setOperationMode(PWM_MODE);
}

void loop(){
  
  // I found that the Haptic Driver stops responding when the
  // applied power is around 254 for the Artemis.
  for (int power = 20; power < 255; power++) {

    // If uploading often the Haptic Driver IC will throw a fault when the PWM
    // signal is cut off suddenly without being set into inactive mode. Let's
    // clear that error (0x10), just in case.
    event = hapDrive.getIrqEvent();
    Serial.print("Interrupt: ");
    Serial.println(event, HEX);
    Serial.println("Clearing event.");
    hapDrive.clearIrq(event);
    
    analogWrite(pwmPin, power);
    Serial.print("Applied power: ");
    Serial.println(power);
    Serial.print("Vibration value: ");
    Serial.println(hapDrive.getVibrate());
    delay(100);

  }

}
