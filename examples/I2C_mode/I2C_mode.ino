/*
Date: 5/2021
Author: Elias Santistevan @ SparkFun Electronics
Writing vibration values via I2C to vibrate the motor. 

This vibrates *extremely* vigurously, adhere the motor to something or it will
produce a fault and stop functioning. 

*/

#include <Wire.h>
#include "Haptic_Driver.h"

Haptic_Driver hapDrive;

void setup(){

  Wire.begin();
  Serial.begin(115200);

  if( !hapDrive.begin())
    Serial.println("Could not communicate with Haptic Driver.");
  else
    Serial.println("Press button to activate.");

  if( !hapDrive.defaultMotor() ) 
    Serial.println("Could not set default settings.");

  // Frequency tracking is done by the IC to ensure that the motor is hitting
  // its resonant frequency. I found that restricting the PCB (squeezing)
  // raises an error which stops operation because it can not reach resonance.
  // I disable here to avoid this error. 
  hapDrive.enableFreqTrack(false);

  Serial.println("Setting I2C Operation.");
  hapDrive.setOperationMode(DRO_MODE);
  Serial.println("Ready.");

  delay(1000);

}

void loop(){

  // If uploading often the Haptic Driver IC will throw a fault when the PWM
  // signal is cut off suddenly without being set into inactive mode. Let's
  // clear that error (0x10), just in case.
  event = hapDrive.getIrqEvent();
  Serial.print("Interrupt: ");
  Serial.println(event, HEX);
  Serial.print("Clearing event.");
  hapDrive.clearIrq(event);

  // Max value is 127 with acceleration on (default).
  hapDrive.setVibrate(25);
  delay(500); 
  hapDrive.setVibrate(0); 
  delay(500);

  Serial.println(hapDrive.getIrqEvent(), HEX);

}
