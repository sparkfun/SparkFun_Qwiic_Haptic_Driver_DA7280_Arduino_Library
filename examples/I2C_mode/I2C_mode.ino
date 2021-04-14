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

  Serial.println("Setting I2C Operation.");
  hapDrive.setOperationMode(DRO_MODE);
  Serial.println("Ready.");

}

void loop(){

  hapDrive.setVibrate(25);
  delay(500); 
  hapDrive.setVibrate(0); 
  delay(500);

}
