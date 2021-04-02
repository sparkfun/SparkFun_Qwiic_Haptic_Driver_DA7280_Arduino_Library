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

  if( !hapDrive.defaultMotorSettings() ) 
    Serial.println("Could not set default settings.");

  hapDrive.enableAcceleration(true);
  hapDrive.setOperationMode(DRO_MODE);

}

void loop(){

  delay(150);
}
