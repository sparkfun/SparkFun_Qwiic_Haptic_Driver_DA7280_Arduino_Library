#include <Wire.h>
#include "Haptic_Driver.h"

Haptic_Driver hapDrive;

void setup(){

  Wire.begin();
  Serial.begin(115200);

  while(!Serial);
  if( !hapDrive.begin())
    Serial.print("Could not communicate with Haptic Driver.");

  pinMode(LED_BUILTIN, OUTPUT);
  if( !hapDrive.setDefaultSettings() ) 
    Serial.println("Could not set default settings.");
  hapDrive.enableAcceleration(true);
  hapDrive.writeI2CWave(10);
  if( !hapDrive.setOperationMode(DRO_MODE))
    Serial.println("Could not begin motor.");

}

void loop(){

  Serial.println("Write wave.");
  hapDrive.writeI2CWave(10);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  hapDrive.writeI2CWave(5);
  delay(1000);

}
