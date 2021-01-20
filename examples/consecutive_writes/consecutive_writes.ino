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
  hapDrive.setDefaultSettings();
}

void loop(){

  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);

}
