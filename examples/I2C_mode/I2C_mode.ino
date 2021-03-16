#include <Wire.h>
#include "Haptic_Driver.h"

Haptic_Driver hapDrive;
const uint8_t CAPTOUCH = 2;

void setup(){

  Wire.begin();
  Serial.begin(115200);

  if( !hapDrive.begin())
    Serial.println("Could not communicate with Haptic Driver.");
  else
    Serial.println("Press button to activate.");

  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CAPTOUCH, INPUT);

  if( !hapDrive.setDefaultSettings() ) 
    Serial.println("Could not set default settings.");

  hapDrive.enableAcceleration(true);
  hapDrive.setOperationMode(DRO_MODE);

}

void loop(){

  if( digitalRead(CAPTOUCH) == HIGH ){

    hapDrive.writeI2CWave(25);
    delay(250); 
    hapDrive.writeI2CWave(0); 
    delay(100);

  }
  
  delay(150);
}
