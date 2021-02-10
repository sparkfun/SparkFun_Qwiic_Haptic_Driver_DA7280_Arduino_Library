#include <Wire.h>
#include "Haptic_Driver.h"

Haptic_Driver hapDrive;

void setup(){

  Wire.begin();
  Serial.begin(115200);

  while(!Serial);
  if( !hapDrive.begin())
    Serial.println("Could not communicate with Haptic Driver.");
  else
    Serial.println("Get ready!");

  pinMode(LED_BUILTIN, OUTPUT);
  if( !hapDrive.setDefaultSettings() ) 
    Serial.println("Could not set default settings.");

}

void loop(){

  if( Serial.available() > 0){       
    if( Serial.readStringUntil('\n') == "w" ){
      if( hapDrive.addSnippet(STEP, 4, 4) )
        Serial.println("Written."); 
      else
        Serial.println("Errored."); 
    }

    if( Serial.readStringUntil('\n') == "p" ){
      hapDrive.setOperationMode(RTWM_MODE);
      hapDrive.playFromMemory();
      Serial.println(hapDrive.checkMemFault());
      Serial.println(hapDrive.checkIrqEvent());
      Serial.println("Playing.....");
    }
  }

  delay(500);

}
