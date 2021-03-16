#include <Wire.h>
#include "Haptic_Driver.h"

Haptic_Driver hapDrive;
hapticSettings qwiicHaptic; 

void setup(){

  Wire.begin();
  Serial.begin(115200);

  while(!Serial);
  if( !hapDrive.begin())
    Serial.println("Could not communicate with Haptic Driver.");
  else
    Serial.println("Get ready!");

  pinMode(LED_BUILTIN, OUTPUT);
  if( hapDrive.setDefaultSettings() ) {
    Serial.println("Set Default Settings.");
    qwiicHaptic = hapDrive.readSettings();
    Serial.print("Nominal Voltage: ");
    Serial.println(qwiicHaptic.nomVolt);
    Serial.print("Absolute Voltage: ");
    Serial.println(qwiicHaptic.absVolt);
    Serial.print("Max Current: ");
    Serial.println(qwiicHaptic.currMax);
    Serial.print("Impedance: ");
    Serial.println(qwiicHaptic.impedance);
  }
  else 
    Serial.println("Could not set default settings.");

  
  hapDrive.setOperationMode(RTWM_MODE);
}

void loop(){

  if( Serial.available() > 0){       
    if( Serial.readStringUntil('\n') == "w" ){
      if( hapDrive.addSnippet(RAMP, 7, 15) )
        Serial.println("Written."); 
      else
        Serial.println("Errored."); 
    }

    if( Serial.readStringUntil('\n') == "p" ){
      if( hapDrive.playFromMemory() )
          Serial.println("Successfully set mode and played from memory");
      Serial.println(hapDrive.checkMemFault());
      //Serial.println(hapDrive.checkIrqEvent());
      Serial.println("Playing.....");
    }
    if( Serial.readStringUntil('\n') == "c" ){
      Serial.println("");
      Serial.println("Clearing interrupts.");
      Serial.println("");
      hapDrive.clearIrq();
    }
  }

  hapDrive.checkDone();

  delay(500);

}
