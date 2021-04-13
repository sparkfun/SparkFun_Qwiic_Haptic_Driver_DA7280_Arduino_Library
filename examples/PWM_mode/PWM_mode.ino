#include <Wire.h>
#include "Haptic_Driver.h"

Haptic_Driver hapDrive;
int event;

void setup(){

  pinMode(PWM0, OUTPUT);

  Wire.begin();
  Serial.begin(115200);

  if( !hapDrive.begin())
    Serial.println("Could not communicate with Haptic Driver.");
  else
    Serial.println("Press button to activate.");

  if( !hapDrive.defaultMotorSettings() ) 
    Serial.println("Could not set default settings.");

  analogWrite(PWM0, 5);

  hapDrive.setOperationMode(PWM_MODE);
  

}

void loop(){


  analogWrite(PWM0, 30);
  event = hapDrive.checkIrqEvent();

  if( event ){
    Serial.println(event);
    hapDrive.clearIrq(event);
    Serial.println(hapDrive.checkIrqEvent());
  }
  
  delay(100);
}
