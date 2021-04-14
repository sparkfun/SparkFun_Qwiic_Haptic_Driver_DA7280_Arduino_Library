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

  if( !hapDrive.defaultMotor() ) 
    Serial.println("Could not set default settings.");
  else
    Serial.println("Ready.");

  analogWrite(PWM0, 60);

  hapDrive.setOperationMode(PWM_MODE);

}

void loop(){

  event = hapDrive.getIrqEvent();

  if( event == E_SEQ_FAULT ){
    Serial.println("PWM Value is incorrect.");
    hapDrive.clearIrq(event);
  }

  Serial.print("Vibration value from applied PWM signal: ");
  Serial.println(hapDrive.getVibrate(), HEX);
  
  delay(1000);
}
