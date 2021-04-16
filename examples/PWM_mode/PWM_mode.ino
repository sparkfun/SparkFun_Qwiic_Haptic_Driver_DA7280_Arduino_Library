#include <Wire.h>
#include "Haptic_Driver.h"

Haptic_Driver hapDrive;
int event;
int pwmPin = 5;
int intPin = 4;
int power = 10;

void setup(){

  pinMode(pwmPin, OUTPUT);
  pinMode(intPin, INPUT);

  Wire.begin();
  Serial.begin(115200);

  if( !hapDrive.begin())
    Serial.println("Could not communicate with Haptic Driver.");

  if( !hapDrive.defaultMotor() ) 
    Serial.println("Could not set default settings.");
  else
    Serial.println("Ready.");

  hapDrive.enableAcceleration(false);
  analogWrite(pwmPin, power);
  hapDrive.setOperationMode(PWM_MODE);

}

void loop(){

  if( digitalRead(intPin) == LOW ) {
    Serial.println("Interrupt detected.");
    event = hapDrive.getIrqEvent();

    if( event == E_SEQ_FAULT ){
      Serial.println("PWM Value is incorrect.");
      hapDrive.clearIrq(event);
    }
    else {
      Serial.print("Event: ");
      Serial.println(event, HEX);
    }
  } 

  Serial.print("Vibration value: ");
  Serial.println(hapDrive.getVibrate(), HEX);
}
