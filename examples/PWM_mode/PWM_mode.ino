#include <Wire.h>
#include "Haptic_Driver.h"

Haptic_Driver hapDrive;
int event;
int pwmPin = 5;
int intPin = 4;

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

  analogWrite(pwmPin, 200);
  hapDrive.setOperationMode(PWM_MODE);
}

void loop(){

  if( digitalRead(intPin) == LOW ) {
    Serial.println("Interrupt detected.");
    event = hapDrive.getIrqEvent();

    if( event == E_SEQ_FAULT ){
      Serial.println("PWM Value is incorrect.");
      analogWrite(pwmPin, 0);
      delay(100);
      analogWrite(pwmPin, 150);
      hapDrive.clearIrq(event);
      Serial.print("Operation Mode: ");
      Serial.println(hapDrive.getOperationMode());
    }
  } 

  Serial.print("Vibration value from applied PWM signal: ");
  Serial.println(hapDrive.getVibrate(), HEX);
  
  delay(500);
}
