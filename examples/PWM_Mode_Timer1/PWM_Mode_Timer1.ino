/*
Date: 5/2021
Author: Elias Santistevan @ SparkFun Electronics
PWM Mode using TimerOne library in addition to the SparkFun Haptic Driver
Library. 

More information on the TimerOne library can be found here: 
https://www.pjrc.com/teensy/td_libs_TimerOne.html. 
      -----or here----
https://playground.arduino.cc/Code/Timer1/

The Haptic Driver IC requires that the PWM signal frequency given to GPI0/PWM pin is at
least 10kHz. The default PWM methods of analogWrite does not provide a method of controlling 
the frequency of the PWM signal.

This vibrates *extremely* vigurously, adhere the motor to something or it will
produce a fault and stop functioning. 

*/

#include <Wire.h>
#include "Haptic_Driver.h"
#include "TimerOne.h" // You can find more information on the TimerOne library
                      //here https://www.pjrc.com/teensy/td_libs_TimerOne.html. 

Haptic_Driver hapDrive;

int pwmPin = 9; // Timer one ONLY modifies PWM on pins 9 and 10
int power = 20;
int event = 0;

void setup(){

  pinMode(pwmPin, OUTPUT);

  Wire.begin();
  Serial.begin(115200);

  Timer1.initialize(100); // This is the amound of microseconds per period 
                          // (f = 1/t). This sets it to 10kHz. 
  Timer1.start();

  if( !hapDrive.begin())
    Serial.println("Could not communicate with Haptic Driver.");
  else
    Serial.println("Qwiic Haptic Driver DA7280 found!");

  if( !hapDrive.defaultMotor() ) 
    Serial.println("Could not set default settings.");
  else
    Serial.println("Ready.");

  // Frequency tracking is done by the IC to ensure that the motor is hitting
  // its resonant frequency. I found that restricting the PCB (squeezing)
  // raises an error which stops operation because it can not reach resonance.
  // I disable here to avoid this error. 
  hapDrive.enableFreqTrack(false);

  Timer1.pwm(pwmPin, power); // Apply signal before setting to PWM mode. 
  delay(10); // An abundance of caution here =P
  hapDrive.setOperationMode(PWM_MODE);
}

void loop(){
  
  for (int power = 10; power < 255; power++) {

    // If uploading often the Haptic Driver IC will throw a fault when the PWM
    // signal is cut off suddenly without being set into inactive mode. Let's
    // clear that error (0x10), just in case.
    event = hapDrive.getIrqEvent();
    Serial.print("Interrupt: ");
    Serial.println(event, HEX);
    Serial.println("Clearing event.");
    hapDrive.clearIrq(event);
    
    Timer1.pwm(pwmPin, power);
    Serial.print("Applied power: ");
    Serial.println(power);
    Serial.print("Vibration value: ");
    Serial.println(hapDrive.getVibrate());
    delay(100);


  }

}
