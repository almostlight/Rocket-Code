#pragma once 
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include "servos.h"

// saves power 
void powerDown() {
  servo_x.detach(); 
  servo_z.detach(); 

  setColor (Black); noTone(BUZZER);  
  setColor (Blue); buzzerTone(1000); delay(333);
  setColor (Green); buzzerTone(500); delay(333); 
  setColor (Red); buzzerTone(250); delay(333); 
  setColor (Black); noTone(BUZZER); 
  
  Serial.end(); 
  while (true) delay(9999);  
}

void ejectParachutePWM(unsigned int pin, unsigned int pulse_width) {
  if (PYROS_ENABLE) {
    analogWrite(pin, pulse_width);
    delay(1000); 
    digitalWrite(pin, LOW);
  } else {
    debugln("PYROS disabled in config!"); 
  }
}

// returns battery voltage in volts 
float readBatteryVoltage () {
  float battery_voltage = analogRead(VOLTAGE) * (3.3/1023.0) * 11.0; 
  return battery_voltage; 
}

void countdown ( int seconds ) {
  int a = 500;
  for (int i=seconds; i>0; i--) {
    debug("T - "); debugln(i); 
    a = -a; 
    buzzerTone(1500 + a); setColor(Red); 
    delay(200); 
    noTone(BUZZER); setColor(Black);
    delay(800); 
  }
}
