#pragma once 
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include "servos.h"
#include "buzzer.h"
#include "led.h"


void buzzerAlarmLoop(int delay_millis, int frequency) {
    while (true) {
    delay(delay_millis); 
        for (int i=0; i<3; i++) {
        delay(300); 
        buzzerTone(frequency); 
        setColor(White); 
        delay(100); 
        noBuzzerTone(); 
        setColor(Black); 
        }
    }
}

// saves power 
void powerDown() {
  servo_x.detach(); 
  servo_z.detach(); 
  setColor (Black); noBuzzerTone();  
  setColor (Blue); buzzerTone(1000); delay(333);
  setColor (Green); buzzerTone(500); delay(333); 
  setColor (Red); buzzerTone(250); delay(333); 
  setColor (Black); noBuzzerTone(); 
  Serial.end(); 
  buzzerAlarmLoop(3000, 3000); 
}

void ejectParachute(unsigned int pin) {
  analogWrite(pin, PYRO_PULSE_WIDTH);
  delay(800); 
  analogWrite(pin, 0);
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
    noBuzzerTone(); setColor(Black);
    delay(800); 
  }
}
