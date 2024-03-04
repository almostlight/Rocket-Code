#pragma once 
#include "config.h"
#include <Arduino.h>
#include <PWMServo.h>
#include <Servo.h>

Servo servo_x; 
Servo servo_z; 


void initTvcServos() {
  servo_x.attach(TVC_X/*, SERVO_MIN, SERVO_MAX*/);
  servo_z.attach(TVC_Y/*, SERVO_MIN, SERVO_MAX*/);  
}


void detachTvcServos() {
  servo_x.detach();     
  servo_z.detach();
/*
  analogWrite(TVC_X, 0);
  analogWrite(TVC_Y, 0);
*/
}


void testTvcServos() {
  int servo_delay = 500; 
  servo_x.write(SERVO_X_HOME); 
  servo_z.write(SERVO_Z_HOME); 
  delay(servo_delay); 
  for (int i=0; i<2; i++) {
    servo_z.write(SERVO_Z_HOME+SERVO_DEG_MIN); delay(servo_delay);
    servo_x.write(SERVO_X_HOME+SERVO_DEG_MIN); delay(servo_delay);
    servo_z.write(SERVO_Z_HOME+SERVO_DEG_MAX); delay(servo_delay);
    servo_x.write(SERVO_X_HOME+SERVO_DEG_MAX); delay(servo_delay);
    servo_z.write(SERVO_Z_HOME+SERVO_DEG_MIN); delay(servo_delay);
    servo_x.write(SERVO_X_HOME+SERVO_DEG_MIN); delay(servo_delay);
  }
  servo_x.write(SERVO_X_HOME); 
  servo_z.write(SERVO_Z_HOME); 
}

