#pragma once 
#include "api/Common.h"
#include "SD.h"
#include "Adafruit_BMP3XX.h"
#include "config.h" 
#include "devices.h"


//  State class with enumerated states 
enum class State {
  WAITING,  // Green    0,255,0
  ASCENT,   // Yellow   255,255,0
  COASTING, // Aqua     0,255,255
  DESCENT,  // Blue     0,0,255 
  LANDED,   // Fuchsia  255,0,255
  ABORT     // Red      255,0,0
}; 

struct RecordType {
  uint32_t recordNumber; 
  uint32_t timeStamp; 
  uint32_t machineStateIndex; 
  float pressure; 
  float altitude; 
  float accelerationX, accelerationY, accelerationZ; 
  float rotationRateX, rotationRateY, rotationRateZ; 
  float angleX, angleY, angleZ; 
  float pidCorrectionX, pidCorrectionY;
}; 

//  Define axis class with PID controller method  
class Axis {
  private:
    int last_call_time, current_call_time, delta_time; 
    float error, lastError, rateError, totalError, pid_P, pid_I, pid_D, pid_output; 

  public: 
    float angle = 0;

    float pid() {
      current_call_time = millis();
      //  the function saves the timestamp of the last time it was called, and calculates the timespan between it and the next call 
      delta_time = current_call_time - last_call_time; 
      error = PID_TARGET - angle;
      //  Proportional
      pid_P = KP * error;
      //  Integral
      if (INTEGRAL_ENABLE) {
        totalError += error * delta_time;
        pid_I = KI * totalError;
      }
      //  Derivative
      rateError = (error - lastError) / delta_time;
      pid_D = KD * rateError;
      //  Output
      pid_output = (pid_P + pid_I + pid_D)*GEAR_RATIO;
      lastError = error;
      last_call_time = current_call_time; 

      if (pid_output > SERVO_DEG_MAX) {
        pid_output = SERVO_DEG_MAX; 
      } else if (pid_output < SERVO_DEG_MIN) {
        pid_output = SERVO_DEG_MIN; 
      }
      return pid_output;
    }
};

