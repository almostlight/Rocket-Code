#pragma once 
#include <Arduino.h>
#include <vector> 
#include <numeric>
#include <stdint.h>
#include "SD.h"
#include "config.h" 
#include "devices.h"

using std::vector; 

//  State class with enumerated states 
enum class StateObject {
  WAITING,  // Green    0,255,0
  BURN,   // Yellow   255,255,0
  END,   // Fuchsia  255,0,255
}; 

//  glorified tuple 
struct ThreeAxes {
  float x, y, z;
}; 

struct ColorRGB {
  uint8_t r, g, b;
}; 
ColorRGB White {255,255,255};
ColorRGB Black {0,0,0};
ColorRGB Red {255,0,0};
ColorRGB Green {0,255,0}; 
ColorRGB Blue {0,0,255};
ColorRGB Fuchsia {255,0,255};
ColorRGB Yellow {255,255,0};
ColorRGB Aqua {0,255,255};


struct RecordType {
  unsigned long recordNumber; 
  unsigned long timeStamp; 
  unsigned int machineStateIndex; 
  float accelerationX, accelerationY, accelerationZ; 
  float rotationRateX, rotationRateY, rotationRateZ; 
  float angleX, angleY, angleZ; 
  float pidCorrectionX, pidCorrectionY;
}; 


class LowPassFilter {      //  Faster but worse for derivatives 
    private:
        float previous_estimate, current_estimate; 
        float alpha;    //  smoothing factor, 0 < alpha < 1 
    public: 
        LowPassFilter (float smoothing_factor): alpha(smoothing_factor) {}   //  initializer list 
        float getPreviousEstimate () { return previous_estimate; }
        float getCurrentEstimate () { return current_estimate; }

        float getEstimate (float measurement) { 
          previous_estimate = current_estimate; 
          current_estimate = alpha*previous_estimate + (1-alpha)*measurement; 
          return current_estimate; 
        } 
}; 


class AxisObject {        //  Axis class with PID controller method  
  private:
    int last_call_time, current_call_time, delta_time; 
    float error, lastError, rateError, totalError, pid_P, pid_I, pid_D; 

  public: 
    float angle = 0.0;

    void getPID ( float& pid_output ) {
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
    }
};
