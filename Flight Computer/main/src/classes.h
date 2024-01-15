#pragma once 
#include <Arduino.h>
#include <vector> 
#include <numeric>
#include "SD.h"
#include "Adafruit_BMP3XX.h"
#include "config.h" 
#include "devices.h"

using std::vector; 

//  State class with enumerated states 
enum class StateObject {
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


class MovingAverageFilter {     //  Smooth curve but delayed 
    private:
        vector<float> data_vector; 
        float previous_estimate, estimate; 
        uint32_t int_window_size; 
        float sum; 
    public: 
        MovingAverageFilter (int window_size) : previous_estimate(0.0), int_window_size(window_size) {}
        float getPreviousEstimate () { return previous_estimate; }

        float getEstimate (float measurement) {
            // delete oldest datapoint if vector exceeds measurement window size
            if (data_vector.size()>=int_window_size) { data_vector.erase(data_vector.begin()); }
            data_vector.push_back(measurement); 
            sum = accumulate(data_vector.begin(), data_vector.end(), 0.0);   //  sum up all values in vector 
            estimate = sum/(data_vector.size()); 
            previous_estimate = estimate; 
            return estimate;  
        } 
}; 


class LowPassFilter {      //  Faster but worse for derivatives 
    private:
        float previous_estimate, estimate; 
        float alpha;    //  smoothing factor, 0 < alpha < 1 
    public: 
        LowPassFilter (float alpha) : previous_estimate(0.0), alpha(alpha) {}   //  initializer list 
        float getPreviousEstimate () { return previous_estimate; }

        float getEstimate (float measurement) { 
            estimate = alpha*previous_estimate + (1-alpha)*measurement; 
            previous_estimate = estimate; 
            return estimate; 
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
