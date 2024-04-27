#pragma once 
#include "config.h"
#include "logger.h"
#include <Arduino.h>


class PID { 
  private:
    bool first_call; 
    unsigned long prev_time, time;
    double delta_t; 
    double error, prev_error, rate_error, total_error, P_term, I_term, D_term; 
    double pid_output;
    double angle; 
    double* angle_ptr; 

  public: 
    PID (double* angle_variable): first_call(true), angle_ptr(angle_variable) {}
    double getCorrection () { return pid_output; }
    double getP () { return P_term; }
    double getI () { return I_term; }
    double getD () { return D_term; }
    
    double compute () {
        //  get value from angle pointer 
        angle = *angle_ptr; 
        if (first_call) {
            first_call = false; 
            prev_time = millis(); 
            prev_error = angle - PID_SETPOINT; 
            pid_output = 0.0f;
        } else {
            time = millis(); 
            delta_t = (time - prev_time) / 1000.0f;     //  get time between calls in seconds 
            error =  angle - PID_SETPOINT;
            //  compute components 
            P_term = error;
            I_term += error * delta_t; 
            D_term = (error - prev_error) / delta_t;
            //  reset integral term if error is/was 0 
            //  this is awesome! 
            if (((error < 0) != (prev_error < 0)) || (error == 0)) {
                I_term = 0.0f; 
            } 
            //  TODO: limit itegral term range to output range 
            //  output
            prev_error = error;
            prev_time = time; 
            pid_output = ((KP * P_term) + (KI * I_term) + (KD * D_term));

            if (pid_output > MOUNT_DEG_MAX) {
                pid_output = MOUNT_DEG_MAX; 
            } else if (pid_output < MOUNT_DEG_MIN) {
                pid_output = MOUNT_DEG_MIN; 
            }
        }
        return pid_output; 
    }
}; 

