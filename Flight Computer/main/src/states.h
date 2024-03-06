#pragma once 
#include <Arduino.h>
#include "config.h"

//  StateName class with enumerated states 
enum class StateName {
  WAITING,  // Green    0,255,0
  POWERED_ASCENT,   // Yellow   255,255,0
  UNPOWERED_ASCENT, // Aqua     0,255,255
  DESCENT,  // Blue     0,0,255 
  LANDED,   // Fuchsia  255,0,255
  ABORT     // Red      255,0,0
}; 


enum class SpecialEvents {
  STARTUP,
  LIFTOFF,
  ACCEL_BURNOUT, 
  TIME_BURNOUT, 
  PRESSURE_EJECT, 
  TIME_EJECT, 
  LANDING
}; 


class StateWrapper {
  public: 
    StateName state_name; 
    StateWrapper(StateName i_state): state_name(i_state) {}

    unsigned int state_index; 
    unsigned long state_change_timestamp; 

    void setState ( StateName target_state) {
      state_name = target_state; 
      state_index = static_cast<int>(target_state);

      switch (target_state) {
        case StateName::WAITING: 
          setColor(Green); 
        break; 
        case StateName::POWERED_ASCENT:
          setColor(Yellow); 
        break; 
        case StateName::UNPOWERED_ASCENT:
          setColor (Aqua);
        break; 
        case StateName::DESCENT: 
          setColor(Blue); 
        break; 
        case StateName::LANDED:
          setColor(Fuchsia); 
        break; 
        case StateName::ABORT:
          setColor(Red); 
        break; 
      }
      state_change_timestamp = millis(); 
    }
};

