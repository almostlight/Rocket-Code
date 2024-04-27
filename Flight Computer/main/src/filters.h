#pragma once 
#include "config.h"
#include <Arduino.h>

using std::vector; 

class MovingAverageFilter {     //  Smooth curve but delayed 
    private:
        vector<double> data_vector; 
        double previous_estimate, current_estimate; 
        unsigned int int_window_size; 
        double sum, change; 
    public: 
        MovingAverageFilter (unsigned int window_size): int_window_size(window_size) {}
        void setInitialEstimate (double estimate) { current_estimate = estimate; previous_estimate = estimate; }

        double getPreviousEstimate () { return previous_estimate; }
        double getCurrentEstimate () { return current_estimate; }

        double getEstimate (double measurement) {
          previous_estimate = current_estimate; 
          // delete oldest datapoint if vector exceeds measurement window size
          if (data_vector.size()>=int_window_size) { data_vector.erase(data_vector.begin()); }
          data_vector.push_back(measurement); 
          sum = accumulate(data_vector.begin(), data_vector.end(), 0.0);   //  sum up all values in vector 
          current_estimate = sum/(data_vector.size()); 
          return current_estimate;  
        } 

        //  difference between current and previous estimate. 
        //  if positive, value is increasing. 
        double getChange () {
          change = current_estimate - previous_estimate; 
          return change; 
        }
}; 


class LowPassFilter {      //  Faster but worse for derivatives 
    private:
        double previous_estimate, current_estimate; 
        double alpha;    //  smoothing factor, 0 < alpha < 1 
    public: 
        LowPassFilter (double smoothing_factor): alpha(smoothing_factor) {}   //  initializer list 
        void setInitialEstimate (double estimate) { current_estimate = estimate; previous_estimate = estimate; }

        double getPreviousEstimate () { return previous_estimate; }
        double getCurrentEstimate () { return current_estimate; }

        double getEstimate (double measurement) { 
          previous_estimate = current_estimate; 
          current_estimate = alpha*previous_estimate + (1-alpha)*measurement; 
          return current_estimate; 
        } 
}; 

