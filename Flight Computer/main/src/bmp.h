#pragma once 
#include "config.h"
#include <Arduino.h>
#include <Adafruit_BMP3XX.h>
#include <bmp3.h>
#include <bmp3_defs.h>

Adafruit_BMP3XX BMP; 


bool initBMP (Adafruit_BMP3XX &BMP) {
  if (BMP.begin_I2C()) { 
    //  Set up oversampling and filter initialization
    BMP.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    BMP.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    BMP.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    BMP.setOutputDataRate(BMP3_ODR_50_HZ);
    if (!BMP.performReading()) {
      debugln(F("Failed to perform reading"));
      return false; 
    } else {
        return true; 
    }
  } else {
    debugln(F("Could not find a valid BMP3XX sensor"));
    return false; 
  }
}


double getAverageAltitude(Adafruit_BMP3XX &BMP) {
  double sum=0; 
  int i=0;
  while ( i<100 ) {
    i++; 
    if ( !BMP.performReading() ) debugln(F("Failed to perform reading"));
    sum += BMP.readAltitude( SEALEVELPRESSURE_HPA );    //  in meters 
  }
  sum = sum/i; 
  return sum; 
}

