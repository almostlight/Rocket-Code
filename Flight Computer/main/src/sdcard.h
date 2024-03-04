#pragma once 
#include "config.h"
#include <Arduino.h>
#include <SD.h>

File sd_logfile; 
char char_sd_filename[64]; 
String data_header_string = "RecordNumber, MachineState, Timestamp, Pressure, Altitude, AccX, AccY, AccZ, RawX, RawY, RawZ, AngX, AngY, AngZ, PID_X, PID_Z"; 


void getNextSDFilename() {
  //  start with this file number 
  uint8_t sd_index = 1;
  do {    // create the file name and see if it exists 
    sprintf( char_sd_filename, "log_%02d.csv", sd_index++ );
    debugln( char_sd_filename );
  } while ( (SD.exists( char_sd_filename )) && (sd_index < 99) );   //  let's not be absurd  
}

bool initSDCard() {
  if (!SD.begin(CS_SD)) {
    debugln(F("Card failed, or not present"));
    return false; 
  } 
  debugln(F("Card initialized."));
  return true; 
}

bool createSDLogfile() {
  getNextSDFilename(); 
  sd_logfile = SD.open( char_sd_filename, FILE_WRITE);
  if (!sd_logfile) {
    debugln(F("Error opening SD logfile."));
    return false; 
  }
  sd_logfile.close(); 
  debugln(F("Logfile created on SD card."));
  return true; 
}
