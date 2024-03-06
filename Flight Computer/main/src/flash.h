#pragma once 
#include "config.h"
#include <Arduino.h>
#include <SerialFlash.h> 

#define FILE_SIZE_1M   1048576L 
#define FILE_SIZE_4M   4194304L 

SerialFlashFile signature;  
SerialFlashFile flash_logfile; 
char char_flash_filename[32]; 

bool RECORDING = false; 


void flashInfo() {      //  info about the flash chip 
  uint8_t id[5];
  SerialFlash.readID( id );
  debug(F("Capacity: "));
  debugln(SerialFlash.capacity( id ));
}


void eraseFlash() {     // erase flash and create signature file 
  debugln(F( "ERASING FLASH CHIP" ));
  SerialFlash.eraseAll();
  while (!SerialFlash.ready());; 
  // create a file so the code can check if there's a filesystem 
  SerialFlash.create( "sig", 16 );
  signature = SerialFlash.open( "sig" );
  signature.write( "sgntr", 6 );
  signature.close(); 
  debugln(F("Done"));
}


bool initFlash() {
  if (!SerialFlash.begin( FC_CS )) {
    debugln(F("SPI Flash not detected"));
    return false; 
  } 
  if ( !SerialFlash.exists("sig") || WIPE_FLASH ) {
    eraseFlash();  
  } else {
    debugln(F( "Flash signature detected" ));  
  }
  flashInfo(); 

  return true; 
}


bool createFlashLogfile( int file_size ) {
  if ( SerialFlash.exists( char_flash_filename ) ) {
    flash_logfile = SerialFlash.open( char_flash_filename ); 
    flash_logfile.erase(); 
  } else {
    SerialFlash.createErasable( char_flash_filename, file_size ); 
    flash_logfile = SerialFlash.open( char_flash_filename ); 
  }

  if ( flash_logfile ) {
    debugln(F("Logfile created on Flash chip.")); 
  } else {
    debugln(F("Error opening Flash logfile."));
    return false;
  } 
  flash_logfile.close(); 
  return true; 
}


bool startRecording() {
  flash_logfile = SerialFlash.open( char_flash_filename ); 
  RECORDING = true; 
  return true; 
}


bool stopRecording() {
  flash_logfile.close(); 
  RECORDING = false; 
  return true; 
}

