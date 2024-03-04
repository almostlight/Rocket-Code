#pragma once 
#include "config.h"
#include "flash.h"
#include "sdcard.h"
#include <Arduino.h>


struct RecordType {
  unsigned long recordNumber; 
  unsigned long timeStamp; 
  unsigned int machineStateIndex; 
  double voltage; 
  double pressure; 
  double altitude; 
  double accelerationX, accelerationY, accelerationZ; 
  double rotationRateX, rotationRateY, rotationRateZ; 
  double angleX, angleY, angleZ; 
  double pidCorrectionX, pidCorrectionZ;
} Record; 


bool logRecordToFlash ( RecordType currentRecord) {
  if ( flash_logfile.available() ) { 
    flash_logfile.write ( (uint8_t*) &currentRecord, sizeof( currentRecord ) ); 
    return true;
  } 
  return false;
}


// generate a CSV file flash data 
bool copyRecordsToSD() { 
  //  start with cursor at initial position 
  flash_logfile = SerialFlash.open( char_flash_filename ); 
  sd_logfile = SD.open(char_sd_filename, FILE_WRITE);
  RecordType current_record; 
  if (flash_logfile && sd_logfile ) {
    debug(F("Contents of flash log:\n"));
    sd_logfile.seek(sd_logfile.size());
    sd_logfile.println(data_header_string);  
    do {
      flash_logfile.read((uint8_t*) &current_record, sizeof(current_record));
      if ( current_record.recordNumber != 0xFFFFFFFF ) {    // empty bit (end of data) 
        debug( current_record.recordNumber );
        debug( "," );
        debug( current_record.machineStateIndex );
        debug( "," );
        debug( current_record.timeStamp );
        debug( "," );
        debug( current_record.pressure );
        debug( "," );
        debug( current_record.altitude );
        debug( "," );
        debug( current_record.accelerationX );
        debug( "," );
        debug( current_record.accelerationY );
        debug( "," );
        debug( current_record.accelerationZ );
        debug( "," );
        debug( current_record.rotationRateX );
        debug( "," );
        debug( current_record.rotationRateY );
        debug( "," );
        debug( current_record.rotationRateZ );
        debug( "," );
        debug( current_record.angleX );
        debug( "," );
        debug( current_record.angleY );
        debug( "," );
        debug( current_record.angleZ );
        debug( "," );
        debug( current_record.pidCorrectionX );
        debug( "," );
        debug( current_record.pidCorrectionZ );
        debugln(); 
        //Writes to SD
        sd_logfile.print( current_record.recordNumber );
        sd_logfile.print( "," );
        sd_logfile.print( current_record.machineStateIndex );
        sd_logfile.print( "," );
        sd_logfile.print( current_record.timeStamp );
        sd_logfile.print( "," );
        sd_logfile.print( current_record.pressure );
        sd_logfile.print( "," );
        sd_logfile.print( current_record.altitude );
        sd_logfile.print( "," );
        sd_logfile.print( current_record.accelerationX );
        sd_logfile.print( "," );
        sd_logfile.print( current_record.accelerationY );
        sd_logfile.print( "," );
        sd_logfile.print( current_record.accelerationZ );
        sd_logfile.print( "," );
        sd_logfile.print( current_record.rotationRateX );
        sd_logfile.print( "," );
        sd_logfile.print( current_record.rotationRateY );
        sd_logfile.print( "," );
        sd_logfile.print( current_record.rotationRateZ );
        sd_logfile.print( "," );
        sd_logfile.print( current_record.angleX );
        sd_logfile.print( "," );
        sd_logfile.print( current_record.angleY );
        sd_logfile.print( "," );
        sd_logfile.print( current_record.angleZ );
        sd_logfile.print( "," );
        sd_logfile.print( current_record.pidCorrectionX );
        sd_logfile.print( "," );
        sd_logfile.print( current_record.pidCorrectionZ );
        sd_logfile.println(); 
      }
    } while ( current_record.recordNumber != 0xFFFFFFFF );
    //Saves the files
    debugln("Data written to SD");
    flash_logfile.close(); 
    sd_logfile.flush();
    sd_logfile.close();
    return true;
  }
  else {
    debugln(F("Failed to open logfile"));
    return false;
  }
}


bool writeRecoveryFile() {
  char char_recovery_file[64]; 
  uint8_t rec_index = 0;
  File rec_file; 
  RecordType current_record; 
  do {    // create the file name and see if it exists 
    sprintf( char_recovery_file, "recovery_%02d.csv", rec_index++ );
  } while ( (SD.exists( char_recovery_file )) && (rec_index < 99) );   //  let's not be absurd  

  rec_file = SD.open( char_recovery_file, FILE_WRITE);
  if (!rec_file) {
    debugln(F("Error opening SD recovery file."));
    return false; 
  }

  rec_file.println(data_header_string);  
  flash_logfile = SerialFlash.open( char_flash_filename ); 

  if ( flash_logfile && rec_file ) {
    do {
      flash_logfile.read((uint8_t*) &current_record, sizeof(current_record));
      if ( current_record.recordNumber != 0xFFFFFFFF ) {    // empty bit (end of data) 
        rec_file.print( current_record.recordNumber );
        rec_file.print( "," );
        rec_file.print( current_record.machineStateIndex );
        rec_file.print( "," );
        rec_file.print( current_record.timeStamp );
        rec_file.print( "," );
        rec_file.print( current_record.pressure );
        rec_file.print( "," );
        rec_file.print( current_record.altitude );
        rec_file.print( "," );
        rec_file.print( current_record.accelerationX );
        rec_file.print( "," );
        rec_file.print( current_record.accelerationY );
        rec_file.print( "," );
        rec_file.print( current_record.accelerationZ );
        rec_file.print( "," );
        rec_file.print( current_record.rotationRateX );
        rec_file.print( "," );
        rec_file.print( current_record.rotationRateY );
        rec_file.print( "," );
        rec_file.print( current_record.rotationRateZ );
        rec_file.print( "," );
        rec_file.print( current_record.angleX );
        rec_file.print( "," );
        rec_file.print( current_record.angleY );
        rec_file.print( "," );
        rec_file.print( current_record.angleZ );
        rec_file.print( "," );
        rec_file.print( current_record.pidCorrectionX );
        rec_file.print( "," );
        rec_file.print( current_record.pidCorrectionZ );
        rec_file.print( "\n" );
      }
    } while ( current_record.recordNumber != 0xFFFFFFFF );    // empty bit (end of data) 

    debugln("Recovery data written to SD");
    rec_file.close();
    flash_logfile.close(); 
    return true;
  }
  else {
    debugln(F("Failed to open recovery file"));
    return false;
  }
}

