#pragma once 
#include <Arduino.h>
#include "SerialFlash.h"
#include <ratio>
#include "Servo.h"
#include "api/Common.h"
#include "config.h" 
#include "classes.h"
#include "devices.h"


void calibrateIMU(LSM9DS1Class &imu_device, int seconds, float &driftX, float &driftY, float &driftZ) {
  driftX = 0;
  driftY = 0;
  driftZ = 0; 
  float raw_x, raw_y, raw_z;
  float driftSumX = 0, driftSumY = 0, driftSumZ = 0;
  uint i = 0;
  uint start_timestamp = millis(); 
  while (millis()<(start_timestamp+(seconds*1000))) {
    i++; 
    imu_device.readGyroscope(raw_x, raw_y, raw_z);   // get drift rates in degrees/second 
    driftSumX += raw_x; 
    driftSumY += raw_y; 
    driftSumZ += raw_z; 
    driftX = driftSumX / i;   // average it out 
    driftY = driftSumY / i;
    driftZ = driftSumZ / i;
  }
} 


// Power-saving shutdown function 
void powerDown() {
  digitalWrite(LED_PWR, LOW);
  pinMode(PIN_ENABLE_SENSORS_3V3, OUTPUT);
  pinMode(PIN_ENABLE_I2C_PULLUP, OUTPUT);
  digitalWrite(PIN_ENABLE_SENSORS_3V3, LOW);    // turn off sensors
  digitalWrite(PIN_ENABLE_I2C_PULLUP, LOW);
  NRF_POWER->SYSTEMOFF = 1;    // power down uC 
}


void setColor(int r, int g, int b) {
  analogWrite(LED_RED, 255 - r);
  analogWrite(LED_GRN, 255 - g);
  analogWrite(LED_BLU, 255 - b);
}


bool checkLanded(float raw_acceleration_x, float raw_acceleration_y, float raw_acceleration_z) { 
  float margin = 0.05; 
  if (abs(raw_acceleration_x - DRIFT_X) < margin && abs(raw_acceleration_y - DRIFT_Y) < margin && abs(raw_acceleration_z - DRIFT_Z) < margin) {
    return true; 
  } else {
    return false; 
  }
}


void ejectParachute() {
  // Fire pyros!! 
  if (PYROS_ENABLE) {
    digitalWrite(PYRO_1, HIGH);
    digitalWrite(PYRO_2, HIGH);
    delay(2000); 
    digitalWrite(PYRO_1, LOW);
    digitalWrite(PYRO_2, LOW);
  } else {
    Serial.println("Pyros disabled in config!"); 
  }
}


void testServos(Servo servo_x, Servo servo_z) {
  int servo_delay = 500; 
  servo_x.write(SERVO_X_HOME); 
  servo_z.write(SERVO_Z_HOME); 
  delay(servo_delay); 
  for (int i=0; i<4; i++) {
    servo_z.write(SERVO_Z_HOME+SERVO_DEG_MIN); delay(servo_delay);
    servo_x.write(SERVO_X_HOME+SERVO_DEG_MIN); delay(servo_delay);
    servo_z.write(SERVO_Z_HOME+SERVO_DEG_MAX); delay(servo_delay);
    servo_x.write(SERVO_X_HOME+SERVO_DEG_MAX); delay(servo_delay);
    servo_z.write(SERVO_Z_HOME+SERVO_DEG_MIN); delay(servo_delay);
    servo_x.write(SERVO_X_HOME+SERVO_DEG_MIN); delay(servo_delay);
  }
  servo_x.write(SERVO_X_HOME); 
  servo_z.write(SERVO_Z_HOME); 
}


int readVoltage ( int pin ) {
  int milivolts = map(analogRead(pin), 0, 1023, 0, 3300);
  return milivolts; 
}


void flashInfo() {      //  info about the flash chip 
  uint8_t id[5];
  SerialFlash.readID( id );
  Serial.print(F("Capacity: "));
  Serial.println(SerialFlash.capacity( id ));
}


bool logRecordToFlash ( RecordType oneRecord) {
  if ( FLASH_log.available() ) { 
    FLASH_log.write ( ( uint8_t* )&oneRecord, sizeof( oneRecord ) ); 
    return true;
  } else {
    Serial.println( "FAILED TO LOG DATA!" );
    return false;
  }
}


void eraseFlash() {     // erase the flash and create a signature file
  Serial.println(F( "Flash doesn't appear to hold a file system - erasing..." ));
  Serial.println(F( "ERASING FLASH CHIP" ));
  SerialFlash.eraseAll();
  while (!SerialFlash.ready());; 
  // create a file so the code can check if there's a filesystem 
  SerialFlash.create( "sig", 16 );
  signature = SerialFlash.open( "sig" );
  signature.write( "sgntr", 6 );
  signature.close(); 
  Serial.println(F("Done"));
}


uint8_t getNextSDFilename( SDClass sd_card, char* sd_filename_array ) {
  uint8_t sd_index = 0;

  do {    // create the filename and see if it exists 
    sprintf( sd_filename_array, "SD_log_%d.csv", sd_index++ );
    Serial.println( sd_filename_array );
  } while ( (sd_card.exists( sd_filename_array )) && (sd_index < 10) );   //  let's not be absurd  
  
  Serial.println( sd_index );
  return sd_index;
}


// generate a CSV formatted output of the flash data 
bool copyRecordsToSD( SerialFlashFile &FLASH_log_file, File SD_log_file ) {
  RecordType current_record; 
  if (FLASH_log_file && SD_log_file ) {
    Serial.print(F("Contents of flash log:\n"));
    do {
      FLASH_log_file.read((uint8_t*)&current_record, sizeof(current_record));
      if ( current_record.recordNumber != 0xFFFFFFFF ) {    // empty bit (end of data) 
        Serial.print( current_record.recordNumber );
        Serial.print( "," );
        Serial.print( current_record.machineStateIndex );
        Serial.print( "," );
        Serial.print( current_record.timeStamp );
        Serial.print( "," );
        Serial.print( current_record.pressure );
        Serial.print( "," );
        Serial.print( current_record.altitude );
        Serial.print( "," );
        Serial.print( current_record.accelerationX );
        Serial.print( "," );
        Serial.print( current_record.accelerationY );
        Serial.print( "," );
        Serial.print( current_record.accelerationZ );
        Serial.print( "," );
        Serial.print( current_record.rotationRateX );
        Serial.print( "," );
        Serial.print( current_record.rotationRateY );
        Serial.print( "," );
        Serial.print( current_record.rotationRateZ );
        Serial.print( "," );
        Serial.print( current_record.angleX );
        Serial.print( "," );
        Serial.print( current_record.angleY );
        Serial.print( "," );
        Serial.print( current_record.angleZ );
        Serial.print( "," );
        Serial.print( current_record.pidCorrectionX );
        Serial.print( "," );
        Serial.print( current_record.pidCorrectionY );
        Serial.print( "\n" );
        //Writes to SD
        SD_log_file.print( current_record.recordNumber );
        SD_log_file.print( "," );
        SD_log_file.print( current_record.machineStateIndex );
        SD_log_file.print( "," );
        SD_log_file.print( current_record.timeStamp );
        SD_log_file.print( "," );
        SD_log_file.print( current_record.pressure );
        SD_log_file.print( "," );
        SD_log_file.print( current_record.altitude );
        SD_log_file.print( "," );
        SD_log_file.print( current_record.accelerationX );
        SD_log_file.print( "," );
        SD_log_file.print( current_record.accelerationY );
        SD_log_file.print( "," );
        SD_log_file.print( current_record.accelerationZ );
        SD_log_file.print( "," );
        SD_log_file.print( current_record.rotationRateX );
        SD_log_file.print( "," );
        SD_log_file.print( current_record.rotationRateY );
        SD_log_file.print( "," );
        SD_log_file.print( current_record.rotationRateZ );
        SD_log_file.print( "," );
        SD_log_file.print( current_record.angleX );
        SD_log_file.print( "," );
        SD_log_file.print( current_record.angleY );
        SD_log_file.print( "," );
        SD_log_file.print( current_record.angleZ );
        SD_log_file.print( "," );
        SD_log_file.print( current_record.pidCorrectionX );
        SD_log_file.print( "," );
        SD_log_file.print( current_record.pidCorrectionY );
        SD_log_file.print( "\n" );
      }
    } while ( current_record.recordNumber != 0xFFFFFFFF );
    //Saves the files
    Serial.println("\nData written to SD");
    return true;
  }
  else {
    Serial.println(F("Failed to open a file"));
    return false;
  }
}

