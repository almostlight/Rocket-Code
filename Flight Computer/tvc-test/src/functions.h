#pragma once 
#include <Arduino.h>
#include <Wire.h>
#include "SerialFlash.h"
#include <ratio>
#include <vector>
#include <stdint.h>
#include "Servo.h"
#include "api/Common.h"
#include "config.h" 
#include "classes.h"
#include "devices.h"


void calibrateIMU(LSM9DS1Class &imu_device, ThreeAxes &accel_error, ThreeAxes &gyro_drift) {
  int initial_millis = millis(); // iteration index  
  int i = 0; 
  float gx, gy, gz, ax, ay, az; // measurements 
  float gyro_x = 0, gyro_y = 0, gyro_z = 0, acc_x = 0, acc_y = 0, acc_z = 0; // sums 
  while ( (millis() - initial_millis) <= CALIBRATION_TIME_MILLIS ) {
    i++; 
    imu_device.readGyroscope(gx, gy, gz);   // get drift rates in degrees/second 
    imu_device.readAcceleration(ax, ay, az);  // get acceleration in g  
    gyro_x += gx; 
    gyro_y += gy; 
    gyro_z += gz;
    acc_x += ax; 
    acc_y += (ay - 1.0); // should be exactly 1 before launch 
    acc_z += az;
  }
  accel_error.x = acc_x / i; 
  accel_error.y = acc_y / i; 
  accel_error.z = acc_z / i; 
  gyro_drift.x = gyro_x / i; 
  gyro_drift.y = gyro_y / i; 
  gyro_drift.z = gyro_z / i; 
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


void setColor(ColorRGB color) {
  analogWrite(LED_RED, 255 - color.r);
  analogWrite(LED_GRN, 255 - color.g);
  analogWrite(LED_BLU, 255 - color.b);
}


bool checkLanded(float gx, float gy, float gz) { 
  float margin = 0.1; 
  if (abs(gx) < margin && abs(gy) < margin && abs(gz) < margin) {
    return true; 
  } else {
    return false; 
  }
}

float getAccelerationMagnitude (float ax, float ay, float az) {
  float magnitude = sqrt(sq(ax) + sq(ay) + sq(az)); 
  return magnitude; 
}


void igniteEngine() {
  digitalWrite(PYRO_1, HIGH);
  digitalWrite(PYRO_2, HIGH);
}


void pyrosLow() {
  digitalWrite(PYRO_1, LOW);
  digitalWrite(PYRO_2, LOW);
}


void testServos(Servo servo_x, Servo servo_z) {
  int servo_delay = 500; 
  servo_x.write(SERVO_X_HOME); 
  servo_z.write(SERVO_Z_HOME); 
  delay(servo_delay); 
  servo_z.write(SERVO_Z_HOME+SERVO_DEG_MIN); delay(servo_delay);
  servo_x.write(SERVO_X_HOME+SERVO_DEG_MIN); delay(servo_delay);
  servo_z.write(SERVO_Z_HOME+SERVO_DEG_MAX); delay(servo_delay);
  servo_x.write(SERVO_X_HOME+SERVO_DEG_MAX); delay(servo_delay);
  servo_z.write(SERVO_Z_HOME+SERVO_DEG_MIN); delay(servo_delay);
  servo_x.write(SERVO_X_HOME+SERVO_DEG_MIN); delay(servo_delay);
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
    FLASH_log.write ( (uint8_t*) &oneRecord, sizeof( oneRecord ) ); 
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


void getNextSDFilename( SDClass sd_card, char* sd_filename_array ) {
  int sd_index = 0;

  do {    // create the filename and see if it exists 
    sprintf( sd_filename_array, "log_%02d.csv", sd_index++ );
    Serial.println( sd_filename_array );
  } while ( (sd_card.exists( sd_filename_array )) && (sd_index < 99) );   //  let's not be absurd  
}


// generate a CSV formatted output of the flash data 
bool copyRecordsToSD( SerialFlashFile &FLASH_log_file, File SD_log_file ) {
  RecordType current_record; 
  if (FLASH_log_file && SD_log_file ) {
    do {
      FLASH_log_file.read((uint8_t*) &current_record, sizeof(current_record));
      if ( current_record.recordNumber != 0xFFFFFFFF ) {    // empty bit (end of data) 
        //Writes to SD
        SD_log_file.print( current_record.recordNumber );
        SD_log_file.print( "," );
        SD_log_file.print( current_record.machineStateIndex );
        SD_log_file.print( "," );
        SD_log_file.print( current_record.timeStamp );
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
    Serial.println(F("Failed to open logfile"));
    return false;
  }
}

