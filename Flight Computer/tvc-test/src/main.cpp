/*
 *
 * Created by almostlight on Nov 1, 2022 
 * 
 * https://github.com/almostlight 
 *
 * Written for the Arduino Nano 33 BLE 
 * 
 */


#define DEBUG 

#ifdef DEBUG 
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else 
#define debug(x)
#define debugln(x)
#endif


#include <Arduino.h>
#include <fcntl.h> 
#include <stdint.h>
#include <vector> 
#include <numeric> 
#include <Wire.h>
#include <SPI.h> 
#include <Servo.h>
#include <Arduino_LSM9DS1.h>
#include <SD.h>
#include <SerialFlash.h> 
//  headers 
#include "classes.h"
#include "functions.h"
#include "config.h"
#include "devices.h"

#define FILE_SIZE_1M   1048576L 
#define FILE_SIZE_4M   4194304L 

using std::vector; 

////    SETUP FUNCTION    //// 


String data_header_string = "# RecordNumber, MachineState, Timestamp, AccX, AccY, AccZ, RawX, RawY, RawZ, AngX, AngY, AngZ, PID_X, PID_Y"; 
char char_flash_filename[32]; 
char char_sd_filename[32]; 
ThreeAxes GyroDriftRates, AccelerationErrors; 


void setup() {
  pinMode(VOLTAGE, INPUT); 
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GRN, OUTPUT);
  pinMode(LED_BLU, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(PYRO_1, OUTPUT);
  pinMode(PYRO_2, OUTPUT);
  setColor (Black); noTone(BUZZER);  
  setColor (Red); tone(BUZZER, 250); delay(1000); 
  setColor (Green); tone(BUZZER, 500); delay(1000); 
  setColor (Blue); tone(BUZZER, 1000); delay(1000);
  setColor (Black); noTone(BUZZER); 

  servo_x.attach(TVC_X, SERVO_MIN, SERVO_MAX);
  servo_z.attach(TVC_Y, SERVO_MIN, SERVO_MAX);  
  testServos( servo_x, servo_z ); 

  Serial.begin(9600);
  debugln(F("Started Serial communication"));

  //  Initialize IMU 
  if (!IMU.begin()) {
    debugln(F("Failed to initialize IMU"));
    if ( FLIGHT ) { setColor (Red); while ( true ); } 
  }

  //  Initialize flash chip 
  sprintf( char_flash_filename, "flashlog" ); 
  if (!SerialFlash.begin( FLASH_CS )) {
    debugln(F("SPI Flash not detected"));
    if ( FLIGHT ) { setColor (Red); while ( true ); }
  }

  //  Create flash logfile 
  flashInfo(); 
  if ( !SerialFlash.exists( "sig" ) && FLIGHT ) {
    eraseFlash();  
  } else {
    debugln(F( "Flash signature detected" ));  
  }
  if ( SerialFlash.exists( char_flash_filename ) ) {
    FLASH_log = SerialFlash.open( char_flash_filename ); 
    FLASH_log.erase(); 
  } else {
    SerialFlash.createErasable( char_flash_filename, FILE_SIZE_4M ); 
    FLASH_log = SerialFlash.open( char_flash_filename ); 
  }

  if ( FLASH_log ) {
    debugln(F("Logfile created on Flash chip.")); 
  } else {
    debugln(F("Error opening SD logfile."));
    if ( FLIGHT ) { setColor (Red); while ( true ); } 
  } 
  FLASH_log.close(); 

    //  Initialize SD card 
    debugln(F("Initializing SD card..."));
    if (!SD.begin(SD_CS)) {
      debugln(F("Card failed, or not present"));
      if ( FLIGHT ) { setColor (Red); while ( true ); } 
    } else {
    debugln(F("Card initialized."));
    }
    
    getNextSDFilename(SD, char_sd_filename); 
    SD_log = SD.open( char_sd_filename, FILE_WRITE);
    delay(100); 
    if (SD_log) {
      debugln(F("Logfile created on SD card."));
      SD_log.println(data_header_string);  
    } else {
      debugln(F("Error opening SD logfile."));
      if ( FLIGHT ) { setColor (Red); while ( true ); }
    }
    SD_log.close(); 

  //  Calibrate IMU 
  setColor(White); 
  calibrateIMU(IMU, AccelerationErrors, GyroDriftRates); 
  debugln(F("\nFinished Calibration")); 

  debugln(F("\nAll devices initialized successfully")); 
  setColor(Green);  
}


////    END OF SETUP FUNCTION    ////


unsigned long execution_time, loop_index = 0; 
unsigned long start_millis_main, end_millis_main; 
float delta_millis_main, liftoff_timestamp; 

float raw_gyro_x, raw_gyro_y, raw_gyro_z; 
float raw_acc_x, raw_acc_y, raw_acc_z; 
float acc_x, acc_y, acc_z, acc_magnitude; 
float x_correction, z_correction; 

RecordType Record; 
StateObject CurrentState = StateObject::WAITING; 
AxisObject xAxis, yAxis, zAxis; 
  //  Initialize filters 
LowPassFilter AccX_LowPass(0.6), AccY_LowPass(0.6), AccZ_LowPass(0.6); 


void loop() {
  //  Delta is converted to seconds to save time   
  delta_millis_main = (end_millis_main - start_millis_main)/1000.0; 
  start_millis_main = millis();
  
  IMU.readGyroscope( raw_gyro_x, raw_gyro_y, raw_gyro_z ); 
  IMU.readAcceleration( raw_acc_x, raw_acc_y, raw_acc_z ); 

  acc_x = AccX_LowPass.getEstimate(raw_acc_x - AccelerationErrors.x); 
  acc_y = AccY_LowPass.getEstimate(raw_acc_y - AccelerationErrors.y); 
  acc_z = AccZ_LowPass.getEstimate(raw_acc_z - AccelerationErrors.z); 

  //  start calculating angles and logging data at liftoff 
  if ( CurrentState == StateObject::BURN ) {
    xAxis.angle += (raw_gyro_x - GyroDriftRates.x) * delta_millis_main;    //  multiply angular rates by time delta 
    yAxis.angle += (raw_gyro_y - GyroDriftRates.y) * delta_millis_main;    //  in deg/sec * seconds 
    zAxis.angle += (raw_gyro_z - GyroDriftRates.z) * delta_millis_main;    //  let's get some roll data! 

    //  Update  record 
    Record.recordNumber++; 
    Record.machineStateIndex = static_cast<int>(CurrentState); 
    Record.timeStamp = millis(); 
    Record.accelerationX = acc_x;
    Record.accelerationY = acc_y;
    Record.accelerationZ = acc_z;
    Record.rotationRateX = raw_gyro_x;
    Record.rotationRateY = raw_gyro_y;
    Record.rotationRateZ = raw_gyro_z;
    Record.angleX = xAxis.angle;
    Record.angleY = yAxis.angle;
    Record.angleZ = zAxis.angle;
    Record.pidCorrectionX = x_correction;
    Record.pidCorrectionY = z_correction;

    logRecordToFlash( Record ); 
  } 


////    STATE ROUTINES    ////


  switch ( CurrentState ) 
  { 
    case StateObject::WAITING:     //  Pre-launch routine 
      // wait 30 seconds 
      for (int i = COUNTDOWN_SEC; i>0; i--) {
        Serial.println(i); 
        delay(1000); 
      }

      FLASH_log = SerialFlash.open( char_flash_filename ); 
      igniteEngine(); 
      CurrentState = StateObject::BURN; setColor (Yellow); liftoff_timestamp = millis(); 
    break;


    case StateObject::BURN:     //  TVC routine 
      delay(15); 
      xAxis.getPID(x_correction);
      zAxis.getPID(z_correction);

      servo_x.write(SERVO_X_HOME + x_correction);
      servo_z.write(SERVO_Z_HOME - z_correction);   //    the positive direction is flipped 

      debug(F("X-Axis: ")); debug(xAxis.angle); debug(F(" \t PID: ")); debugln(x_correction/GEAR_RATIO); 
      debug(F("Z-Axis: ")); debug(zAxis.angle); debug(F(" \t PID: ")); debugln(z_correction/GEAR_RATIO);    

      if ( millis() > (liftoff_timestamp + MAX_BURN_TIME_LIMIT_MILLIS) ) {     
        servo_x.write(SERVO_X_HOME); 
        servo_z.write(SERVO_Z_HOME); 
        CurrentState = StateObject::END; setColor(Fuchsia); 
      } 
    break;


    case StateObject::END:     //  Landed routine 
      pyrosLow(); 
      tone(BUZZER, 2000); 
      //  Copy logfile from Flash to SD 
      FLASH_log = SerialFlash.open( char_flash_filename ); 
      SD_log = SD.open(char_sd_filename, FILE_WRITE);
      copyRecordsToSD( FLASH_log, SD_log );   //  wait until finished 
      FLASH_log.close(); 
      SD_log.flush(); 
      SD_log.close();      //  finish writing to SD card 
      setColor(Black); noTone(BUZZER); 
      delay(1000); 
      powerDown(); 
    break; 
  }


////    END OF STATE ROUTINES   ////


  end_millis_main = millis();
  execution_time = (end_millis_main - start_millis_main); 
  loop_index ++; 
  
  debug(F("execution time (ms): ")); debugln(execution_time); 

}
