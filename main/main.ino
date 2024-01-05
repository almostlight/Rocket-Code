/*
 *
 * Created by almostlight on Nov 1, 2022 
 *
 * https://github.com/almostlight 
 * 
 */

#include <Wire.h>
#include <SPI.h> 
#include <Servo.h>
#include <Adafruit_BMP3XX.h>
#include <BMP3.h>
#include <BMP3_defs.h>
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

////    SETUP FUNCTION    //// 


String data_header_string = "# RecordNumber, Timestamp, MachineState, Pressure, Altitude, AccX, AccY, AccZ, RawX, RawY, RawZ, AngX, AngY, AngZ, PID_X, PID_Y, PID_Z"; 
String flash_filename = "flashlog";
char char_flash_filename[16]; 
char char_sd_filename[16];


void setup() {
  flash_filename.toCharArray( char_flash_filename, flash_filename.length() );

  pinMode(VOLTAGE, INPUT); 
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GRN, OUTPUT);
  pinMode(LED_BLU, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(PYRO_1, OUTPUT);
  pinMode(PYRO_2, OUTPUT);
  setColor (0, 0, 0); noTone(BUZZER);  
  setColor (100, 0, 0); tone(BUZZER, 250); delay(1000); 
  setColor (0, 100, 0); tone(BUZZER, 500); delay(1000); 
  setColor (0, 0, 100); tone(BUZZER, 1000); delay(1000);
  setColor (0, 0, 0); noTone(BUZZER); 

  servo_x.attach(TVC_X, SERVO_MIN, SERVO_MAX);
  servo_z.attach(TVC_Y, SERVO_MIN, SERVO_MAX);  
  testServos( servo_x, servo_z ); 

  Serial.begin(9600);
  //while (!Serial);      //  wait for serial 
  Serial.println(F("Started Serial communication"));

  //  Initialize IMU 
  if (!IMU.begin()) {
    Serial.println(F("Failed to initialize IMU"));
    if (FLIGHT) setColor (255,0,0); while ( true ); 
  }
  Serial.print(IMU.gyroscopeSampleRate());

  //  Initialize BMP 3XX 
  if (BMP.begin_I2C()) { 
    //  Set up oversampling and filter initialization
    BMP.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    BMP.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    BMP.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    BMP.setOutputDataRate(BMP3_ODR_50_HZ);
    if (!BMP.performReading()) {
      Serial.println(F("Failed to perform reading"));
      if ( FLIGHT ) setColor (255,0,0); while ( true ); 
    }
  } else {
    Serial.println(F("Could not find a valid BMP3XX sensor"));
    if ( FLIGHT ) setColor (255,0,0); while ( true ); 
  }

  //  Initialize flash chip 
  if (!SerialFlash.begin( FLASH_CS )) {
    Serial.println(F("SPI Flash not detected. Check wiring. Maybe you need to pull up WP/IO2 and HOLD/IO3? Freezing..."));
    if ( FLIGHT ) setColor (255,0,0); while ( true );
  }
  flashInfo(); 
  if ( !SerialFlash.exists( "sig" ) && FLIGHT ) {
    eraseFlash();  
  } else {
    Serial.println(F( "Flash signature detected" ));  
  }

  //  Create flash logfile
  if ( SerialFlash.exists( char_flash_filename ) ) {
    FLASH_log = SerialFlash.open( char_flash_filename ); 
    FLASH_log.erase(); 
  } else {
    SerialFlash.createErasable( char_flash_filename, FILE_SIZE_4M ); 
    FLASH_log = SerialFlash.open( char_flash_filename ); 
  }

  if ( FLASH_log ) {
    Serial.println(F("Logfile created on Flash chip.")); 
  } else {
    Serial.println(F("Error opening SD logfile."));
    if ( FLIGHT ) setColor (255,0,0); while ( true ); 
  } 
  FLASH_log.close(); 

    //  Initialize SD card 
    Serial.println(F("Initializing SD card..."));
    if (!SD.begin(SD_CS)) {
      Serial.println(F("Card failed, or not present"));
      if ( FLIGHT ) setColor (255,0,0); while ( true ); 
    } else {
    Serial.println(F("Card initialized."));
    }
    
    getNextSDFilename( SD, char_sd_filename ); 
    SD_log = SD.open( char_sd_filename, FILE_WRITE);
    if (SD_log) {
      Serial.println(F("Logfile created on SD card."));
      SD_log.println(data_header_string);  
    } else {
      Serial.println(F("Error opening SD logfile."));
      if (FLIGHT) setColor (255,0,0); while ( true ); 
    }
    SD_log.close(); 

  //  Finished initialization 
  Serial.print(F("\nAll devices initialized successfully")); 

  //  Calibrate IMU 
  setColor(255,255,255); 
  calibrateIMU(IMU, CALIBRATION_TIME_SECONDS, DRIFT_X, DRIFT_Y, DRIFT_Z); 
  Serial.print(F("\t X: ")); Serial.print(DRIFT_X); Serial.print(F("\t Y: ")); Serial.print(DRIFT_Y); Serial.print(F("\t Z: ")); Serial.print(DRIFT_Z); Serial.print(F("\n"));
  Serial.println(F("\nFinished Calibration")); 
  setColor(0,255,0);  
}


////    END OF SETUP FUNCTION    ////


bool PARACHUTE_EJECTED = false; 
State currentState = State::WAITING; 
Axis xAxis, yAxis, zAxis; 
RecordType Record; 
uint32_t pressure_decrease_iterator = 0; 
uint32_t execution_sum, loop_index = 0; 
uint32_t start_millis_main, end_millis_main; float delta_millis_main; 
uint32_t liftoff_timestamp = 0, coasting_start_timestamp = 0; 
float pressure, prev_pressure, altitude; 
float raw_gyro_x, raw_gyro_y, raw_gyro_z; 
float raw_acc_x, raw_acc_y, raw_acc_z; 
float local_acc_x, local_acc_y, local_acc_z; 
float idle_acc_y; 
float x_correction, z_correction; 

void loop() {
  //  Delta is converted to seconds to save time   
  delta_millis_main = (end_millis_main - start_millis_main)/1000.0; 
  start_millis_main = millis();

  while ( !IMU.accelerationAvailable() || !IMU.gyroscopeAvailable() );    //  this will pause measurements until new ones are available 
  if ( !BMP.performReading() ) Serial.println(F("Failed to perform reading"));
  altitude = BMP.readAltitude( SEALEVELPRESSURE_HPA );    //  in meters 
  pressure = BMP.pressure;      //  in Pascals 
  IMU.readGyroscope( raw_gyro_x, raw_gyro_y, raw_gyro_z ); 
  IMU.readAcceleration( raw_acc_x, raw_acc_y, raw_acc_z ); 

  if ( loop_index == 0 ) {
    idle_acc_y = raw_acc_y; 
  }

  if ( currentState != State::WAITING ) {
    xAxis.angle += (raw_gyro_x - DRIFT_X) * delta_millis_main;    //  multiply angular rates by time delta 
    yAxis.angle += (raw_gyro_y - DRIFT_Y) * delta_millis_main;    //  in deg/sec * seconds 
    zAxis.angle += (raw_gyro_z - DRIFT_Z) * delta_millis_main;    //  let's get some roll data! 

    //  Update  record 
    Record.recordNumber++; 
    Record.machineStateIndex = static_cast<int>(currentState); 
    Record.timeStamp = millis(); 
    Record.pressure = pressure; 
    Record.altitude = altitude; 
    Record.accelerationX = raw_acc_x;
    Record.accelerationY = raw_acc_y;
    Record.accelerationZ = raw_acc_z;
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


  switch ( currentState ) 
  { 
    case State::WAITING:     //  Pre-launch routine 
      Serial.print(F("Acceleration  X: ")); Serial.print(raw_acc_x); Serial.print(F("\t Y: ")); 
      Serial.print(raw_acc_y); Serial.print(F("\t Z: ")); Serial.print(raw_acc_z); Serial.print(F("\n")); 
      // Serial.print(F("Battery voltage: ")); Serial.println(F( readVoltage( VOLTAGE ) * 11 )); 
      
      if ( raw_acc_y < (idle_acc_y - 0.1) ) {      //    detect liftoff 
        delay(100);         //    short delay to prevent accidental detection 
        IMU.readAcceleration( local_acc_x, local_acc_y, local_acc_z );
        if ( local_acc_y < (idle_acc_y - 0.1) ) {
          FLASH_log = SerialFlash.open( char_flash_filename ); 
          currentState = State::ASCENT; setColor (255,255,0); liftoff_timestamp = millis(); 
        }
      }
    break;


    case State::ASCENT:     //  TVC routine 
      x_correction = xAxis.pid();
      z_correction = zAxis.pid();

      servo_x.write(SERVO_X_HOME + x_correction);
      servo_z.write(SERVO_Z_HOME - z_correction);   //    the positive direction is flipped 

      Serial.print(F("X-Axis: ")); Serial.print(xAxis.angle); Serial.print(F("\t")); Serial.print(x_correction/GEAR_RATIO); Serial.print(F("\t")); 
      Serial.print(F("Z-Axis: ")); Serial.print(zAxis.angle); Serial.print(F("\t")); Serial.print(z_correction/GEAR_RATIO); Serial.print(F("\t"));       
      Serial.print(F("Pressure = ")); Serial.print(pressure); Serial.print(F(" hPa \t")); Serial.print(F("Altitude = ")); Serial.print(altitude); Serial.println(F("m"));

      if ( raw_acc_y > 1.2 /* horribly arbitrary */ || millis() > (liftoff_timestamp + MOTOR_BURN_TIME_MILLIS) ) {     //  check if acceleration is negative again 
        servo_x.write(SERVO_X_HOME); 
        servo_z.write(SERVO_Z_HOME); 
        currentState = State::COASTING; setColor (0,255,255); coasting_start_timestamp = millis();  
      } else if ( abs(xAxis.angle) > ABORT_DEGREE_THRESHOLD || abs(zAxis.angle) > ABORT_DEGREE_THRESHOLD ) {
        currentState = State::ABORT; 
      }
    break;


    case State::COASTING:     //  Coast routine 
      prev_pressure = pressure; 
      delay(200);                     //  stupid arbitrary delay 
      if ( ( BMP.readPressure() - 0.5 ) > prev_pressure ) {     //  check if pressure is increasing 
        pressure_decrease_iterator ++; 
      } else {
        pressure_decrease_iterator = 0; 
      }
      if ( pressure_decrease_iterator > 5 ) {     //  eject if pressure has increased n times 
        ejectParachute(); PARACHUTE_EJECTED = true; 
        Serial.println(F("Ejection triggered by pressure"));  
      } else if ( abs(xAxis.angle) > PARACHUTE_DEGREE_THRESHOLD || abs(zAxis.angle) > PARACHUTE_DEGREE_THRESHOLD ) {     //  check if pressure has increased over 3 times 
        ejectParachute(); PARACHUTE_EJECTED = true; 
        Serial.println(F("Ejection triggered by orientation")); 
      } else if ( millis() - coasting_start_timestamp > EJECTION_DELAY_MILLIS ) {
        ejectParachute(); PARACHUTE_EJECTED = true;  
        Serial.println(F("Ejection triggered by delay"));  
      }
      if ( PARACHUTE_EJECTED ) {
        servo_x.detach();     //  Save power by disabling TVC servos 
        servo_z.detach(); 
        currentState = State::DESCENT; setColor (0,0,255); 
        break; 
      }
    break; 
  

    case State::DESCENT:     //  Parachute descent routine 
      delay(500); 
      IMU.readAcceleration( local_acc_x, local_acc_y, local_acc_z ); 
      if ( abs( raw_acc_x - local_acc_x ) < IDLE_ACCELERATION_MARGIN && abs( raw_acc_y - local_acc_y ) < IDLE_ACCELERATION_MARGIN && abs( raw_acc_z - local_acc_z ) < IDLE_ACCELERATION_MARGIN ) {
        currentState = State::LANDED; setColor (255,0,255); 
      }
    break;
    
    
    case State::LANDED:     //  Landed routine 
      tone(BUZZER, 2000); 
      //  Copy logfile from Flash to SD 
      FLASH_log = SerialFlash.open( char_flash_filename ); 
      SD_log = SD.open(char_sd_filename, FILE_WRITE);
      copyRecordsToSD( FLASH_log, SD_log );   //  wait until finished 
      FLASH_log.close(); 
      SD_log.flush(); 
      SD_log.close(); 
      SD.end();         //  unmount SD card 
      setColor(0, 0, 0); noTone(BUZZER); 
      if (SUSPEND_ENABLE) {
        delay(100); 
        powerDown(); 
      } 
      else Serial.println(F("Suspend disabled in config!")); while ( true ); 
    break; 


    case State::ABORT: 
      Serial.println(F("ABORTED")); 
      servo_x.write(SERVO_X_HOME); 
      servo_z.write(SERVO_Z_HOME); 
      setColor(255, 0, 0); tone(BUZZER, 3000); 
      delay(200);
      setColor(0, 0, 0); noTone(BUZZER); 

      if ( millis() > (liftoff_timestamp + MOTOR_BURN_TIME_MILLIS) ) {     //  check if motor has burned out 
        currentState = State::COASTING; setColor (0,255,255); coasting_start_timestamp = millis(); 
      }
    break; 
  } 


////    END OF STATE ROUTINES   ////


  if (ADC_ENABLE) { 
    if (readVoltage(VOLTAGE) < MILIVOLT_THRESHOLD) {
      Serial.println(F("Battery near depletion. Suspending.")); 
      setColor (255, 0, 0); 
      if (SUSPEND_ENABLE) powerDown(); 
      else Serial.println(F("Suspend disabled in config!")); while ( true ); 
    } 
  } 

  end_millis_main = millis();
  execution_sum += (end_millis_main - start_millis_main); 
  loop_index ++; 

  Serial.print(F("Timestamp: ")); Serial.println(start_millis_main); 
  Serial.print(F("  Average execution time (ms) is: ")); Serial.println(execution_sum/loop_index); 
}

