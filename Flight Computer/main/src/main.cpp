/*
 *
 * Created by almostlight on Nov 1, 2022 
 * 
 * https://github.com/almostlight 
 *
 * Written for the Pitch v1.0 flight computer 
 * 
 */


//  config
#include "config.h"
//  libraries 
#include <Arduino.h>
#include <fcntl.h> 
#include <stdint.h>
#include <vector> 
#include <numeric> 
//  headers 
#include "bmp.h"
#include "buzzer.h"
#include "filters.h"
#include "imu.h"
#include "led.h"
#include "logger.h"
#include "pid.h"
#include "power.h"
#include "servos.h"
#include "states.h"


using std::vector; 

////    SETUP FUNCTION    //// 

bool STARTUP_ERROR = false;
bool APOGEE = false; 
bool PARACHUTE_EJECTED = false; 

ThreeAxes GyroDriftRate, AccelerationAverage; 
ThreeAxes RawGyroRate, RawAcceleration;  
ThreeAxes Angle, Acceleration; 
double zero_altitude; 

vector<double> pressure_change_vector; 
unsigned long execution_time, loop_index = 0; 
unsigned long start_millis_main, end_millis_main; double delta_millis_main; 
double pressure, altitude; 
double smooth_pressure, pressure_change, pressure_change_avg; 
double acc_magnitude; 
double voltage; 
StateWrapper CurrentState(StateName::WAITING); 
PID pidX(&Record.angleX); 
PID pidZ(&Record.angleZ); 

  //  Initialize filters 
MovingAverageFilter Pressure_MovAvgFilter(20); 
LowPassFilter AccX_LowPass(0.6), AccY_LowPass(0.6), AccZ_LowPass(0.6); 
LowPassFilter Pressure_LowPass(0.6), Altitude_LowPass(0.6); 


void setup() {
  pinMode(VOLTAGE, INPUT); 
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GRN, OUTPUT);
  pinMode(LED_BLU, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(PYRO_1, OUTPUT);
  pinMode(PYRO_2, OUTPUT);
  pinMode(PYRO_3, OUTPUT);
  pinMode(OUT1, OUTPUT); 
  pinMode(OUT2, OUTPUT); 
  pinMode(OUT3, OUTPUT); 
  
  setColor (Black); noTone(BUZZER);  
  setColor (Red); buzzerTone(250); delay(333); 
  setColor (Green); buzzerTone(500); delay(333); 
  setColor (Blue); buzzerTone(1000); delay(333);
  setColor (Black); noTone(BUZZER); 

  Serial.begin(9600);
  //while (!Serial);     
  debugln(F("Started Serial communication"));

  if (PARACHUTE_TEST) {
    delay(1000); 
    countdown(10); 
    setColor(Yellow); 
    if (PYROS_ENABLE) {
      ejectParachutePWM(PYRO_1, 255); 
    }
    setColor(Black); 
    //  stop here 
    while (true); 
  }

  initTvcServos(); 
  testTvcServos(); 

  //  Initialize sensors  
  if (!initIMU(IMU)) {
    STARTUP_ERROR = true; 
  }
  if (!initBMP(BMP)) {
    STARTUP_ERROR = true; 
  }

  //  Initialize data logger if enabled 
  if (LOGGING_ENABLE) { 
    if (!initSDCard()) {
      STARTUP_ERROR = true; 
    }
    if (!createSDLogfile()) {
      STARTUP_ERROR = true; 
    }
    if (!initFlash()) {
      STARTUP_ERROR = true; 
    }
    //  Create recovery file of previous flight on SD 
    writeRecoveryFile(); 

    if (!createFlashLogfile(FILE_SIZE_4M)) {
      STARTUP_ERROR = true; 
    }
    startRecording();   //  opens logfile 
  }

  if (STARTUP_ERROR) {
    setColor (Red); while ( true ); 
  }
  //  Calibrate IMU 
  setColor(White); 
  calibrateIMU(IMU, AccelerationAverage, GyroDriftRate); 
  debug("Average y-acceleration: "); debugln(AccelerationAverage.y); 
  zero_altitude = getAverageAltitude(BMP); 

  debug(F("\nAll devices initialized successfully")); 
  setColor(Green);  
}

////    END OF SETUP FUNCTION    ////

void loop() {
  //  Delta is converted to seconds to save time   
  delta_millis_main = (end_millis_main - start_millis_main)/1000.0; 
  start_millis_main = millis();

  if ( !BMP.performReading() ) debugln(F("Failed to perform reading"));

  voltage = readBatteryVoltage(); 
  altitude = (Altitude_LowPass.getEstimate(BMP.readAltitude( SEALEVELPRESSURE_HPA )) - zero_altitude);    //  in meters 
  pressure = Pressure_LowPass.getEstimate(BMP.pressure);      //  in Pascals 
  smooth_pressure = Pressure_MovAvgFilter.getEstimate(BMP.pressure); 

  IMU.refresh(); 
  IMU.getGyroscope( RawGyroRate.x, RawGyroRate.y, RawGyroRate.z ); 
  IMU.getAcceleration( RawAcceleration.x, RawAcceleration.y, RawAcceleration.z ); 

  Acceleration.x = AccX_LowPass.getEstimate(RawAcceleration.x); 
  Acceleration.y = AccY_LowPass.getEstimate(RawAcceleration.y); 
  Acceleration.z = AccZ_LowPass.getEstimate(RawAcceleration.z); 

  //  start calculating angles and logging data at liftoff 
  if ( CurrentState.state_name != StateName::WAITING ) {
    Angle.x += (RawGyroRate.x - GyroDriftRate.x) * delta_millis_main;    //  multiply angular rates by time delta 
    Angle.y += (RawGyroRate.y - GyroDriftRate.y) * delta_millis_main;    //  in deg/sec * seconds 
    Angle.z += (RawGyroRate.z - GyroDriftRate.z) * delta_millis_main;    //  let's get some roll data! 
  } 

////    STATE ROUTINES    ////

  switch ( CurrentState.state_name ) 
  { 
    case StateName::WAITING:     //  Pre-launch routine 
      if (CHECK_BATTERY) { 
        if (voltage < BATTERY_MIN_VOLTS) {
          debugln(F("Battery near depletion. Suspending.")); 
          if (SUSPEND_ENABLE) powerDown(); 
          else { debugln(F("Suspend disabled in config.")); while ( true ); }
        } 
      } 

      debug(F("Acceleration  X: ")); debug(Acceleration.x); debug(F("\t Y: ")); debug(Acceleration.y); debug(F("\t Z: ")); debug(Acceleration.z); debug(F("\n")); 
      
      if ( Acceleration.y > (AccelerationAverage.y + IDLE_ACCELERATION_MARGIN)   &&   loop_index > 100 ) {      //    detect liftoff at upward acceleration 
        CurrentState.setState(StateName::POWERED_ASCENT); 
      } else if ( STATIC_FIRE && loop_index > 100 ) {
        countdown(COUNTDOWN_TIME_SEC); 
        CurrentState.setState(StateName::POWERED_ASCENT); 
        //  signal to light motor 
        analogWrite(PYRO_3, 255); 
        debugln("Motor ignition.");
      }
    break;


    case StateName::POWERED_ASCENT:     //  TVC routine 

      servo_x.write(SERVO_X_HOME - (pidX.compute() * GEAR_RATIO));
      servo_z.write(SERVO_Z_HOME + (pidZ.compute() * GEAR_RATIO));   //    the positive direction is flipped 

      acc_magnitude = getAccelerationMagnitude(Acceleration.x, Acceleration.y, Acceleration.z); 

      debug(F("X-Axis: ")); debug(Angle.x); debug(F(" deg \t")); debugln(pidX.getCorrection()); 
      debug(F("Z-Axis: ")); debug(Angle.z); debug(F(" deg \t")); debugln(pidZ.getCorrection());    
      debug(F("Pressure: ")); debug(pressure); debug(F(" hPa \t")); debug(F("Altitude: ")); debug(altitude); debugln(F(" m"));
      debug(F("Acc magnitude: ")); debug(acc_magnitude); debugln(F(" m/s^2"));

      if ( !STATIC_FIRE ) {
        if ( (Acceleration.y < (AccelerationAverage.y + IDLE_ACCELERATION_MARGIN)) || ( millis() > (CurrentState.state_change_timestamp + MOTOR_BURN_TIME_LIMIT_MILLIS ) ) ) {     
          servo_x.write(SERVO_X_HOME); 
          servo_z.write(SERVO_Z_HOME); 
          CurrentState.setState(StateName::UNPOWERED_ASCENT); 
        } else if ( fabs(Record.angleX) > ABORT_DEGREE_THRESHOLD || fabs(Record.angleZ) > ABORT_DEGREE_THRESHOLD ) {
          CurrentState.setState(StateName::ABORT); 
        }
      } else if ( STATIC_FIRE && ((millis() - CurrentState.state_change_timestamp) > MOTOR_BURN_TIME_LIMIT_MILLIS )) {
        CurrentState.setState(StateName::DESCENT); 
        analogWrite(PYRO_3, 0); 
        servo_x.write(SERVO_X_HOME); 
        servo_z.write(SERVO_Z_HOME); 
        delay(500); 
      }
    break;


    case StateName::UNPOWERED_ASCENT:     //  Coast routine 
      //  Save power by disabling TVC servos 
      delay(500); 
      detachTvcServos(); 
      
      pressure_change = Pressure_MovAvgFilter.getChange(); 
      debug ("Pressure change: \t"); debugln (pressure_change); 
      pressure_change_vector.push_back(pressure_change); 
      if (pressure_change_vector.size() > 40) { 
        pressure_change_vector.erase(pressure_change_vector.begin()); 
        //  check if total change from previous n measurements is positive 
        if ( accumulate(pressure_change_vector.begin(), pressure_change_vector.end(), 0.0) > 0.0 ) {  
          APOGEE = true; debugln(F("Ejection triggered by pressure"));  
        } else if ( millis() - CurrentState.state_change_timestamp > EJECTION_DELAY_MILLIS ) {
          APOGEE = true; debugln(F("Ejection triggered by delay"));  
        }
      } 

      if ( APOGEE ) {   
        ejectParachutePWM(PYRO_1, 255); 
        PARACHUTE_EJECTED = true; 
      } 

      if ( PARACHUTE_EJECTED ) {
        CurrentState.setState(StateName::DESCENT); 
      }
    break; 


    case StateName::DESCENT:     //  Parachute descent routine 
      if ( checkLanded(RawGyroRate.x, RawGyroRate.y, RawGyroRate.z) || (millis() - CurrentState.state_change_timestamp > 30000)) {  
        CurrentState.setState(StateName::LANDED); 
      }
    break;
    
    
    case StateName::LANDED:     //  Landed routine 
      buzzerTone(2000);  
      if (LOGGING_ENABLE) { 
        stopRecording();    //  closes logfile 
        copyRecordsToSD(); 
      }
      delay(1000); 
      setColor(Black); noTone(BUZZER); 
      if (SUSPEND_ENABLE) { 
        powerDown(); 
      } else { 
        debugln(F("Suspend disabled in config!")); while ( true ); 
      }
    break; 


    case StateName::ABORT: 
      debugln(F("ABORTED")); 
      servo_x.write(SERVO_X_HOME); 
      servo_z.write(SERVO_Z_HOME); 
      setColor(Red); buzzerTone(3000); 
      delay(200);
      setColor(Black); noTone(BUZZER); 

      if ( millis() > (CurrentState.state_change_timestamp + MOTOR_BURN_TIME_LIMIT_MILLIS) ) {     //  check if motor has burned out 
        CurrentState.setState(StateName::UNPOWERED_ASCENT); 
      }
    break; 
  } 

////    END OF STATE ROUTINES   ////

  if ( CurrentState.state_name != StateName::WAITING ) {
    //  Update  record 
    Record.recordNumber++; 
    Record.timeStamp = millis(); 
    Record.machineStateIndex = CurrentState.state_index; 
    Record.voltage = voltage; 
    Record.pressure = pressure; 
    Record.altitude = altitude; 
    Record.accelerationX = Acceleration.x;
    Record.accelerationY = Acceleration.y;
    Record.accelerationZ = Acceleration.z;
    Record.rotationRateX = RawGyroRate.x;
    Record.rotationRateY = RawGyroRate.y;
    Record.rotationRateZ = RawGyroRate.z;
    Record.angleX = Angle.x;
    Record.angleY = Angle.y;
    Record.angleZ = Angle.z;
    Record.pidCorrectionX = pidX.getCorrection();
    Record.pidCorrectionZ = pidZ.getCorrection();

    //  Write struct to local memory 
    if (LOGGING_ENABLE) { 
      if ( !logRecordToFlash(Record) ) { debugln("Failed to write data"); }
    }
  }

  end_millis_main = millis();
  execution_time = (end_millis_main - start_millis_main); 
  loop_index ++; 
  
  //  print battery charge in % 
  debug(F("Battery charge: ")); debug(map(voltage, BATTERY_MIN_VOLTS, BATTERY_MAX_VOLTS, 0, 100)); debugln(F("%")); 
  debug(F("execution time (ms): ")); debugln(execution_time); 
  debugln(); 
}

